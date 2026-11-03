#include "rb/runtime/transport.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "rb/runtime/json.hpp"

namespace rb::runtime {

namespace {
std::string errorText(const std::string &what) { return what + ": " + std::strerror(errno); }

speed_t baudConstant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default: throw std::runtime_error("unsupported baud");
  }
}

// PIDs that already hold `real` open, found through /proc.
std::set<std::string> deviceOwners(const std::string &real) {
  std::set<std::string> owners;
  DIR *proc = opendir("/proc");
  if (!proc) return owners;
  while (dirent *entry = readdir(proc)) {
    const std::string pid = entry->d_name;
    if (pid.empty() || !std::all_of(pid.begin(), pid.end(), [](unsigned char c) { return std::isdigit(c); })) continue;
    const std::string dir = "/proc/" + pid + "/fd";
    DIR *fds = opendir(dir.c_str());
    if (!fds) continue;
    while (dirent *fd = readdir(fds)) {
      char target[512];
      ssize_t n = readlink((dir + "/" + fd->d_name).c_str(), target, sizeof target - 1);
      if (n > 0 && std::string(target, static_cast<std::size_t>(n)) == real) owners.insert(pid);
    }
    closedir(fds);
  }
  closedir(proc);
  return owners;
}
}  // namespace

int openSerial(const std::string &path, int baud) {
  char resolved[4096];
  const std::string real = realpath(path.c_str(), resolved) ? resolved : path;
  const auto owners = deviceOwners(real);
  if (!owners.empty()) {
    std::string list;
    for (const auto &pid : owners) list += (list.empty() ? "" : ",") + pid;
    throw std::runtime_error("UART is already open by PID(s): " + list);
  }
  const speed_t speed = baudConstant(baud);
  const int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) throw std::runtime_error(errorText("open " + path));
  try {
    if (ioctl(fd, TIOCEXCL) < 0) throw std::runtime_error(errorText("TIOCEXCL"));
    termios attrs{};
    if (tcgetattr(fd, &attrs) < 0) throw std::runtime_error(errorText("tcgetattr"));
    attrs.c_iflag = attrs.c_oflag = attrs.c_lflag = 0;
    attrs.c_cflag = CS8 | CREAD | CLOCAL;
    cfsetispeed(&attrs, speed);
    cfsetospeed(&attrs, speed);
    attrs.c_cc[VMIN] = 0;
    attrs.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &attrs) < 0) throw std::runtime_error(errorText("tcsetattr"));
    tcflush(fd, TCIOFLUSH);
    return fd;
  } catch (...) {
    ::close(fd);
    throw;
  }
}

void Writer::add(const std::vector<hcr::Frame> &frames) {
  for (const auto &frame : frames) {
    const auto raw = frame.encode();
    pending_.insert(pending_.end(), raw.begin(), raw.end());
  }
  if (pending_.size() > 4096) throw std::runtime_error("output transport backpressure");
}

void Writer::flush() {
  if (pending_.empty()) return;
  const ssize_t n = ::write(fd_, pending_.data(), pending_.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
    throw std::runtime_error(errorText("write"));
  }
  pending_.erase(pending_.begin(), pending_.begin() + n);
}

DashboardProcess::DashboardProcess(std::string root, std::string host, int port, std::string token_file,
                                   std::string status_file, std::string command_socket)
    : root_(std::move(root)), host_(std::move(host)), token_file_(std::move(token_file)),
      status_file_(std::move(status_file)), command_socket_(std::move(command_socket)),
      requested_port_(port) {}

void DashboardProcess::start() {
  if (pid_ > 0) throw std::runtime_error("dashboard process already started");
  int out[2];
  if (pipe(out) < 0) throw std::runtime_error(errorText("pipe"));
  const char *python = std::getenv("RB_PYTHON");
  const std::string interpreter = python && *python ? python : "python3";
  const std::string page = root_ + "/web/index.html";
  std::vector<std::string> args = {interpreter, "-u", "-m", "rb_runtime.dashboard", "--host", host_,
                                   "--port", std::to_string(requested_port_), "--token-file", token_file_,
                                   "--status-file", status_file_, "--command-socket", command_socket_,
                                   "--page-file", page};
  std::vector<char *> argv;
  for (auto &a : args) argv.push_back(a.data());
  argv.push_back(nullptr);
  const std::string pythonpath = root_ + "/src" + (std::getenv("PYTHONPATH") ? ":" + std::string(std::getenv("PYTHONPATH")) : "");
  pid_ = fork();
  if (pid_ < 0) {
    ::close(out[0]);
    ::close(out[1]);
    throw std::runtime_error(errorText("fork"));
  }
  if (pid_ == 0) {
    dup2(out[1], STDOUT_FILENO);
    const int null = open("/dev/null", O_RDONLY);
    if (null >= 0) dup2(null, STDIN_FILENO);
    for (int fd = 3; fd < 1024; ++fd) ::close(fd);  // nothing of the control process leaks
    setenv("PYTHONPATH", pythonpath.c_str(), 1);
    execvp(argv[0], argv.data());
    _exit(127);
  }
  ::close(out[1]);
  try {
    // Startup only, before entering the control loop: never wait for HTTP from the
    // telemetry path, or restart the web synchronously there.
    pollfd p{out[0], POLLIN, 0};
    if (poll(&p, 1, 5000) <= 0) throw std::runtime_error("dashboard process startup timeout");
    std::string line;
    char c;
    while (line.size() < 1024 && ::read(out[0], &c, 1) == 1 && c != '\n') line += c;
    const Json result = Json::parse(line);
    if (const Json *error = result.find("error")) throw std::runtime_error(error->dump());
    port_ = static_cast<int>(result.at("port").integer());
    ::close(out[0]);
  } catch (...) {
    ::close(out[0]);
    close();
    throw;
  }
}

bool DashboardProcess::alive() {
  if (pid_ <= 0) return false;
  int status = 0;
  return waitpid(pid_, &status, WNOHANG) == 0;
}

void DashboardProcess::close() {
  if (pid_ <= 0) return;
  kill(pid_, SIGTERM);
  int status = 0;
  for (int i = 0; i < 40; ++i) {
    if (waitpid(pid_, &status, WNOHANG) != 0) {
      pid_ = -1;
      return;
    }
    usleep(50000);
  }
  kill(pid_, SIGKILL);
  waitpid(pid_, &status, 0);
  pid_ = -1;
}

}  // namespace rb::runtime

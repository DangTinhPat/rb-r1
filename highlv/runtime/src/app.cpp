// Process shell around HybridCore: UART/socket transport, operator command socket,
// status file and dashboard child. Port of rb_runtime/app.py.
#include "rb/runtime/app.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <memory>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

#include "rb/runtime/commands.hpp"
#include "rb/runtime/hybrid_core.hpp"
#include "rb/runtime/json.hpp"
#include "rb/runtime/logic.hpp"
#include "rb/runtime/policy.hpp"
#include "rb/runtime/transport.hpp"

namespace rb::runtime {

namespace {

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

double monotonic() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void log(const std::string &event, Json data = Json::object()) {
  Json line = Json::object();
  line.set("event", event);
  for (const auto &m : data.members()) line.set(m.first, m.second);
  std::fprintf(stderr, "%s\n", line.dump().c_str());
  std::fflush(stderr);
}

std::string readFile(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::string sysError(const std::string &what) { return what + ": " + std::strerror(errno); }

void setNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) throw std::runtime_error(sysError("fcntl"));
}

struct Options {
  std::string config, serial, socket, backend, status_file;
  std::string command_socket = "/tmp/rb-hybrid-command.sock";
  std::string web_host = "127.0.0.1", web_token_file = "/tmp/rb-hybrid-web.token";
  int listen_tcp = -1, web_port = 8080;
  int realtime_priority = -1;
  bool no_web = false;
};

Options parseOptions(int argc, char **argv, const std::string &root) {
  Options o;
  o.config = root + "/config/runtime.json";
  auto value = [&](int &i) -> std::string {
    if (i + 1 >= argc) throw std::invalid_argument(std::string("missing value for ") + argv[i]);
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config") o.config = value(i);
    else if (a == "--serial") o.serial = value(i);
    else if (a == "--socket") o.socket = value(i);
    else if (a == "--listen-tcp") o.listen_tcp = std::stoi(value(i));
    else if (a == "--backend") o.backend = value(i);
    else if (a == "--status-file") o.status_file = value(i);
    else if (a == "--command-socket") o.command_socket = value(i);
    else if (a == "--web-host") o.web_host = value(i);
    else if (a == "--web-port") o.web_port = std::stoi(value(i));
    else if (a == "--web-token-file") o.web_token_file = value(i);
    else if (a == "--no-web") o.no_web = true;
    else if (a == "--realtime-priority") o.realtime_priority = std::stoi(value(i));
    else throw std::invalid_argument("unknown option " + a);
  }
  return o;
}

// Remove a stale unix socket, but never a regular file or a socket someone serves.
void clearSocketPath(const std::string &path, const char *what) {
  struct stat st;
  if (stat(path.c_str(), &st) != 0) return;
  if (!S_ISSOCK(st.st_mode)) throw std::runtime_error(std::string(what) + " path occupied by a regular file");
  const int probe = socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
  const bool served = connect(probe, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0;
  ::close(probe);
  if (served) throw std::runtime_error(std::string(what) + " is already served");
  unlink(path.c_str());
}

int listenUnix(const std::string &path, int backlog) {
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) throw std::runtime_error(sysError("socket"));
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof addr.sun_path) throw std::runtime_error("socket path too long");
  std::strcpy(addr.sun_path, path.c_str());
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0) {
    ::close(fd);
    throw std::runtime_error(sysError("bind " + path));
  }
  chmod(path.c_str(), 0600);
  if (listen(fd, backlog) < 0) {
    ::close(fd);
    throw std::runtime_error(sysError("listen"));
  }
  setNonBlocking(fd);
  return fd;
}

int listenLoopback(int port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) throw std::runtime_error(sysError("socket"));
  const int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0 || listen(fd, 1) < 0) {
    ::close(fd);
    throw std::runtime_error(sysError("bind loopback"));
  }
  setNonBlocking(fd);
  return fd;
}

void sendAll(int fd, const std::string &text) {
  std::size_t sent = 0;
  while (sent < text.size()) {
    const ssize_t n = send(fd, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) throw std::runtime_error(sysError("send"));
    sent += static_cast<std::size_t>(n);
  }
}

void replyError(int fd, const std::string &error) {
  Json r = Json::object();
  r.set("accepted", false);
  r.set("error", error);
  try { sendAll(fd, r.dump() + "\n"); } catch (const std::exception &) {}
}

void writeStatus(const std::string &path, const Json &snapshot) {
  const std::string temp = path + ".tmp";
  {
    std::ofstream out(temp, std::ios::binary | std::ios::trunc);
    out << snapshot.dump(2) << "\n";
    if (!out) throw std::runtime_error("cannot write " + temp);
  }
  if (rename(temp.c_str(), path.c_str()) != 0) throw std::runtime_error(sysError("rename status"));
}

std::string findRoot(const char *argv0) {
  if (const char *env = std::getenv("RB_RUNTIME_ROOT")) {
    char buf[4096];
    if (realpath(env, buf)) return buf;
  }
  char exe[4096];
  const ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 1);
  std::string path = n > 0 ? std::string(exe, static_cast<std::size_t>(n)) : argv0;
  // <root>/build/<target>/bin/rb_runtime
  for (int i = 0; i < 4; ++i) path = path.substr(0, path.find_last_of('/'));
  char buf[4096];
  return realpath(path.c_str(), buf) ? std::string(buf) : path;
}

}  // namespace

int runApp(int argc, char **argv, PolicyFactory factory) {
  std::string root, command_path;
  int command_listener = -1, command_connection = -1, listener = -1, connection = -1, fd = -1, lock_fd = -1;
  std::unique_ptr<DashboardProcess> web;
  std::unique_ptr<Policy> policy;
  std::unique_ptr<StandupLogic> logic;
  int exit_code = 0;
  try {
    root = findRoot(argv[0]);
    const Options args = parseOptions(argc, argv, root);
    Json cfg = Json::parse(readFile(args.config));
    if (!args.backend.empty()) cfg.set("backend", args.backend);
    if (!args.serial.empty()) { cfg.set("transport", "serial"); cfg.set("serial_port", args.serial); }
    if (!args.socket.empty()) { cfg.set("transport", "unix"); cfg.set("socket", args.socket); }
    if (!args.status_file.empty()) cfg.set("status_file", args.status_file);
    if (args.realtime_priority >= 0) cfg.set("realtime_priority", args.realtime_priority);

    const CoreConfig core_config = CoreConfig::fromJson(cfg);
    const std::string backend = cfg.at("backend").string();
    const std::string transport_cfg = cfg.at("transport").string();
    const std::string status_file = cfg.at("status_file").string();
    if (!(0 <= core_config.home_hold_s && core_config.home_hold_s < core_config.warmup_start_s &&
          core_config.warmup_start_s < core_config.handoff_earliest_s))
      throw std::invalid_argument("invalid phase timings");
    if (core_config.warmup_steps < 50 || core_config.inference_deadline_ms > 10)
      throw std::invalid_argument("policy requires at least 50 warm-up ticks and a 10 ms deadline");
    if (!(10 <= core_config.inference_hard_deadline_ms && core_config.inference_hard_deadline_ms <= 15) ||
        !(1 <= core_config.max_consecutive_deadline_misses && core_config.max_consecutive_deadline_misses <= 3))
      throw std::invalid_argument("invalid bounded jitter budget");
    if (backend != "rknn")
      throw std::invalid_argument("native runtime supports only the rknn backend (got " + backend + ")");

    lock_fd = open((status_file + ".lock").c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) != 0) throw std::runtime_error("runtime already running (status lock held)");

    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    const int priority = cfg.find("realtime_priority") ? static_cast<int>(cfg.at("realtime_priority").integer()) : 0;
    if (priority) {
      sched_param param{};
      param.sched_priority = priority;
      if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) throw std::runtime_error(sysError("sched_setscheduler"));
    }
    policy = factory ? factory(root) : makeRknnPolicy(root);
    logic = std::make_unique<StandupLogic>();
    HybridCore core(*policy, *logic, core_config);

    std::string transport_name = args.listen_tcp >= 0 ? "loopback-tcp" : transport_cfg;
    if (args.listen_tcp >= 0) {
      listener = listenLoopback(args.listen_tcp);
    } else if (transport_cfg == "unix") {
      const std::string path = cfg.at("socket").string();
      clearSocketPath(path, "socket");
      listener = listenUnix(path, 1);
    } else if (transport_cfg == "serial" && !cfg.at("serial_port").string().empty()) {
      fd = openSerial(cfg.at("serial_port").string(), static_cast<int>(cfg.at("baud").integer()));
    } else {
      throw std::invalid_argument("UART port must be configured before selecting serial transport");
    }

    command_path = args.command_socket;
    clearSocketPath(command_path, "operator command socket");
    command_listener = listenUnix(command_path, 2);

    if (!args.no_web) {
      web = std::make_unique<DashboardProcess>(root, args.web_host, args.web_port, args.web_token_file,
                                               status_file, command_path);
      try {
        web->start();
      } catch (const std::exception &error) {
        Json d = Json::object();
        d.set("reason", error.what());
        log("web_unavailable", d);
        web.reset();
      }
    }

    std::unique_ptr<Writer> writer;
    if (fd >= 0) writer = std::make_unique<Writer>(fd);
    hcr::Parser parser;
    double last_hello = 0, last_status = 0, command_deadline = 0;
    std::optional<Mode> last_mode;
    double max_control_ms = 0, max_telemetry_ms = 0;
    std::string command_buffer;

    {
      Json d = Json::object();
      d.set("backend", backend);
      d.set("transport", transport_name);
      d.set("web", web ? Json("http://" + args.web_host + ":" + std::to_string(web->port())) : Json());
      d.set("web_pid", web ? Json(static_cast<int>(web->pid())) : Json());
      d.set("token_file", args.no_web ? Json() : Json(args.web_token_file));
      log("ready", d);
    }

    auto dropConnection = [&] {
      if (connection >= 0) ::close(connection);
      connection = -1;
      fd = -1;
      writer.reset();
    };

    while (!g_stop) {
      double now = monotonic();
      if (command_connection >= 0 && now >= command_deadline) {
        ::close(command_connection);
        command_connection = -1;
        command_buffer.clear();
      }
      if (writer) {
        try {
          writer->add(core.watchdog(now));
          if (!core.link && now - last_hello >= 1) {
            writer->add({core.hello()});
            last_hello = now;
          }
          if (now - last_status >= 1) writer->add({core.status()});
          writer->flush();
        } catch (const std::runtime_error &error) {
          core.fail("transport_fault", error.what());
          if (listener < 0) throw;
          dropConnection();
        }
      }
      if (now - last_status >= 1 || !last_mode || core.mode != *last_mode) {
        Json snapshot = core.snapshot();
        snapshot.set("transport", transport_name);
        snapshot.set("web_process_alive", web != nullptr && web->alive());
        snapshot.set("parser_errors", parser.errors());
        snapshot.set("max_handler_ms", max_control_ms);
        snapshot.set("max_telemetry_ms", max_telemetry_ms);
        snapshot.set("updated_monotonic", now);
        writeStatus(status_file, snapshot);
        if (!last_mode || core.mode != *last_mode) log("mode", snapshot);
        last_mode = core.mode;
        last_status = now;
      }

      std::vector<pollfd> fds = {{command_listener, POLLIN, 0}};
      if (fd >= 0) fds.push_back({fd, POLLIN, 0});
      else if (listener >= 0) fds.push_back({listener, POLLIN, 0});
      if (command_connection >= 0) fds.push_back({command_connection, POLLIN, 0});
      if (poll(fds.data(), fds.size(), 5) <= 0) continue;

      for (const pollfd &ready : fds) {
        if (!(ready.revents & (POLLIN | POLLHUP | POLLERR))) continue;
        if (ready.fd == command_listener) {
          const int client = accept(command_listener, nullptr, nullptr);
          if (client < 0) continue;
          if (command_connection >= 0) {
            ::close(client);
          } else {
            command_connection = client;
            setNonBlocking(client);
            command_buffer.clear();
            command_deadline = monotonic() + 1.0;
          }
          continue;
        }
        if (ready.fd == command_connection) {
          try {
            char chunk[129];
            const ssize_t n = recv(command_connection, chunk, sizeof chunk, 0);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            if (n < 0) throw std::runtime_error(sysError("recv"));
            command_buffer.append(chunk, static_cast<std::size_t>(n));
            if (command_buffer.size() > 128) throw std::invalid_argument("command exceeds 128 bytes");
            if (command_buffer.find('\n') == std::string::npos && n > 0) continue;
            if (!command_buffer.empty()) {
              const std::size_t eol = command_buffer.find('\n');
              const std::string line = command_buffer.substr(0, eol);
              const std::string tail = eol == std::string::npos ? "" : command_buffer.substr(eol + 1);
              if (tail.find_first_not_of(" \t\r\n") != std::string::npos)
                throw std::invalid_argument("send one command per connection");
              const Command cmd = parseCommand(line);
              if (cmd.op == "RESET_ENCODERS") last_status = 0;
              const auto rejected = core.counters.rejected;
              const auto reply = cmd.op == "ESTOP" ? core.estop() : core.command(cmd.op, monotonic(), cmd.vx, cmd.wz);
              if (writer) {
                writer->add(reply);
                writer->flush();
              }
              const bool accepted = core.counters.rejected == rejected;
              Json r = Json::object();
              r.set("accepted", accepted);
              r.set("mode", modeName(core.mode));
              r.set("fault", core.reason);
              r.set("error", !accepted && core.last_command_error ? Json(*core.last_command_error) : Json());
              sendAll(command_connection, r.dump() + "\n");
            }
            ::close(command_connection);
            command_connection = -1;
          } catch (const std::exception &error) {
            replyError(command_connection, error.what());
            ::close(command_connection);
            command_connection = -1;
          }
          continue;
        }
        if (fd >= 0 && ready.fd == fd) {
          try {
            std::uint8_t buf[4096];
            const ssize_t n = read(fd, buf, sizeof buf);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
            if (n < 0) throw std::runtime_error(sysError("read"));
            if (n == 0) throw std::runtime_error("MCU connection closed");
            for (const hcr::Frame &frame : parser.feed(buf, static_cast<std::size_t>(n))) {
              const double begin = monotonic();
              auto reply = core.receive(frame, begin);
              const double duration = (monotonic() - begin) * 1000;
              max_control_ms = std::max(max_control_ms, duration);
              if (frame.kind == hcr::kTelemetry) max_telemetry_ms = std::max(max_telemetry_ms, duration);
              if (frame.kind == hcr::kTelemetry && duration > 15 && core.mode >= Mode::Home && core.mode <= Mode::Model) {
                char text[96];
                std::snprintf(text, sizeof text, "telemetry handler %.3f ms exceeds 15 ms", duration);
                reply = {core.fail("control_deadline", text), core.status()};
              }
              if (frame.kind == hcr::kEncoderHomeAck) last_status = 0;
              writer->add(reply);
            }
            writer->flush();
          } catch (const std::runtime_error &error) {
            if (core.mode == Mode::Disarmed || core.mode == Mode::WaitLink) core.newEpoch();
            else core.fail("transport_fault", error.what());
            if (listener < 0) throw;
            dropConnection();
          }
        } else if (fd < 0 && listener >= 0 && ready.fd == listener) {
          const int client = accept(listener, nullptr, nullptr);
          if (client < 0) continue;
          setNonBlocking(client);
          connection = client;
          fd = client;
          writer = std::make_unique<Writer>(fd);
          parser = hcr::Parser();
          core.newEpoch();
          last_hello = 0;
        }
      }
    }
    if (writer) {
      writer->add({core.fail("transport_fault", "service stopping")});
      writer->flush();
    }
  } catch (const std::exception &error) {
    Json d = Json::object();
    d.set("reason", error.what());
    log("fatal", d);
    exit_code = 1;
  }
  web.reset();
  if (connection >= 0) ::close(connection);
  else if (fd >= 0) ::close(fd);
  if (listener >= 0) ::close(listener);
  if (command_connection >= 0) ::close(command_connection);
  if (command_listener >= 0) {
    ::close(command_listener);
    unlink(command_path.c_str());
  }
  logic.reset();
  policy.reset();
  if (lock_fd >= 0) ::close(lock_fd);
  return exit_code;
}

}  // namespace rb::runtime

// UART open/flush helpers and the dashboard child process.
#pragma once
#include <string>
#include <sys/types.h>
#include <vector>

#include "rb/runtime/protocol.hpp"

namespace rb::runtime {

// Raw 8N1 non-blocking serial port, exclusive. Throws std::runtime_error when another
// process already holds the device or the baud rate is unsupported.
int openSerial(const std::string &path, int baud);

class Writer {
 public:
  explicit Writer(int fd) : fd_(fd) {}
  // Throws std::runtime_error on output backpressure (> 4096 queued bytes).
  void add(const std::vector<hcr::Frame> &frames);
  void flush();  // throws std::runtime_error on a write error; EAGAIN keeps the bytes queued

 private:
  int fd_;
  std::vector<std::uint8_t> pending_;
};

// Web is expendable: a separate process with no UART or controller objects. It is
// exec'd (not forked into) so native NPU threads never leak into it.
class DashboardProcess {
 public:
  DashboardProcess(std::string root, std::string host, int port, std::string token_file,
                   std::string status_file, std::string command_socket);
  ~DashboardProcess() { close(); }
  void start();  // startup only; throws std::runtime_error
  void close();
  bool alive();
  int port() const { return port_; }
  pid_t pid() const { return pid_; }

 private:
  std::string root_, host_, token_file_, status_file_, command_socket_;
  int requested_port_, port_ = 0;
  pid_t pid_ = -1;
};

}  // namespace rb::runtime

// Pipe front end for the stand-up adapter (Python helper). The adapter itself lives in
// rb/recovery/standup_adapter.hpp so the native runtime can run it in-process.
#include "rb/recovery/standup_adapter.hpp"

int main() {
  static_assert(sizeof(double) == 8, "binary pipe protocol requires float64");
  auto logic = std::make_unique<StandupAdapter>();
  std::array<double, 24> request{};
  while (std::fread(request.data(), sizeof(double), request.size(), stdin) == request.size()) {
    std::array<double, 14> reply{};
    try {
      bool valid = true;
      for (auto v : request)
        valid = valid && std::isfinite(v);
      if (!valid)
        reply[0] = -1;
      else if (request[23] != 0)
        logic = std::make_unique<StandupAdapter>();
      else
        logic->step(request, reply);
    } catch (...) {
      reply[0] = -1;
    }
    if (std::fwrite(reply.data(), sizeof(double), reply.size(), stdout) != reply.size() ||
        std::fflush(stdout))
      return 2;
  }
  return std::ferror(stdin) ? 1 : 0;
}

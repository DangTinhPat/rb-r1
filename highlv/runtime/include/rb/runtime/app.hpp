#pragma once
#include <functional>
#include <memory>
#include <string>

#include "rb/runtime/policy.hpp"

namespace rb::runtime {

using PolicyFactory = std::function<std::unique_ptr<Policy>(const std::string &root)>;

// Command-line front end: the same options as `python -m rb_runtime`, except that
// stand-up logic runs in-process and only the RKNN backend exists. Returns the exit code.
// `factory` defaults to the RKNN policy; tests inject a fake one.
int runApp(int argc, char **argv, PolicyFactory factory = nullptr);

}  // namespace rb::runtime

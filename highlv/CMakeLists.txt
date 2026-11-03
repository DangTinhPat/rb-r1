cmake_minimum_required(VERSION 3.20)
project(rb_runtime LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

add_library(rb_recovery STATIC
  native/src/recovery_controller.cpp
  native/src/leg_height.cpp
  native/src/lqr_balancer.cpp
  native/src/imu_state_estimator.cpp
  native/src/whole_body_model.cpp
  native/src/lqr_config.cpp)
target_include_directories(rb_recovery PUBLIC native/include)
target_compile_definitions(rb_recovery PUBLIC RB_RECOVERY_SPLIT_SUPPORT=1)
target_compile_options(rb_recovery PRIVATE -Wall -Wextra -Wpedantic)

add_executable(standup_helper native/standup_main.cpp)
target_link_libraries(standup_helper PRIVATE rb_recovery)
option(RB_STATIC_HELPER "Link the recovery helper statically for Buildroot" OFF)
if(RB_STATIC_HELPER)
  target_link_options(standup_helper PRIVATE -static)
endif()

add_library(tensor_math SHARED native/tensor_math.c)
target_compile_options(tensor_math PRIVATE -O3 -ffreestanding)
target_link_options(tensor_math PRIVATE -nostdlib -Wl,--hash-style=both)

# Native runtime: HybridCore, HCR1 master, RKNN policy and in-process stand-up logic.
# It dlopen()s the board's librknnmrt.so, so it must be linked dynamically against the
# board's libc. The static glibc cross build above cannot host it: set
# RB_BUILD_RUNTIME=ON only with the Luckfox SDK toolchain (see README).
option(RB_BUILD_RUNTIME "Build the native rb_runtime" ON)
if(RB_BUILD_RUNTIME)
  add_library(tensor_math_static STATIC native/tensor_math.c)
  set_target_properties(tensor_math_static PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(rb_runtime_core STATIC
    runtime/src/json.cpp
    runtime/src/protocol.cpp
    runtime/src/commands.cpp
    runtime/src/policy.cpp
    runtime/src/standup_logic.cpp
    runtime/src/hybrid_core.cpp
    runtime/src/transport.cpp
    runtime/src/app.cpp)
  target_include_directories(rb_runtime_core PUBLIC runtime/include protocol/include)
  target_link_libraries(rb_runtime_core PUBLIC rb_recovery tensor_math_static ${CMAKE_DL_LIBS})
  target_compile_options(rb_runtime_core PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(rb_runtime runtime/src/main.cpp)
  target_link_libraries(rb_runtime PRIVATE rb_runtime_core)

  enable_testing()
  add_executable(rb_runtime_tests runtime/tests/runtime_tests.cpp)
  target_link_libraries(rb_runtime_tests PRIVATE rb_runtime_core)
  target_compile_options(rb_runtime_tests PRIVATE -Wall -Wextra)
  target_compile_definitions(rb_runtime_tests PRIVATE RB_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  add_test(NAME rb_runtime_tests COMMAND rb_runtime_tests)
endif()

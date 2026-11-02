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

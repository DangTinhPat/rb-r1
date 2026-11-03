cmake_minimum_required(VERSION 3.20)
project(rb_runtime LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Both libraries are freestanding (no libc, libm or STL): they load into the LF's own Python
# through ctypes whatever libc the cross compiler used.

# NPU: quantize/dequantize tensors and run one inference in a single native call.
add_library(tensor_math SHARED native/tensor_math.c)
target_compile_options(tensor_math PRIVATE -O3 -ffreestanding)
target_link_options(tensor_math PRIVATE -nostdlib -Wl,--hash-style=both)

# Fold-down planner (C++).
add_library(fold_planner SHARED native/fold_planner.cpp)
target_compile_options(fold_planner PRIVATE -O3 -ffreestanding -fno-exceptions -fno-rtti
                       -fno-threadsafe-statics -fno-stack-protector -fno-asynchronous-unwind-tables
                       -Wall -Wextra -Wpedantic)
target_link_options(fold_planner PRIVATE -nostdlib -Wl,--hash-style=both)

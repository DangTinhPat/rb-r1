cmake_minimum_required(VERSION 3.16.0)
# src/libs holds the reusable components; src/app is the application entry point.
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src/libs")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rb_r1_check_motor)

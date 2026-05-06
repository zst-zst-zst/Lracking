cmake_minimum_required(VERSION 3.10)
project(LaserTracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

option(BUILD_CONTROL "Build control module" ON)
option(BUILD_DETECT "Build detect module" ON)
option(BUILD_SOLVE "Build solve module" ON)
option(BUILD_GALAXY_CAMERA "Build galaxy_camera module" ON)
option(BUILD_GIMBAL_SERIAL "Build gimbal_serial module" ON)
option(BUILD_TOOLS "Build tools" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

add_subdirectory(common)

if(BUILD_GIMBAL_SERIAL)
  add_subdirectory(gimbal)
endif()

if(BUILD_GALAXY_CAMERA)
  add_subdirectory(camera)
endif()

if(BUILD_DETECT)
  add_subdirectory(detect)
endif()

if(BUILD_SOLVE AND NOT TARGET solve)
  add_subdirectory(solve)
endif()

if(BUILD_CONTROL)
  add_subdirectory(control)
endif()

if(NOT TARGET tracking)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tracking ${CMAKE_BINARY_DIR}/tracking)
endif()

if(NOT TARGET plotter)
  add_subdirectory(plotter)
endif()

if(BUILD_TOOLS)
  add_subdirectory(calibrate)
endif()


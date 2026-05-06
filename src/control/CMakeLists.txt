cmake_minimum_required(VERSION 3.10)
project(control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONTROL_BUILD_EXAMPLES "Build control examples" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

if(NOT TARGET common)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common_build)
endif()

add_library(control STATIC
  src/controller.cpp
  src/config.cpp
)

target_include_directories(control
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(control
  PUBLIC
    common
    ${OpenCV_LIBS}
)

# control_demo 已移至 tests/control_demo.cpp

# 主入口 tracking_main 已移至 src/tracking/tracking_main.cpp (由 src/tracking/CMakeLists.txt 构建)

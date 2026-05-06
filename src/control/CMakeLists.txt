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

# ── 比赛主系统 ──
if(NOT TARGET galaxy_camera)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../camera ${CMAKE_CURRENT_BINARY_DIR}/galaxy_camera_build)
endif()
if(NOT TARGET detect)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../detect ${CMAKE_CURRENT_BINARY_DIR}/detect_build)
endif()
if(NOT TARGET gimbal_serial)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../gimbal ${CMAKE_CURRENT_BINARY_DIR}/gimbal_serial_build)
endif()
# 主比赛系统入口 (tracking_main): 完整链路 = 相机 + 检测 + 控制 + 云台串口
# (调试/桌面测试用 src/tracking/track.cpp; 比赛/真机用此处)
add_executable(tracking_main examples/tracking_main.cpp)
target_link_libraries(tracking_main PRIVATE control detect galaxy_camera gimbal_serial common)

cmake_minimum_required(VERSION 3.10)
project(calibrate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# ── 依赖: 从 src/ 构建树继承 ──
if(NOT TARGET galaxy_camera)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../camera
                   ${CMAKE_CURRENT_BINARY_DIR}/galaxy_camera_build)
endif()
if(NOT TARGET gimbal_serial)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../gimbal
                   ${CMAKE_CURRENT_BINARY_DIR}/gimbal_serial_build)
endif()
if(NOT TARGET common)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common
                   ${CMAKE_CURRENT_BINARY_DIR}/common_build)
endif()

# ── 激光同轴校准 ──
add_executable(laser_boresight src/laser_boresight.cpp)

target_include_directories(laser_boresight PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../camera/include
  ${CMAKE_CURRENT_LIST_DIR}/../gimbal/include
  ${CMAKE_CURRENT_LIST_DIR}/../common/include
)

target_link_libraries(laser_boresight PRIVATE
  galaxy_camera gimbal_serial common
  ${OpenCV_LIBS} opencv_freetype
)

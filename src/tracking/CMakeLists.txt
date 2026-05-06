cmake_minimum_required(VERSION 3.10)
project(tracking_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

# 现在 tracking 在 src/ 内, 所有依赖 target (common/detect/solve/control/...)
# 由 src/CMakeLists.txt 统一 add_subdirectory 加载, 此处直接使用即可.

set(SRC_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

set(TRACKING_INC
  ${SRC_ROOT}/detect
  ${SRC_ROOT}/detect/TRTInferX/include
  ${SRC_ROOT}/solve
  ${SRC_ROOT}/camera/include
  ${SRC_ROOT}/gimbal/include
  ${SRC_ROOT}/common/include
  ${SRC_ROOT}/control/include
  ${SRC_ROOT}/plotter
)

add_executable(tracking ${CMAKE_CURRENT_LIST_DIR}/track.cpp)
target_include_directories(tracking PRIVATE ${TRACKING_INC} ${SRC_ROOT})
target_link_libraries(tracking PRIVATE
  detect solve galaxy_camera gimbal_serial control plotter common ${OpenCV_LIBS})

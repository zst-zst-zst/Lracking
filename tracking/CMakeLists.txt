cmake_minimum_required(VERSION 3.10)
project(tracking_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

set(TRACKING_SRC "${CMAKE_CURRENT_LIST_DIR}/../src")
set(DETECT_SRC   "${CMAKE_CURRENT_LIST_DIR}/../src/detect")
set(SOLVE_SRC    "${CMAKE_CURRENT_LIST_DIR}/../src/solve")
set(PLOTTER_DIR  "${CMAKE_CURRENT_LIST_DIR}/../src/plotter")

if(NOT TARGET common)
  add_subdirectory(${TRACKING_SRC}/common ${CMAKE_BINARY_DIR}/common)
endif()
if(NOT TARGET gimbal_serial)
  add_subdirectory(${TRACKING_SRC}/gimbal ${CMAKE_BINARY_DIR}/gimbal_serial)
endif()
if(NOT TARGET galaxy_camera)
  add_subdirectory(${TRACKING_SRC}/camera ${CMAKE_BINARY_DIR}/galaxy_camera)
endif()
if(NOT TARGET control)
  add_subdirectory(${TRACKING_SRC}/control ${CMAKE_BINARY_DIR}/control)
endif()
if(NOT TARGET detect)
  add_subdirectory(${DETECT_SRC} ${CMAKE_BINARY_DIR}/detect)
endif()
if(NOT TARGET solve)
  add_subdirectory(${SOLVE_SRC} ${CMAKE_BINARY_DIR}/solve)
endif()
if(NOT TARGET plotter)
  add_subdirectory(${PLOTTER_DIR} ${CMAKE_BINARY_DIR}/plotter)
endif()

set(TRACKING_INC
  ${DETECT_SRC}
  ${DETECT_SRC}/TRTInferX/include
  ${SOLVE_SRC}
  ${TRACKING_SRC}/camera/include
  ${TRACKING_SRC}/gimbal/include
  ${TRACKING_SRC}/common/include
  ${TRACKING_SRC}/control/include
  ${PLOTTER_DIR}
)

add_executable(tracking ${CMAKE_CURRENT_LIST_DIR}/track.cpp)
target_include_directories(tracking PRIVATE ${TRACKING_INC} ${TRACKING_SRC})
target_link_libraries(tracking PRIVATE
  detect solve galaxy_camera gimbal_serial control plotter common ${OpenCV_LIBS})
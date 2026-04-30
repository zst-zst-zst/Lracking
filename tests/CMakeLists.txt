cmake_minimum_required(VERSION 3.10)
project(tracking_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

# ── 复用 Tracking 工程模块 ──
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

# ── 公共 include ──
set(TEST_INC
  ${DETECT_SRC}
  ${DETECT_SRC}/TRTInferX/include
  ${SOLVE_SRC}
  ${TRACKING_SRC}/camera/include
  ${TRACKING_SRC}/gimbal/include
  ${TRACKING_SRC}/common/include
  ${TRACKING_SRC}/control/include
  ${PLOTTER_DIR}
)

# ── 1. 串口 ──
add_executable(serial serial.cpp)
target_include_directories(serial PRIVATE ${TEST_INC})
target_link_libraries(serial PRIVATE gimbal_serial common)

# ── 2. 激光接收装置识别 ──
add_executable(laser_rx laser_rx.cpp)
target_include_directories(laser_rx PRIVATE ${TEST_INC})
target_link_libraries(laser_rx PRIVATE detect galaxy_camera common ${OpenCV_LIBS})

# ── 3. 无人机识别 ──
add_executable(plane plane.cpp)
target_include_directories(plane PRIVATE ${TEST_INC})
target_link_libraries(plane PRIVATE detect galaxy_camera common ${OpenCV_LIBS})

# ── 4. 跟踪照射 ──
add_executable(track track.cpp)
target_include_directories(track PRIVATE ${TEST_INC})
target_link_libraries(track PRIVATE
  detect solve galaxy_camera gimbal_serial control plotter common ${OpenCV_LIBS})

# ── 6. 偏移推断 ──
add_executable(offset offset.cpp)
target_include_directories(offset PRIVATE ${TEST_INC})
target_link_libraries(offset PRIVATE detect solve galaxy_camera common ${OpenCV_LIBS})

# ── 7. 打中测试 ──
add_executable(hit hit.cpp)
target_include_directories(hit PRIVATE ${TEST_INC})
target_link_libraries(hit PRIVATE
  detect solve galaxy_camera gimbal_serial control common ${OpenCV_LIBS})

# ── 8. 颜色过滤 ──
add_executable(color color.cpp)
target_include_directories(color PRIVATE ${TEST_INC})
target_link_libraries(color PRIVATE galaxy_camera common ${OpenCV_LIBS})

# ── 9. 延迟测量 ──
add_executable(latency latency.cpp)
target_include_directories(latency PRIVATE ${TEST_INC})
target_link_libraries(latency PRIVATE detect solve control gimbal_serial galaxy_camera common ${OpenCV_LIBS})

# ── 10. 相机硬件测试 ──
add_executable(camera_demo camera_demo.cpp)
target_include_directories(camera_demo PRIVATE ${TEST_INC})
target_link_libraries(camera_demo PRIVATE galaxy_camera)

# ── 11. 级联检测可视化 ──
add_executable(detect_demo detect_demo.cpp)
target_include_directories(detect_demo PRIVATE ${TEST_INC} ${TRACKING_SRC})
target_link_libraries(detect_demo PRIVATE detect solve plotter common ${OpenCV_LIBS})

# ── 13. 控制器测试 ──
add_executable(control_demo control_demo.cpp)
target_include_directories(control_demo PRIVATE ${TEST_INC})
target_link_libraries(control_demo PRIVATE control common)

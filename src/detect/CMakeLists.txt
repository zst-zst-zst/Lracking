cmake_minimum_required(VERSION 3.10)
project(detect LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──
find_package(OpenCV REQUIRED)

set(TRACKING_SRC "${CMAKE_CURRENT_LIST_DIR}/..")

if(NOT TARGET common)
  add_subdirectory(${TRACKING_SRC}/common ${CMAKE_BINARY_DIR}/common)
endif()

# TRTInferX (TensorRT YOLO inference)
if(NOT TARGET trt_yolo)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/TRTInferX ${CMAKE_BINARY_DIR}/TRTInferX)
endif()

# ── ONNX Runtime (optional, for dev machines without TensorRT) ──
option(DETECT_WITH_ONNX "Build ONNX Runtime detector backend" OFF)

# ── detect library ──
set(DETECT_SOURCES
  measurement.cpp
  cascade_detector.cpp
)
if(DETECT_WITH_ONNX)
  list(APPEND DETECT_SOURCES onnx_detector.cpp)
endif()

add_library(detect STATIC ${DETECT_SOURCES})
target_include_directories(detect PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/TRTInferX/include
)
target_link_libraries(detect PUBLIC
  common
  trt_yolo
  ${OpenCV_LIBS}
)

if(DETECT_WITH_ONNX)
  find_package(onnxruntime QUIET)
  if(onnxruntime_FOUND)
    target_link_libraries(detect PUBLIC onnxruntime::onnxruntime)
  else()
    # Fallback: assume system-installed onnxruntime
    target_link_libraries(detect PUBLIC onnxruntime)
  endif()
  target_compile_definitions(detect PUBLIC DETECT_WITH_ONNX=1)
  message(STATUS "detect: ONNX Runtime backend enabled")
endif()

# detect_demo 已移至 tests/detect_demo.cpp

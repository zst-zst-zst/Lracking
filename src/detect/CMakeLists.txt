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
  # measurement.cpp 已弃用 (未被任何文件引用, toMeasurement 可直接内联)
  cascade_detector.cpp
)
if(DETECT_WITH_ONNX)
  list(APPEND DETECT_SOURCES onnx_detector.cpp sr_inferer.cpp)
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
  set(ORT_ROOT "${TRACKING_SRC}/thirdparty/onnxruntime-linux-x64-gpu-1.23.2")
  if(EXISTS "${ORT_ROOT}/include/onnxruntime_cxx_api.h")
    target_include_directories(detect PUBLIC "${ORT_ROOT}/include")
    target_link_directories(detect PUBLIC "${ORT_ROOT}/lib")
    target_link_libraries(detect PUBLIC onnxruntime)
    set_property(TARGET detect APPEND PROPERTY
      INSTALL_RPATH "${ORT_ROOT}/lib")
    set_property(TARGET detect APPEND PROPERTY
      BUILD_RPATH "${ORT_ROOT}/lib")
    message(STATUS "detect: using thirdparty ONNX Runtime at ${ORT_ROOT}")
  else()
    find_package(onnxruntime QUIET)
    if(onnxruntime_FOUND)
      target_link_libraries(detect PUBLIC onnxruntime::onnxruntime)
    else()
      target_link_libraries(detect PUBLIC onnxruntime)
    endif()
  endif()
  target_compile_definitions(detect PUBLIC DETECT_WITH_ONNX=1)
  message(STATUS "detect: ONNX Runtime backend enabled")
endif()

# detect_demo 已移至 tests/detect_demo.cpp

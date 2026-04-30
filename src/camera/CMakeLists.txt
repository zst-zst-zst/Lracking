cmake_minimum_required(VERSION 3.10)
project(galaxy_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(_tracking_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(GALAXY_CAMERA_BUILD_MODE_FILE
    "${_tracking_root}/config/camera.yaml"
    CACHE FILEPATH "Camera config file (compute_mode: gpu|cpu)")

set(_camera_compute_mode "")
if(EXISTS "${GALAXY_CAMERA_BUILD_MODE_FILE}")
  file(STRINGS "${GALAXY_CAMERA_BUILD_MODE_FILE}" _camera_mode_lines
       REGEX "^[ \t]*compute_mode[ \t]*:")
  if(_camera_mode_lines)
    list(GET _camera_mode_lines 0 _camera_mode_line)
    string(REGEX REPLACE "^[ \t]*compute_mode[ \t]*:[ \t]*\"?([A-Za-z_]+)\"?.*$"
                         "\\1"
                         _camera_compute_mode
                         "${_camera_mode_line}")
    string(TOLOWER "${_camera_compute_mode}" _camera_compute_mode)
  endif()
endif()

set(_camera_default_cuda ON)
set(_camera_default_npp ON)
if(_camera_compute_mode STREQUAL "cpu")
  set(_camera_default_cuda OFF)
  set(_camera_default_npp OFF)
endif()

option(GALAXY_CAMERA_BUILD_EXAMPLES "Build examples" ON)
option(GALAXY_CAMERA_ENABLE_CUDA "Enable CUDA features when available" ${_camera_default_cuda})
option(GALAXY_CAMERA_ENABLE_NPP "Enable NPP demosaic when available" ${_camera_default_npp})

if(_camera_compute_mode STREQUAL "cpu")
  set(GALAXY_CAMERA_ENABLE_CUDA OFF CACHE BOOL "Enable CUDA features when available" FORCE)
  set(GALAXY_CAMERA_ENABLE_NPP OFF CACHE BOOL "Enable NPP demosaic when available" FORCE)
  message(STATUS "Camera build mode: CPU (from ${GALAXY_CAMERA_BUILD_MODE_FILE})")
elseif(_camera_compute_mode STREQUAL "gpu")
  set(GALAXY_CAMERA_ENABLE_CUDA ON CACHE BOOL "Enable CUDA features when available" FORCE)
  set(GALAXY_CAMERA_ENABLE_NPP ON CACHE BOOL "Enable NPP demosaic when available" FORCE)
  message(STATUS "Camera build mode: GPU (from ${GALAXY_CAMERA_BUILD_MODE_FILE})")
elseif(NOT _camera_compute_mode STREQUAL "")
  message(WARNING "Unknown compute_mode='${_camera_compute_mode}' in ${GALAXY_CAMERA_BUILD_MODE_FILE}; keeping CMake options")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(GALAXY_CAMERA_CXX_FLAGS -O3 -Wall -Wextra -Wpedantic)
endif()

# OpenCV
find_package(OpenCV REQUIRED)

if(GALAXY_CAMERA_ENABLE_CUDA)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    message(STATUS "CUDA toolkit found; enabling CUDA features")
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
      # CMake 3.22 may not accept "native" for CUDA; pin to Ampere (RTX 3060 = 86).
      set(CMAKE_CUDA_ARCHITECTURES 86)
    endif()
  else()
    message(STATUS "CUDA toolkit not found; CUDA features disabled")
  endif()
endif()

find_path(OPENCV_CUDAIMGPROC_INCLUDE
  NAMES opencv2/cudaimgproc.hpp
  PATHS ${OpenCV_INCLUDE_DIRS}
)
if(OPENCV_CUDAIMGPROC_INCLUDE)
  message(STATUS "OpenCV CUDA modules detected")
endif()

# Galaxy SDK
set(_default_galaxy_sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/galaxy_sdk")
if(NOT DEFINED GALAXY_SDK_ROOT)
  set(GALAXY_SDK_ROOT "${_default_galaxy_sdk_root}" CACHE PATH "Galaxy SDK root directory")
elseif(NOT EXISTS "${GALAXY_SDK_ROOT}/inc" AND EXISTS "${_default_galaxy_sdk_root}/inc")
  message(STATUS "GALAXY_SDK_ROOT invalid (${GALAXY_SDK_ROOT}), fallback to ${_default_galaxy_sdk_root}")
  set(GALAXY_SDK_ROOT "${_default_galaxy_sdk_root}" CACHE PATH "Galaxy SDK root directory" FORCE)
endif()
set(GALAXY_INCLUDE_DIR "${GALAXY_SDK_ROOT}/inc")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(GALAXY_LIB_DIR "${GALAXY_SDK_ROOT}/lib/aarch64")
else()
  set(GALAXY_LIB_DIR "${GALAXY_SDK_ROOT}/lib/x86_64")
endif()

if(NOT EXISTS "${GALAXY_INCLUDE_DIR}")
  message(FATAL_ERROR "Galaxy include dir not found: ${GALAXY_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${GALAXY_LIB_DIR}")
  message(FATAL_ERROR "Galaxy lib dir not found: ${GALAXY_LIB_DIR}")
endif()

add_library(galaxy_camera STATIC
  src/galaxy_camera.cpp
  src/galaxy_camera_config.cpp
  src/galaxy_camera_frame.cpp
  src/galaxy_camera_gpu.cpp
)

if(GALAXY_CAMERA_ENABLE_NPP AND CUDAToolkit_FOUND)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  target_sources(galaxy_camera PRIVATE src/gpu_demosaic_npp.cu)
endif()

target_include_directories(galaxy_camera
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${GALAXY_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_directories(galaxy_camera PUBLIC ${GALAXY_LIB_DIR})

target_link_libraries(galaxy_camera
  PUBLIC
    ${OpenCV_LIBS}
    gxiapi
    pthread
    dl
)

set_target_properties(galaxy_camera PROPERTIES
  BUILD_RPATH "${GALAXY_LIB_DIR}"
  INSTALL_RPATH "${GALAXY_LIB_DIR}"
)

if(GALAXY_CAMERA_CXX_FLAGS)
  target_compile_options(galaxy_camera PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${GALAXY_CAMERA_CXX_FLAGS}>)
endif()

if(GALAXY_CAMERA_ENABLE_CUDA AND CUDAToolkit_FOUND)
  target_compile_options(galaxy_camera PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-w>
  )
endif()

if(GALAXY_CAMERA_ENABLE_CUDA AND CUDAToolkit_FOUND)
  target_compile_definitions(galaxy_camera PUBLIC GALAXY_CAMERA_WITH_CUDA)
  target_include_directories(galaxy_camera PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(galaxy_camera PUBLIC CUDA::cudart)
endif()

if(OPENCV_CUDAIMGPROC_INCLUDE)
  target_compile_definitions(galaxy_camera PUBLIC GALAXY_CAMERA_WITH_OPENCV_CUDA)
endif()

if(GALAXY_CAMERA_ENABLE_NPP AND CUDAToolkit_FOUND)
  target_compile_definitions(galaxy_camera PUBLIC GALAXY_CAMERA_WITH_NPP)
  target_link_libraries(galaxy_camera PUBLIC CUDA::nppc CUDA::nppicc)
endif()

# galaxy_camera_demo 已移至 tests/camera_demo.cpp

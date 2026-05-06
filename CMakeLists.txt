cmake_minimum_required(VERSION 3.10)
# 顶层 CMake: 统一构建入口
# 用法: cmake -B build -S . && cmake --build build -j$(nproc)
# 子模块 (src/calibrate, tests, tracking) 不应再独立构建,
# 统一从顶层 build/ 出
project(LaserTrackingTop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 统一可执行文件输出到 build/bin/, 库到 build/lib/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 核心: 加载 src/ (含所有库 + tracking + calibrate, 通过 add_subdirectory)
add_subdirectory(src)

# 可选: 测试 demo 集合 (不启用单元测试, 仅手动 demo)
option(BUILD_TESTS "Build manual demo executables under tests/" OFF)
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

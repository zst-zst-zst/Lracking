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

# 可选: Qt5 控制面板 GUI (启动器 + 配置编辑 + 状态监控)
# 默认尝试构建; 如果系统无 Qt5 则跳过, 不影响主项目
option(BUILD_CONTROL_PANEL "Build Qt5 control_panel GUI (auto-skip if Qt5 missing)" ON)
if(BUILD_CONTROL_PANEL)
  find_package(Qt5 QUIET COMPONENTS Core Widgets Gui)
  if(Qt5_FOUND)
    message(STATUS "Qt5 found (${Qt5_VERSION}), building control_panel GUI")
    add_subdirectory(src/UI)
  else()
    message(STATUS "Qt5 not found, skipping control_panel GUI (set -DBUILD_CONTROL_PANEL=OFF to silence)")
  endif()
endif()

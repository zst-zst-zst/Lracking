cmake_minimum_required(VERSION 3.10)
project(control_panel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

# yaml-cpp 用于 ConfigTab 解析 yaml (Tracking 已使用 OpenCV FileStorage,
# 但表单编辑场景下 yaml-cpp 更灵活, 系统通常已安装)
find_package(yaml-cpp QUIET)

set(CONTROL_PANEL_SOURCES
  main.cpp
  ControlPanel.cpp
)

set(CONTROL_PANEL_HEADERS
  ControlPanel.h
)

add_executable(control_panel
  ${CONTROL_PANEL_SOURCES}
  ${CONTROL_PANEL_HEADERS}
)

target_include_directories(control_panel PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(control_panel PRIVATE
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

if(yaml-cpp_FOUND)
  target_link_libraries(control_panel PRIVATE yaml-cpp::yaml-cpp)
  target_compile_definitions(control_panel PRIVATE HAVE_YAML_CPP)
endif()

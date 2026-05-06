cmake_minimum_required(VERSION 3.10)
project(control_panel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
find_package(OpenCV REQUIRED)  # ConfigTab 用 cv::FileStorage 读 yaml

set(CONTROL_PANEL_SOURCES
  main.cpp
  ControlPanel.cpp
  LaunchTab.cpp
  ConfigTab.cpp
  StatusTab.cpp
  ProcessManager.cpp
)

set(CONTROL_PANEL_HEADERS
  ControlPanel.h
  LaunchTab.h
  ConfigTab.h
  StatusTab.h
  ProcessManager.h
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
  ${OpenCV_LIBS}
)

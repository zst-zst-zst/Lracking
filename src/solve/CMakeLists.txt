cmake_minimum_required(VERSION 3.10)
project(solve LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

set(TRACKING_SRC "${CMAKE_CURRENT_LIST_DIR}/..")

if(NOT TARGET common)
  add_subdirectory(${TRACKING_SRC}/common ${CMAKE_BINARY_DIR}/common)
endif()

# ── solve library ──
# Tracking (Kalman + IoU) + match analysis (P-value, stability tiers, flight phase)
add_library(solve STATIC
  target_tracker.cpp
  match_analyzer.cpp
)
target_include_directories(solve PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(solve PUBLIC
  common
  ${OpenCV_LIBS}
)

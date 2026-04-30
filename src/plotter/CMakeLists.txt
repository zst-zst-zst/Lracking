cmake_minimum_required(VERSION 3.10)
project(plotter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nlohmann_json QUIET)

add_library(plotter STATIC plotter.cpp)

target_include_directories(plotter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(nlohmann_json_FOUND)
  target_link_libraries(plotter PUBLIC nlohmann_json::nlohmann_json)
else()
  # fallback: header-only, assume installed system-wide
  find_path(NLOHMANN_JSON_INCLUDE nlohmann/json.hpp)
  if(NLOHMANN_JSON_INCLUDE)
    target_include_directories(plotter PUBLIC ${NLOHMANN_JSON_INCLUDE})
  else()
    message(FATAL_ERROR "nlohmann/json.hpp not found. Install: sudo apt install nlohmann-json3-dev")
  endif()
endif()

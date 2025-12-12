cmake_minimum_required(VERSION 3.10)

project(RS_marching)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Debug build if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  # set(CMAKE_BUILD_TYPE Debug)
endif()

# Base flags for all builds
set(cmake_cxx_flags "${cmake_cxx_flags} -g -fno-omit-frame-pointer")

# Profiling-specific flags (only enable `-pg` for gprof)
option(ENABLE_GPROF "Enable gprof profiling" OFF)
if(ENABLE_GPROF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -no-pie")
endif()

# Debug build settings (safe for debugging)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -std=c++20 -D_USE_MATH_DEFINES")

# Release build settings
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wpedantic -Wextra -Wnull-dereference -fexpensive-optimizations -mfpmath=sse -fno-trapping-math -fno-math-errno -flto=auto -std=c++20 -D_USE_MATH_DEFINES")


# Set output directory to project source dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(include)

# add_subdirectory(abseil-cpp)

add_executable(RS_marching
  src/main.cpp
  src/UnderSpecifiedRSPlanner.cpp
  src/Solver.cpp
  src/Environment.cpp
  src/Parser.cpp
  src/Marching.cpp
)

find_package(absl CONFIG REQUIRED)
find_package(unordered_dense CONFIG REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
target_link_libraries(RS_marching PRIVATE sfml-graphics absl::flat_hash_map unordered_dense::unordered_dense)

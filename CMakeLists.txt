cmake_minimum_required(VERSION 3.5)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
string(COMPARE EQUAL "${ARCHITECTURE}" x86_64 host64bit)

if(host64bit)
  # Set the location of the toolchain file if needed
  set(CMAKE_TOOLCHAIN_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake"
    CACHE FILEPATH
    "toolchain path" FORCE)

  message(STATUS "Cross compiling to ARM   ${CMAKE_TOOLCHAIN_FILE}")
endif(host64bit)

project(flyMS LANGUAGES CXX C)

# Compile for c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the binary and library directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen3/cmake)
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen3)
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Cannot Find Eigen3")
endif()

# Sets variable to a value if variable is unset.
#Build the robotics cape library
include(cmake/px4_add_library.cmake)
add_subdirectory(external/librobotcontrol)

include_directories(external/librobotcontrol/library/include)

# Build the Yaml-Cpp config file parser
add_subdirectory(external/yaml-cpp)

# Build the Yaml-Cpp config file parser
add_subdirectory(external/spdlog)

# Build the PX4 estimation control library
add_subdirectory(external/ecl)

# Build the Accel/Gyro Fusion library
add_subdirectory(external/Fusion)

# Build the Digital Filter Library
add_subdirectory(lib/filter)

# Build the pru_handler
add_subdirectory(src/pru_handler)

# Get the flyMS sources
add_subdirectory(src/flyMS)

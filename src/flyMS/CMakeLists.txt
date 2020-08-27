cmake_minimum_required(VERSION 3.5)

project(flyMS VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DDEBUG)

file(GLOB SRCS_FLYMS PATTERN src/*.cpp)
list(APPEND SRCS_FLYMS )

# Link statically to all libaries
set(CMAKE_EXE_LINKER_FLAGS "-static")

#Build flyMS
add_executable(flyMS
  src/flyMS.cc
  src/flyMSApp.cc
  src/gps.cc
  src/imu.cc
  src/pruClient.cc
  src/setpoint.cc
  src/ulog/ulog.cc
)

target_link_libraries(flyMS
  PUBLIC
    m
    rt
    ecl
    spdlog
    pthread
    robotics_cape
    filter
    Fusion
    yaml-cpp
)

target_include_directories(flyMS
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


install(TARGETS flyMS
  DESTINATION /usr/bin/)


cmake_minimum_required(VERSION 3.5)

project(flyMS VERSION 1.0.0)

add_definitions(-DDEBUG)

set(SRCS_FLYMS_C++
  src/config.cpp
  src/logger.cpp
  src/ekf.cpp
  src/flyMS.cpp
  src/flyMSApp.cpp
  src/gps.cpp
  src/imu.cpp
  src/pruClient.cpp
  src/setpoint.cpp
  src/startupRoutine.cpp
  src/logger.cpp
)

# Link statically to all libaries
set(CMAKE_EXE_LINKER_FLAGS "-static")

#Build flyMS
add_executable(flyMS_C++ ${SRCS_FLYMS_C++})

target_link_libraries(flyMS_C++
  PUBLIC
    m
    rt
    ecl
    pthread
    robotics_cape
    filter
    Fusion
    yaml-cpp
  )

target_include_directories(flyMS_C++
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

install(TARGETS flyMS_C++
    DESTINATION /usr/bin/)

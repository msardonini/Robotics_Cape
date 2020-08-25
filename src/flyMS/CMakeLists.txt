cmake_minimum_required(VERSION 3.5)

project(flyMS VERSION 1.0.0)

add_definitions(-DDEBUG)

file(GLOB SRCS_FLYMS PATTERN src/*.cpp)
list(APPEND SRCS_FLYMS src/ulog/ulog.cc)

# Link statically to all libaries
set(CMAKE_EXE_LINKER_FLAGS "-static")

#Build flyMS
add_executable(flyMS
  ${SRCS_FLYMS}
  )

target_link_libraries(flyMS
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

target_include_directories(flyMS
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

install(TARGETS flyMS
  DESTINATION /usr/bin/)

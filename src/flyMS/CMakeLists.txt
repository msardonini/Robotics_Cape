
cmake_minimum_required(VERSION 2.8)

#project(flyMS)

add_definitions(-DDEBUG)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

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
	m
	rt
	ecl
	pthread
	robotics_cape
	filter
	Fusion
	yaml-cpp)

install(TARGETS flyMS_C++
		DESTINATION /usr/bin/)

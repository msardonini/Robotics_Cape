
cmake_minimum_required(VERSION 2.8)

#project(flyMS)

add_definitions(-DDEBUG)

include_directories(inc/Cpp)

set(FLYMS_DIR src/Cpp)

set(SRCS_FLYMS_C++
	${FLYMS_DIR}/config.cpp
	${FLYMS_DIR}/logger.cpp
	${FLYMS_DIR}/ekf.cpp
	${FLYMS_DIR}/flyMS.cpp
	${FLYMS_DIR}/flyMSApp.cpp
	${FLYMS_DIR}/gps.cpp
	${FLYMS_DIR}/imu.cpp
	${FLYMS_DIR}/pruClient.cpp
	${FLYMS_DIR}/setpoint.cpp
	${FLYMS_DIR}/startupRoutine.cpp
	${FLYMS_DIR}/logger.cpp
)

# Link statically to all libaries
set(CMAKE_EXE_LINKER_FLAGS "-static")

include_directories(${FLYMS_DIR}/inc)

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
	DESTINATION /usr/bin/
	)


cmake_minimum_required(VERSION 2.8)

#project(flyMS)

# set(FLYMS_DIR src/flyMS/src/C)
# include_directories(src/flyMS/inc/C)

# set(SRCS_FLYMS
# 	${FLYMS_DIR}/config.c
# 	${FLYMS_DIR}/flyMS.c
# 	${FLYMS_DIR}/ekf.cpp
# 	${FLYMS_DIR}/gps.c
# 	${FLYMS_DIR}/logger.c
# 	${FLYMS_DIR}/misc_functions.c
# 	${FLYMS_DIR}/imu_handler.c
# 	${FLYMS_DIR}/setpoint.c
# )

# include_directories(${FLYMS_DIR}/inc)

# #Build flyMS
# add_executable(flyMS ${SRCS_FLYMS})
 
# #Link Needed Libraries
# find_library(ECL_LIB ecl)
# find_library(ROBOTICSCAPE_LIB roboticscape)
# target_link_libraries(flyMS m
# 	rt,
# 	pthread,
# 	ecl,
# 	roboticscape,
# 	filter,
# 	Fusion)

# add_executable(flight_replay 
# 		${FLYMS_DIR}/test/replayFlight.cpp
# 		${FLYMS_DIR}/src/logger.c)
# target_link_libraries(flight_replay pthread)
# target_link_libraries(flight_replay roboticscape)


# Build the C++ version of the flight control system

include_directories(src/flyMS/inc/Cpp)

set(FLYMS_DIR src/flyMS/src/Cpp)

set(SRCS_FLYMS_C++
	${FLYMS_DIR}/config.cpp
	${FLYMS_DIR}/ekf.cpp
	${FLYMS_DIR}/flyMS.cpp
	${FLYMS_DIR}/flyMSApp.cpp
	${FLYMS_DIR}/gps.cpp
	${FLYMS_DIR}/imu.cpp
	${FLYMS_DIR}/pruClient.cpp
	${FLYMS_DIR}/setpoint.cpp
	${FLYMS_DIR}/startupRoutine.cpp
	# ${FLYMS_DIR}/src/logger.cpp
)

include_directories(${FLYMS_DIR}/inc)

#Build flyMS
add_executable(flyMS_C++ ${SRCS_FLYMS_C++})


target_link_libraries(flyMS_C++ m
	rt,
	pthread,
	ecl,
	roboticscape,
	filter,
	Fusion,
	yaml-cpp)

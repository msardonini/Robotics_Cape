
cmake_minimum_required(VERSION 2.8)

#project(flyMS)

set(FLYMS_DIR src/flyMS/)

set(SRCS_FLYMS
	${FLYMS_DIR}config.c
	${FLYMS_DIR}flyMS.c
	${FLYMS_DIR}ekf.cpp
	${FLYMS_DIR}gps.c
	${FLYMS_DIR}logger.c
	${FLYMS_DIR}misc_functions.c
	${FLYMS_DIR}imu_handler.c
	)

#Build flyMS
add_executable(flyMS ${SRCS_FLYMS})
 
#Link Needed Libraries
find_library(ECL_LIB ecl)
find_library(ROBOTICSCAPE_LIB roboticscape)
target_link_libraries(flyMS m)
target_link_libraries(flyMS rt)
target_link_libraries(flyMS pthread)
target_link_libraries(flyMS ecl)
target_link_libraries(flyMS roboticscape)
target_link_libraries(flyMS filter)
target_link_libraries(flyMS Fusion)

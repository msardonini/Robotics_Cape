
cmake_minimum_required(VERSION 2.8)

#project(flyMS)

set(FLYMS_DIR src/flyMS/)

set(SRCS_FLYMS
	${FLYMS_DIR}src/config.c
	${FLYMS_DIR}src/flyMS.c
	${FLYMS_DIR}src/ekf.cpp
	${FLYMS_DIR}src/gps.c
	${FLYMS_DIR}src/logger.c
	${FLYMS_DIR}src/misc_functions.c
	${FLYMS_DIR}src/imu_handler.c
	)

include_directories(flyMS ${FLYMS_DIR}inc)

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


cmake_minimum_required(VERSION 2.8)

#project(flyMS)

set(FLYMS_DIR src/flyMS/)

add_definitions(-DFLYMS_ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
#add_definitions( "-D_SOME_DEFINE=\"Some value with spaces, etc\"" "-D DEF1=\"Another string\"")


set(SRCS_FLYMS
	${FLYMS_DIR}config.c
	${FLYMS_DIR}flyMS.c
	${FLYMS_DIR}ekf.cpp
	${FLYMS_DIR}filter.c
	${FLYMS_DIR}gps.c
	${FLYMS_DIR}kalman.c
	${FLYMS_DIR}linear_algebraMS.c
	${FLYMS_DIR}logger.c
	${FLYMS_DIR}misc_functions.c
	)

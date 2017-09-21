
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
	)
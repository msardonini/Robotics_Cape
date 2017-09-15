
cmake_minimum_required(VERSION 2.8)

#project(flyMS)

set(FLYMS_DIR src/flyMS/)


set(SRCS_FLYMS
	${FLYMS_DIR}src/config.c
	${FLYMS_DIR}src/flyMS.c
	${FLYMS_DIR}src/ekf.cpp
	${FLYMS_DIR}src/filter.c
	${FLYMS_DIR}src/gps.c
	${FLYMS_DIR}src/kalman.c
	${FLYMS_DIR}src/linear_algebraMS.c
	${FLYMS_DIR}src/logger.c
	${FLYMS_DIR}src/misc_functions.c
	)

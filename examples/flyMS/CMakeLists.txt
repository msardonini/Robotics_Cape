
cmake_minimum_required(VERSION 2.8)

#project(flyMS C CXX)
project(flyMS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})


include_directories(
	..
	${MATRIX_DIR}
	${EIGEN3_INCLUDE_DIR}
	)

set(SRCS
	src/config.c
	src/flyMS.c
	src/ekf.cpp
	src/filter.c
	src/gps.c
	src/kalman.c
	src/linear_algebraMS.c
	src/logger.c
	src/misc_functions.c
	)

	#-Werror

add_definitions(-DPOSIX_SHARED)

add_compile_options(
	-std=c++11
	-Wall
	-Wno-deprecated-declarations
	-Wno-enum-compare
	-Wno-unused-local-typedefs
	-Wno-unused-parameter
	)




find_library(ROBOTICSCAPE_LIB roboticscape)
find_library(ECL_LIB ecl)


add_executable(flyMS ${SRCS})
target_link_libraries(flyMS m)
target_link_libraries(flyMS rt)
target_link_libraries(flyMS pthread)
target_link_libraries(flyMS ecl)
target_link_libraries(flyMS ${ROBOTICSCAPE_LIB})


cmake_minimum_required(VERSION 2.8)

project(FILTER)

set(FILTER_DIR lib/filter/)

include_directories(
	${FILTER_DIR}
)

set(FILTER_SRCS
	${FILTER_DIR}filter.c
)


add_library(filter SHARED ${FILTER_SRCS})

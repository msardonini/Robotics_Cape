
cmake_minimum_required(VERSION 2.8)

project(FILTER)

set(FILTER_DIR ./)

include_directories(
	${FILTER_DIR}
)

set(FILTER_SRCS
	${FILTER_DIR}filter.c
)


add_library(filter SHARED ${FILTER_SRCS})

target_include_directories(filter PUBLIC ${FILTER_DIR})

install(TARGETS filter
	DESTINATION /usr/lib)

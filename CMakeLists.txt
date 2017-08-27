cmake_minimum_required(VERSION 2.8)

project(flyMS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_compile_options(
	-Wall
	-Wno-deprecated-declarations
	-Wno-enum-compare
	-Wno-unused-local-typedefs
	-Wno-unused-parameter
	)

include(libraries/ecl/EKF/CMakeLists.txt)
include(libraries/CMakeLists.txt)
include(examples/flyMS/CMakeLists.txt)


find_library(ECL_LIB ecl)
find_library(ROBOTICSCAPE_LIB roboticscape)


add_executable(flyMS ${SRCS_FLYMS})
target_link_libraries(flyMS m)
target_link_libraries(flyMS rt)
target_link_libraries(flyMS pthread)
target_link_libraries(flyMS ecl)
target_link_libraries(flyMS roboticscape)
#target_link_libraries(flyMS ${ROBOTICSCAPE_LIB})


install(TARGETS flyMS DESTINATION "/usr/bin/")

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
	
#Build the PX4 estimation control library
include(lib/ecl/CMakeLists.txt)

#Build the robotics cape library
include(lib/robotics_cape/CMakeLists.txt)

#Get the flyMS sources
include(examples/flyMS/CMakeLists.txt)

set(PRU_HANDLER_SRS
	examples/pru_handler/src/pru_handler.c)

add_executable(pru_handler ${PRU_HANDLER_SRS})	
target_link_libraries(pru_handler roboticscape)
target_link_libraries(pru_handler m)
target_link_libraries(pru_handler pthread)

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

#Define install directory
install(TARGETS flyMS DESTINATION /usr/bin/)
install(TARGETS pru_handler DESTINATION /usr/bin/)
install(TARGETS roboticscape DESTINATION /usr/lib/)
install(TARGETS ecl DESTINATION /usr/lib/)

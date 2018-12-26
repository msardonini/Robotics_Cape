cmake_minimum_required(VERSION 2.8)

project(flyMS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_compile_options(
	# -Wall
	-Wno-deprecated-declarations
	-Wno-enum-compare
	-Wno-unused-local-typedefs
	-Wno-unused-parameter
	)

add_definitions(-DFLYMS_ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
	
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen3/cmake)
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
	include_directories(${EIGEN3_INCLUDE_DIR})
else()
	message(FATAL_ERROR "Cannot Find Eigen3")
endif()

# Sets variable to a value if variable is unset.
#Build the robotics cape library
include(cmake/px4_add_library.cmake)
add_subdirectory(lib/librobotcontrol)
include_directories(lib/librobotcontrol/library/include)

# #Build the Yaml-Cpp config file parser
add_subdirectory(lib/yaml-cpp)

# #Build the PX4 estimation control library
add_subdirectory(lib/ecl)

# #Build the Accel/Gyro Fusion library
add_subdirectory(lib/Fusion)

# #Build the Digital Filter Library
add_subdirectory(lib/filter)

# #Get the do_nothing sources
# include(src/do_nothing/CMakeLists.txt)

# #Get the flyMS sources
add_subdirectory(src/flyMS)

#Build the pru_handler
add_subdirectory(src/pru_handler)

# #Build rc_test_imu
# include(src/rc_test_imu/CMakeLists.txt)

# #Build rc_test_fusion
# include(src/rc_test_fusion/CMakeLists.txt)

# #Build rc_test_barometer
# include(src/rc_test_barometer/CMakeLists.txt)

# #Build rc_test_dsm
# include(src/rc_test_dsm/CMakeLists.txt)

# #Build rc_test_dmp
# include(src/rc_test_dmp/CMakeLists.txt)

# #Build rc_calibrate_gyro
# include(src/rc_calibrate_gyro/CMakeLists.txt)

# #Build rc_calibrate_escs
# include(src/rc_calibrate_escs/CMakeLists.txt)

# #Build rc_calibrate_mag
# include(src/rc_calibrate_mag/CMakeLists.txt)

# #Build rc_calibrate_dsm
# include(src/rc_calibrate_dsm/CMakeLists.txt)

#Define install directory
# install(TARGETS flyMS DESTINATION /usr/bin/)
# install(TARGETS pru_handler DESTINATION /usr/bin/)
# install(TARGETS roboticscape DESTINATION /usr/lib/)
# install(TARGETS ecl DESTINATION /usr/lib/)
# install(TARGETS Fusion DESTINATION /usr/lib)
# install(TARGETS filter DESTINATION /usr/lib)
# install(TARGETS do_nothing DESTINATION /usr/bin)

# install(TARGETS rc_calibrate_dsm DESTINATION /usr/bin/)
# install(TARGETS rc_calibrate_gyro DESTINATION /usr/bin/)
# install(TARGETS rc_calibrate_escs DESTINATION /usr/bin/)
# install(TARGETS rc_calibrate_mag DESTINATION /usr/bin/)

# install(TARGETS rc_test_fusion DESTINATION /usr/bin/)
# install(TARGETS rc_test_imu DESTINATION /usr/bin/)
# install(TARGETS rc_test_barometer DESTINATION /usr/bin/)
# install(TARGETS rc_test_dsm DESTINATION /usr/bin/)
# install(TARGETS rc_test_dmp DESTINATION /usr/bin/)

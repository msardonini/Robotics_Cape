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

add_definitions(-DFLYMS_ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
	
add_subdirectory(lib/yaml-cpp)

#Build the PX4 estimation control library
include(lib/ecl/CMakeLists.txt)

#Build the robotics cape library
include(lib/robotics_cape/CMakeLists.txt)

#Build the Accel/Gyro Fusion library
include(lib/Fusion/CMakeLists.txt)

#Build the Accel/Gyro Fusion library
include(lib/filter/CMakeLists.txt)

#Get the do_nothing sources
include(src/do_nothing/CMakeLists.txt)

#Get the flyMS sources
include(src/flyMS/CMakeLists.txt)

#Build the pru_handler
include(src/pru_handler/CMakeLists.txt)

#Build rc_test_imu
include(src/rc_test_imu/CMakeLists.txt)

#Build rc_test_fusion
include(src/rc_test_fusion/CMakeLists.txt)

#Build rc_test_barometer
include(src/rc_test_barometer/CMakeLists.txt)

#Build rc_test_dsm
include(src/rc_test_dsm/CMakeLists.txt)

#Build rc_test_dmp
include(src/rc_test_dmp/CMakeLists.txt)

#Build rc_calibrate_gyro
include(src/rc_calibrate_gyro/CMakeLists.txt)

#Build rc_calibrate_escs
include(src/rc_calibrate_escs/CMakeLists.txt)

#Build rc_calibrate_mag
include(src/rc_calibrate_mag/CMakeLists.txt)

#Build rc_calibrate_dsm
include(src/rc_calibrate_dsm/CMakeLists.txt)

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

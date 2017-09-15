cmake_minimum_required(VERSION 2.8)

set(RC_CALIBRATE_MAG_SRCS
          src/rc_calibrate_mag/rc_calibrate_mag.c)
  
 
add_executable(rc_calibrate_mag ${RC_CALIBRATE_MAG_SRCS})
target_link_libraries(rc_calibrate_mag  roboticscape)
target_link_libraries(rc_calibrate_mag m)
target_link_libraries(rc_calibrate_mag pthread)


cmake_minimum_required(VERSION 2.8)

set(RC_CALIBRATE_ESCS_SRCS
          src/rc_calibrate_escs/rc_calibrate_escs.c)
  
 
add_executable(rc_calibrate_escs ${RC_CALIBRATE_ESCS_SRCS})
target_link_libraries(rc_calibrate_escs roboticscape)
target_link_libraries(rc_calibrate_escs m)
target_link_libraries(rc_calibrate_escs pthread)


cmake_minimum_required(VERSION 2.8)

set(RC_CALIBRATE_SRCS
          src/rc_calibrate_dsm/rc_calibrate_dsm.c)
  
 
add_executable(rc_calibrate_dsm ${RC_CALIBRATE_SRCS})
target_link_libraries(rc_calibrate_dsm roboticscape)
target_link_libraries(rc_calibrate_dsm m)
target_link_libraries(rc_calibrate_dsm pthread)


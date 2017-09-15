cmake_minimum_required(VERSION 2.8)

set(RC_CALIBRATE_GYRO_SRCS
          src/rc_calibrate_gyro/rc_calibrate_gyro.c)
  
 
add_executable(rc_calibrate_gyro ${RC_CALIBRATE_GYRO_SRCS})
target_link_libraries(rc_calibrate_gyro roboticscape)
target_link_libraries(rc_calibrate_gyro m)
target_link_libraries(rc_calibrate_gyro pthread)


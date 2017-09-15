cmake_minimum_required(VERSION 2.8)

set(RC_TEST_IMU_SRCS
          src/rc_test_imu/rc_test_imu.c)
  
 
add_executable(rc_test_imu ${RC_TEST_IMU_SRCS})
target_link_libraries(rc_test_imu roboticscape)
target_link_libraries(rc_test_imu m)
target_link_libraries(rc_test_imu pthread)

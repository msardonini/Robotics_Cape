cmake_minimum_required(VERSION 2.8)

set(RC_TEST_BAROMETER_SRCS
          src/rc_test_barometer/rc_test_barometer.c)
  
 
add_executable(rc_test_barometer ${RC_TEST_BAROMETER_SRCS})
target_link_libraries(rc_test_barometer roboticscape)
target_link_libraries(rc_test_barometer m)
target_link_libraries(rc_test_barometer pthread)

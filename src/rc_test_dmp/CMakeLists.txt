cmake_minimum_required(VERSION 2.8)

set(RC_TEST_DMP_SRCS
          src/rc_test_dmp/rc_test_dmp.c)
  
 
add_executable(rc_test_dmp ${RC_TEST_DMP_SRCS})
target_link_libraries(rc_test_dmp roboticscape)
target_link_libraries(rc_test_dmp m)
target_link_libraries(rc_test_dmp pthread)


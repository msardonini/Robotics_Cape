cmake_minimum_required(VERSION 2.8)

set(RC_TEST_DSM_SRCS
          src/rc_test_dsm/rc_test_dsm.c)
  
 
add_executable(rc_test_dsm ${RC_TEST_DSM_SRCS})
target_link_libraries(rc_test_dsm roboticscape)
target_link_libraries(rc_test_dsm m)
target_link_libraries(rc_test_dsm pthread)


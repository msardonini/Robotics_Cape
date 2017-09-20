cmake_minimum_required(VERSION 2.8)

set(RC_TEST_FUSION_SRCS
          src/rc_test_fusion/rc_test_fusion.c)
  
 
add_executable(rc_test_fusion ${RC_TEST_FUSION_SRCS})
target_link_libraries(rc_test_fusion roboticscape)
target_link_libraries(rc_test_fusion m)
target_link_libraries(rc_test_fusion pthread)
target_link_libraries(rc_test_fusion Fusion)
target_link_libraries(rc_test_fusion filter)

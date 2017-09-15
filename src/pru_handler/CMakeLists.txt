cmake_minimum_required(VERSION 2.8)

set(PRU_HANDLER_SRS
          examples/pru_handler/pru_handler.c)
  
 
add_executable(pru_handler ${PRU_HANDLER_SRS})
target_link_libraries(pru_handler roboticscape)
target_link_libraries(pru_handler m)
target_link_libraries(pru_handler pthread)


cmake_minimum_required(VERSION 3.5)

set(PRU_HANDLER_SRC
          pruHandlerApp.cpp
          pruHandler.cpp)
  
 
add_executable(pruHandler ${PRU_HANDLER_SRC})


target_link_libraries(pruHandler robotics_cape pthread)
# target_link_libraries(pru_handler m)
# target_link_libraries(pru_handler pthread)


cmake_minimum_required(VERSION 3.5)
 
add_executable(pruHandler
  pruHandlerApp.cpp
  pruHandler.cpp
)

target_link_libraries(pruHandler
  PUBLIC
  robotics_cape
  pthread
)

install(TARGETS pruHandler
  DESTINATION /usr/bin/
)

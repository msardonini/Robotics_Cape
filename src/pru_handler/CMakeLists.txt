cmake_minimum_required(VERSION 3.5)

# Link statically to all libaries
set(CMAKE_EXE_LINKER_FLAGS "-static")

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

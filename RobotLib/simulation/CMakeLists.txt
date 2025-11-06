cmake_minimum_required(VERSION 3.10)
project(RobotLibSimulation)

# C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# Include RobotLib headers
include_directories(../include)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Build examples
add_executable(basic_simulation 01_basic_simulation.cpp)
target_link_libraries(basic_simulation ${SDL2_LIBRARIES})

add_executable(pid_navigation 02_pid_navigation.cpp)
target_link_libraries(pid_navigation ${SDL2_LIBRARIES})

add_executable(line_follower 03_line_follower.cpp)
target_link_libraries(line_follower ${SDL2_LIBRARIES})

# Install targets (optional)
install(TARGETS basic_simulation pid_navigation line_follower
        RUNTIME DESTINATION bin)

# Print info
message(STATUS "RobotLib Simulation Examples")
message(STATUS "  SDL2 found: ${SDL2_LIBRARIES}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build examples:")
message(STATUS "    - basic_simulation")
message(STATUS "    - pid_navigation")
message(STATUS "    - line_follower")

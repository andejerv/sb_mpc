cmake_minimum_required(VERSION 3.15)
project(Examples)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# specify the header files location
include_directories(../include/SB_MPC)

# add all source files to build
file(GLOB SOURCES "../src/*.cpp")

# build example programs
#add_executable(example1 example1.cpp ${SOURCES})
#add_executable(example2 example2.cpp ${SOURCES})
add_executable(mpc_test mpc_test.cpp ${SOURCES})

add_executable(sfml_sim sfml_sim.cpp ${SOURCES})
target_link_libraries(sfml_sim sfml-graphics sfml-window sfml-system)
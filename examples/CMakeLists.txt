cmake_minimum_required(VERSION 3.15)
project(Examples)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# specify the location of the library's headers
include_directories(../include)

# build an example program
add_executable(example1 example1.cpp)
target_link_libraries(example1 SB_MPC)

# build another example program
add_executable(example2 example2.cpp)
target_link_libraries(example2 SB_MPC)
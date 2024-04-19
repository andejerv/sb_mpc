cmake_minimum_required(VERSION 3.15)
project(Examples)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# specify the header files location
include_directories(../include/SB_MPC)

# add all source files to build
file(GLOB SOURCES "../src/*.cpp")

# build an example program
add_executable(example1 example1.cpp ${SOURCES})
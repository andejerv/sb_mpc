cmake_minimum_required(VERSION 3.29.2)

project(blueboat_sb_mpc
    VERSION 1.0
    DESCRIPTION "A brief CMake experiment"
    LANGUAGES CXX)

add_executable(sb_mpc_test
            main.cpp
            sb_mpc.cpp)

target_compile_options(myApp PRIVATE -Wall -Wextra -Wpedantic)
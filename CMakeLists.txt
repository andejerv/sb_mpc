cmake_minimum_required(VERSION 3.22.0)

project(blueboat_sb_mpc
    VERSION 1.0
    DESCRIPTION "A brief CMake experiment"
    LANGUAGES CXX)

add_executable(sb_mpc_test
            main.cpp
            sb_mpc.cpp
            bb_model.cpp
            object.cpp
            utilities.cpp)

target_compile_options(sb_mpc_test PRIVATE -Wall -Wextra -Wpedantic)
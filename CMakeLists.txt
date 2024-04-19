cmake_minimum_required(VERSION 3.15)
project(SB_MPC)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# specify the header files location
include_directories(include)

# specify the source files location
file(GLOB SOURCES "src/*.cpp")

# generate the static library from the sources
add_library(SB_MPC STATIC ${SOURCES})

# specify where the public headers of this library are
target_include_directories(SB_MPC PUBLIC include)
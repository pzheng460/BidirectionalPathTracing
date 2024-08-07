cmake_minimum_required(VERSION 3.10)
project(BidirectionalPathTracing)

set(CMAKE_CXX_STANDARD 17)

# Add the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.hpp")

# Add the executable with all source files
add_executable(BidirectionalPathTracing ${SOURCES})

cmake_minimum_required(VERSION 3.10)
project(poly-exploration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTest)

include_directories(include)

add_subdirectory(src) 

enable_testing()
add_subdirectory(test)

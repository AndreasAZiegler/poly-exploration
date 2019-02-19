cmake_minimum_required(VERSION 3.10)
project(poly-exploration)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")

# Set the compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-g -O0 -Wall")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTest)

find_package(kindr) 
include_directories(${kindr_INCLUDE_DIRS}) 

include(CodeCoverage)
set(COVERAGE_LCOV_EXCLUDES '/usr/include/*')

include_directories(include)

add_subdirectory(src) 


enable_testing()
add_subdirectory(test)

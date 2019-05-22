cmake_minimum_required(VERSION 3.10)
project(poly_exploration
  LANGUAGES CXX
  VERSION 0.1.0
)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")

# Set the compiler options
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "-g -ggdb3 -O0 -Wall")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g -ggdb3 -Wall -Wall -Wextra -Wpedantic)


#find_package(Glog REQUIRED)
#include_directories(${GLOG_INCLUDE_DIRS})

#find_package(GTest)

#find_package(kindr) 
#include_directories(${kindr_INCLUDE_DIRS}) 

#include_directories(${PROJECT_SOURCE_DIR}/libs/eventpp-master/include)

#include(CodeCoverage)
#set(COVERAGE_LCOV_EXCLUDES '/usr/include/*')

#include_directories(include/poly-exploration)

add_subdirectory(libs/kindr)

add_subdirectory(src) 


enable_testing()
add_subdirectory(test)

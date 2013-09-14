################################################################################
#-------------------------------------------------------------------------------
# Project setup.
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)

project(LUM_BENCHMARK)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

#-------------------------------------------------------------------------------
# Directory setup.
#-------------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

message("Source tree path: ${PROJECT_SOURCE_DIR}")
message("Binary tree path: ${PROJECT_BINARY_DIR}")

#-------------------------------------------------------------------------------
# Compiler flags.
#-------------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -std=c++11 -O2 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=c++11 -O2")

#-------------------------------------------------------------------------------
# Subdirectories.
#-------------------------------------------------------------------------------
add_subdirectory(src)
add_subdirectory(include)
################################################################################

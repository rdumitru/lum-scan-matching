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

set(CMAKE_BUILD_TYPE Debug)
#-------------------------------------------------------------------------------
# Directory setup.
#-------------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(3RDPARTY_DIR ${PROJECT_SOURCE_DIR}/3rdparty/3dtk-1.2)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${3RDPARTY_DIR}/include)
include_directories(${3RDPARTY_DIR}/3rdparty/)
include_directories(${3RDPARTY_DIR}/3rdparty/glui)
include_directories(${3RDPARTY_DIR}/3rdparty/wxthings/include/)
include_directories(${3RDPARTY_DIR}/3rdparty/ann_1.1.1_modified/include)
message(${3RDPARTY_DIR})
include_directories(${3RDPARTY_DIR}/3rdparty/ann_1.1.1_modified/src)
link_directories(${3RDPARTY_DIR}/obj)
link_directories(${3RDPARTY_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/lib)

message("Source tree path: ${PROJECT_SOURCE_DIR}")
message("Binary tree path: ${PROJECT_BINARY_DIR}")

#-------------------------------------------------------------------------------
# Compiler flags.
#-------------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")

#-------------------------------------------------------------------------------
# Subdirectories.
#-------------------------------------------------------------------------------
add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(3rdparty/3dtk-1.2)
################################################################################

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(superVoxels)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -pthread")

set(PCL_BUILD_TYPE Release)

file(GLOB superVoxels_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(superVoxels ${superVoxels_SRC})

target_link_libraries (superVoxels ${PCL_LIBRARIES})

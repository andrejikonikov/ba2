cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_ground_separator)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB PCL_ground_separator_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(ground_separator ${PCL_ground_separator_SRC})

target_link_libraries (ground_separator ${PCL_LIBRARIES})

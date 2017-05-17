cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(viewer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

file(GLOB viewer_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(Viewer ${viewer_SRC})

target_link_libraries (Viewer ${PCL_LIBRARIES})

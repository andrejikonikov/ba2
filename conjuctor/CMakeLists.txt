cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(conjuctor)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS system filesystem REQUIRED)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -pthread")

set(PCL_BUILD_TYPE Release)

file(GLOB conjuctor_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(conjuctor ${conjuctor_SRC})

target_link_libraries (conjuctor ${PCL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

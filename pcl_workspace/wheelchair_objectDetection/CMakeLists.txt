cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(wheelchair_obj)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (wheelchair_obj wheelchair_obj.cpp)
target_link_libraries (wheelchair_obj ${PCL_LIBRARIES})

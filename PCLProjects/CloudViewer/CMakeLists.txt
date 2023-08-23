cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLProjects)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCLProjects main.cpp CommonProcesses.cpp Segmentation.cpp RegionGrowing.cpp RANSAC.cpp)
target_link_libraries (PCLProjects ${PCL_LIBRARIES})
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pc2path)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pc2path src/pc2path.cpp)
target_link_libraries (pc2path ${PCL_LIBRARIES})

add_executable (pcl_stl_sample src/pcl_stl_sample.cpp)
target_link_libraries (pcl_stl_sample ${PCL_LIBRARIES})

add_executable (extract_indices src/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

add_executable (viewer src/viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_write)

set( CMAKE_CXX_FLAGS "-std=c++11 -O0 -g" )
set( CMAKE_C_FLAGS "-std=c++11 -O0 -g" )
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcl_lesson_1.cc)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (ply_transform transform_cloud.cc)
target_link_libraries (ply_transform ${PCL_LIBRARIES})

add_executable (my_point_t my_point_t.cc)
target_link_libraries (my_point_t  ${PCL_LIBRARIES})

message(-----------------------)
message(${PCL_LIBRARIES})
message(-----------------------)
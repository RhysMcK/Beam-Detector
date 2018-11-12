cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

project(pcl_beam_detection)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (beam_detector src/beam_detector.cpp)
target_link_libraries (beam_detector ${PCL_LIBRARIES})

add_executable (beam_cluster src/beam_cluster.cpp)
target_link_libraries (beam_cluster ${PCL_LIBRARIES})

add_executable (view_pcd src/view_pcd.cpp)
target_link_libraries (view_pcd ${PCL_LIBRARIES})

add_executable (correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (Global_hypothesis_verification src/Global_hypothesis_verification.cpp)
target_link_libraries (Global_hypothesis_verification ${PCL_LIBRARIES})
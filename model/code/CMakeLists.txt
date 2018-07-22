cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(GeneratePointCloudFromPositionData)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(GeneratePointCloudFromPositionData GeneratePointCloudFromPositionData.cpp)
target_link_libraries(GeneratePointCloudFromPositionData ${PCL_LIBRARIES})

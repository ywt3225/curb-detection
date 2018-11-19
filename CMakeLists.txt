cmake_minimum_required(VERSION 2.8.3)
project(velodyne_points_filter)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(velodyne_points_filter velodyne_points_filter.cpp)
target_link_libraries(velodyne_points_filter ${PCL_LIBRARIES} ${catkin_LIBRARIES})
  



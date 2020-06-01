cmake_minimum_required(VERSION 2.8.3)
project(laser_to_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_to_pointcloud
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs tf
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node src/node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(node
   ${catkin_LIBRARIES}
 )

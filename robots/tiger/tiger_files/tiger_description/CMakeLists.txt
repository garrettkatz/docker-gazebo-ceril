cmake_minimum_required(VERSION 2.8.3)
project(tiger_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	#  roscpp
	#  urdf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tiger_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


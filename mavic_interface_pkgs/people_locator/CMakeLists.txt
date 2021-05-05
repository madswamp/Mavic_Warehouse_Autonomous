cmake_minimum_required(VERSION 3.0.2)
project(people_locator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_locator
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2_ros
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(people_locator src/people_locator.cpp src/people_locator.cpp)
target_link_libraries(people_locator ${catkin_LIBRARIES})
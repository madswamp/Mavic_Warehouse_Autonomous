cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME mavic_transform_frames)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

set(MAVIC_TRANSFORM_FRAMES_SOURCE_DIR)

set(MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR
        include/mavic_transform_frames)

set(MAVIC_TRANSFORM_FRAMES_LIB_FILES
        include/mavic_transform_frames/mavic_transform_frames.h)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        aerostack_msgs)


catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs aerostack_msgs)


include_directories(
        ${catkin_INCLUDE_DIRS} ${MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR}
)

add_library(mavic_transform_frames_include ${MAVIC_TRANSFORM_FRAMES_LIB_FILES})
add_dependencies(mavic_transform_frames_include ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames_include ${catkin_LIBRARIES})

add_executable(mavic_transform_frames  src/mavic_transform_frames.cpp)
add_dependencies(mavic_transform_frames ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames mavic_transform_frames_include)
target_link_libraries(mavic_transform_frames ${catkin_LIBRARIES})

set_target_properties(mavic_transform_frames PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_transform_frames_include PROPERTIES LINKER_LANGUAGE CXX)


















#[[find_package(catkin REQUIRED COMPONENTS
  aerostack_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mavic_transform_frames
#  CATKIN_DEPENDS aerostack_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)]]


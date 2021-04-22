cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME mavic_interface_ros)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

set(MAVIC_TRANSFORM_FRAMES_SOURCE_DIR)

set(MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR
        include/mavic_transform_frames)

set(MAVIC_TRANSFORM_FRAMES_LIB_FILES
        include/mavic_interface_ros/mavic_transform_frames/mavic_transform_frames.h
        src/mavic_transform_frames/mavic_transform_frames.cpp)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        aerostack_msgs
        tf)


catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs aerostack_msgs)


include_directories(
        ${catkin_INCLUDE_DIRS} ${MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR}
)

add_library(mavic_transform_frames_include ${MAVIC_TRANSFORM_FRAMES_LIB_FILES})
add_dependencies(mavic_transform_frames_include ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames_include ${catkin_LIBRARIES})

add_executable(mavic_transform_frames_node src/mavic_transform_frames/mavic_transform_frames_node.cpp)
add_dependencies(mavic_transform_frames_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames_node  mavic_transform_frames_include)
target_link_libraries(mavic_transform_frames_node  ${catkin_LIBRARIES})

set_target_properties(mavic_transform_frames_node  PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_transform_frames_include PROPERTIES LINKER_LANGUAGE CXX)


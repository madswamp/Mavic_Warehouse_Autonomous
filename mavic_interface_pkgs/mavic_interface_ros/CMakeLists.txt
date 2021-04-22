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


set(MAVIC_STATE_INCLUDE_DIR
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_state)

set(MAVIC_STATE_LIB_FILES
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_state/mavic_state.h
        src/mavic_interface_aerostack/mavic_state/mavic_state.cpp)


set(MAVIC_RAW_LOCALIZATION_LIB_FILES
        include/mavic_interface_ros/mavic_raw_localization/mavic_raw_localization.h
        src/mavic_raw_localization/mavic_raw_localization.cpp)

set(MAVIC_RAW_LOCALIZATION_INCLUDE_DIR
        include/mavic_interface_ros/mavic_raw_localization)

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
        ${catkin_INCLUDE_DIRS} ${MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR}  ${MAVIC_STATE_INCLUDE_DIR} ${MAVIC_RAW_LOCALIZATION_INCLUDE_DIR}
)

add_library(mavic_transform_frames_include ${MAVIC_TRANSFORM_FRAMES_LIB_FILES})
add_dependencies(mavic_transform_frames_include ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames_include ${catkin_LIBRARIES})

add_library(mavic_state_includes ${MAVIC_STATE_LIB_FILES})
add_dependencies(mavic_state_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_state_includes ${catkin_LIBRARIES})

add_library(mavic_raw_localization_includes ${MAVIC_RAW_LOCALIZATION_LIB_FILES})
add_dependencies(mavic_raw_localization_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_raw_localization_includes ${catkin_LIBRARIES})


add_executable(mavic_transform_frames_node src/mavic_transform_frames/mavic_transform_frames_node.cpp)
add_dependencies(mavic_transform_frames_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_transform_frames_node  mavic_transform_frames_include)
target_link_libraries(mavic_transform_frames_node  ${catkin_LIBRARIES})

add_executable(mavic_state_node src/mavic_interface_aerostack/mavic_state/mavic_state_node.cpp)
add_dependencies(mavic_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_state_node mavic_state_includes)
target_link_libraries(mavic_state_node ${catkin_LIBRARIES})

add_executable(mavic_raw_localization_node src/mavic_raw_localization/mavic_raw_localization_node.cpp)
add_dependencies(mavic_raw_localization_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_raw_localization_node  mavic_raw_localization_includes)
target_link_libraries(mavic_raw_localization_node  ${catkin_LIBRARIES})



set_target_properties(mavic_transform_frames_node  PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_transform_frames_include PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_state_node PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_state_includes PROPERTIES LINKER_LANGUAGE CXX)


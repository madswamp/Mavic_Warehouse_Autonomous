cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME mavic_interface_ros)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


set(MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR
        include/mavic_transform_frames)

set(MAVIC_STATE_INCLUDE_DIR
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_state)

set(MAVIC_RAW_LOCALIZATION_INCLUDE_DIR
        include/mavic_interface_ros/mavic_raw_localization)

set(MAVIC_RECEIVE_VIDEO_INCLUDE_DIR
        include/mavic_interface_ros/mavic_receive_video)

set(MAVIC_TAGS_WORLD_TF_INCLUDE_DIR
        include/mavic_interface_ros/mavic_tags_world_tf)



set(MAVIC_TRANSFORM_FRAMES_LIB_FILES
        include/mavic_interface_ros/mavic_transform_frames/mavic_transform_frames.h
        src/mavic_transform_frames/mavic_transform_frames.cpp)

set(MAVIC_STATE_LIB_FILES
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_state/mavic_state.h
        src/mavic_interface_aerostack/mavic_state/mavic_state.cpp)


set(MAVIC_RAW_LOCALIZATION_LIB_FILES
        include/mavic_interface_ros/mavic_raw_localization/mavic_raw_localization.h
        src/mavic_raw_localization/mavic_raw_localization.cpp)

set(MAVIC_RECEIVE_VIDEO_LIB_FILES
        include/mavic_interface_ros/mavic_receive_video/mavic_receive_video.h
        src/mavic_receive_video/mavic_receive_video.cpp)

set(MAVIC_TAGS_WORLD_TF_LIB_FILES
        include/mavic_interface_ros/mavic_tags_world_tf/mavic_tags_world_tf.h
        src/mavic_tags_world_tf/mavic_tags_world_tf.cpp)



find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        aerostack_msgs
        tf
        camera_info_manager
        apriltag_ros)


find_package(PkgConfig)
find_package( OpenCV REQUIRED )


pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})


catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs aerostack_msgs cv_bridge camera_info_manager
        DEPENDS apriltag)


include_directories(
        ${catkin_INCLUDE_DIRS} ${MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR}  ${MAVIC_STATE_INCLUDE_DIR} ${MAVIC_RAW_LOCALIZATION_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${apriltag_INCLUDE_DIRS}
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

add_library(mavic_receive_video_includes ${MAVIC_RECEIVE_VIDEO_LIB_FILES})
add_dependencies(mavic_receive_video_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_receive_video_includes ${catkin_LIBRARIES} ${OpenCV_LIBS})

#[[add_library(mavic_tags_world_tf_includes ${MAVIC_TAGS_WORLD_TF_LIB_FILES})
add_dependencies(mavic_tags_world_tf_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_tags_world_tf_includes ${catkin_LIBRARIES})]]



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

add_executable(mavic_receive_video_node src/mavic_receive_video/mavic_receive_video_node.cpp)
add_dependencies(mavic_receive_video_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_receive_video_node  mavic_receive_video_includes)
target_link_libraries(mavic_receive_video_node  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#[[add_executable(mavic_tags_world_tf_node src/mavic_tags_world_tf/mavic_tags_world_tf_node.cpp)
add_dependencies(mavic_tags_world_tf_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_tags_world_tf_node  mavic_tags_world_tf_includes)
target_link_libraries(mavic_tags_world_tf_node  ${catkin_LIBRARIES})]]





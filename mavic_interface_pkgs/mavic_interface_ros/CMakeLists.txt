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

set(MAVIC_SLAM_GTSAM_INCLUDE_DIR
        include/mavic_interface_ros/mavic_slam_gtsam)

set(MAVIC_OPTIMIZED_FRAMES_INCLUDE_DIR
        include/mavic_interface_ros/mavic_optimized_frames)

set(MAVIC_DUNKER_POSE_INCLUDE_DIR
        include/mavic_interface_ros/mavic_dunker_pose)

set(MAVIC_OPTIMIZED_LOCALIZATION_AEROSTACK_INCLUDE_DIR
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_optimized_localization)

set(MAVIC_SEND_VELOCITY_AIRCRAFT_INCLUDE_DIR
        include/mavic_interface_ros/mavic_send_velocity_aircraft)

set(MAVIC_SEND_WAYPOINTS_FREE_BOT_INCLUDE_DIR
        include/mavic_interface_ros/mavic_send_waypoints_free_bot)



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

set(MAVIC_SLAM_GTSAM_LIB_FILES
        include/mavic_interface_ros/mavic_slam_gtsam/mavic_slam_gtsam.h
        src/mavic_slam_gtsam/mavic_slam_gtsam.cpp)

set(MAVIC_OPTIMIZED_FRAMES_LIB_FILES
        include/mavic_interface_ros/mavic_optimized_frames/mavic_optimized_frames.h
        src/mavic_optimized_frames/mavic_optimized_frames.cpp)

set(MAVIC_DUNKER_POSE_LIB_FILES
        include/mavic_interface_ros/mavic_dunker_pose/mavic_dunker_pose.h
        src/mavic_dunker_pose/mavic_dunker_pose.cpp)

set(MAVIC_OPTIMIZED_LOCALIZATION_AEROSTACK_LIB_FILES
        include/mavic_interface_ros/mavic_interface_aerostack/mavic_optimized_localization/mavic_optimized_localization.h
        src/mavic_interface_aerostack/mavic_optimized_localization/mavic_optimized_localization.cpp)

set(MAVIC_SEND_VELOCITY_AIRCRAFT_LIB_FILES
        include/mavic_interface_ros/mavic_send_velocity_aircraft/mavic_send_velocity_aircraft.h
        src/mavic_send_velocity_aircraft/mavic_send_velocity_aircraft.cpp)

set(MAVIC_SEND_WAYPOINTS_FREE_BOT_LIB_FILES
        include/mavic_interface_ros/mavic_send_waypoints_free_bot/mavic_send_waypoints_free_bot.h
        src/mavic_send_waypoints_free_bot/mavic_send_waypoints_free_bot.cpp)



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
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})


pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})


catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs aerostack_msgs cv_bridge camera_info_manager
        DEPENDS apriltag)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${MAVIC_TRANSFORM_FRAMES_INCLUDE_DIR}
        ${MAVIC_STATE_INCLUDE_DIR}
        ${MAVIC_RAW_LOCALIZATION_INCLUDE_DIR}
        ${MAVIC_RECEIVE_VIDEO_INCLUDE_DIR}
        ${MAVIC_SLAM_GTSAM_INCLUDE_DIR}
        ${MAVIC_OPTIMIZED_FRAMES_INCLUDE_DIR}
        ${MAVIC_DUNKER_POSE_INCLUDE_DIR}
        ${MAVIC_OPTIMIZED_LOCALIZATION_AEROSTACK_INCLUDE_DIR}
        ${MAVIC_SEND_VELOCITY_AIRCRAFT_INCLUDE_DIR}
        ${MAVIC_SEND_WAYPOINTS_FREE_BOT_INCLUDE_DIR}
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

add_library(mavic_slam_gtsam_includes ${MAVIC_SLAM_GTSAM_LIB_FILES})
add_dependencies(mavic_slam_gtsam_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_slam_gtsam_includes gtsam ${catkin_LIBRARIES})

add_library(mavic_optimized_frames_includes ${MAVIC_OPTIMIZED_FRAMES_LIB_FILES})
add_dependencies(mavic_optimized_frames_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_optimized_frames_includes ${catkin_LIBRARIES})

add_library(mavic_dunker_pose_includes ${MAVIC_DUNKER_POSE_LIB_FILES})
add_dependencies(mavic_dunker_pose_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_dunker_pose_includes ${catkin_LIBRARIES})

add_library(mavic_optimized_localization_aerostack_includes ${MAVIC_OPTIMIZED_LOCALIZATION_AEROSTACK_LIB_FILES})
add_dependencies(mavic_optimized_localization_aerostack_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_optimized_localization_aerostack_includes ${catkin_LIBRARIES})

add_library(mavic_send_velocity_aircraft_includes ${MAVIC_SEND_VELOCITY_AIRCRAFT_LIB_FILES})
add_dependencies(mavic_send_velocity_aircraft_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_send_velocity_aircraft_includes ${catkin_LIBRARIES})

add_library(mavic_send_waypoints_free_bot_includes ${MAVIC_SEND_WAYPOINTS_FREE_BOT_LIB_FILES})
add_dependencies(mavic_send_waypoints_free_bot_includes ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_send_waypoints_free_bot_includes ${catkin_LIBRARIES})



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

add_executable(mavic_slam_gtsam_node src/mavic_slam_gtsam/mavic_slam_gtsam_node.cpp)
add_dependencies(mavic_slam_gtsam_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_slam_gtsam_node  mavic_slam_gtsam_includes)
target_link_libraries(mavic_slam_gtsam_node gtsam ${catkin_LIBRARIES})

add_executable(mavic_optimized_frames_node src/mavic_optimized_frames/mavic_optimized_frames_node.cpp)
add_dependencies(mavic_optimized_frames_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_optimized_frames_node  mavic_optimized_frames_includes)
target_link_libraries(mavic_optimized_frames_node ${catkin_LIBRARIES})

add_executable(mavic_dunker_pose_node src/mavic_dunker_pose/mavic_dunker_pose_node.cpp)
add_dependencies(mavic_dunker_pose_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_dunker_pose_node  mavic_dunker_pose_includes)
target_link_libraries(mavic_dunker_pose_node ${catkin_LIBRARIES})

add_executable(mavic_optimized_localization_aerostack_node src/mavic_interface_aerostack/mavic_optimized_localization/mavic_optimized_localization_node.cpp)
add_dependencies(mavic_optimized_localization_aerostack_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_optimized_localization_aerostack_node  mavic_optimized_localization_aerostack_includes)
target_link_libraries(mavic_optimized_localization_aerostack_node ${catkin_LIBRARIES})

add_executable(mavic_send_velocity_aircraft_node src/mavic_send_velocity_aircraft/mavic_send_velocity_aircraft_node.cpp)
add_dependencies(mavic_send_velocity_aircraft_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_send_velocity_aircraft_node  mavic_send_velocity_aircraft_includes)
target_link_libraries(mavic_send_velocity_aircraft_node ${catkin_LIBRARIES})

add_executable(mavic_send_waypoints_free_bot_node src/mavic_send_waypoints_free_bot/mavic_send_waypoints_free_bot_node.cpp)
add_dependencies(mavic_send_waypoints_free_bot_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_send_waypoints_free_bot_node  mavic_send_waypoints_free_bot_includes)
target_link_libraries(mavic_send_waypoints_free_bot_node ${catkin_LIBRARIES})





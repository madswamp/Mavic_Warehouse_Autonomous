cmake_minimum_required(VERSION 3.0.2)

set(PROJECT_NAME mavic_interface_aerostack)
project(${PROJECT_NAME})

add_definitions(-std=c++11)




set(MAVIC_STATE_LIB_FILES
        include/mavic_interface_aerostack/mavic_state/mavic_state.h
        src/mavic_state/mavic_state.cpp)



find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        aerostack_msgs
        tf)

catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs aerostack_msgs tf)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(mavic_state_includes ${MAVIC_STATE_LIB_FILES})
add_dependencies(mavic_state_includes ${catkin_EXPORTED_TARGETS})

add_executable(mavic_state_node src/mavic_state/mavic_state_node.cpp)
add_dependencies(mavic_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_state_node mavic_state_includes)
target_link_libraries(mavic_state_node ${catkin_LIBRARIES})

set_target_properties(mavic_state_node PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(mavic_state_includes PROPERTIES LINKER_LANGUAGE CXX)
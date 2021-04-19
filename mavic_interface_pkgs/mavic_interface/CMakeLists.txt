cmake_minimum_required(VERSION 3.0.2)

project(mavic_interface)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros mav_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions cv_bridge)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros mav_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions)


include_directories(src/include ${catkin_INCLUDE_DIRS})



add_library(mavic_interface src/include/mavic_command.h src/include/mavic_image.h)
add_dependencies(mavic_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_interface ${catkin_LIBRARIES})

add_executable(mavic_command  src/mavic_command.cpp src/include/mavic_command.h)
add_dependencies(mavic_command ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_command mavic_interface)
target_link_libraries(mavic_command ${catkin_LIBRARIES})

add_executable(mavic_state src/mavic_state.cpp src/include/mavic_state.h)
add_dependencies(mavic_state ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_state mavic_interface)
target_link_libraries(mavic_state ${catkin_LIBRARIES})

#[[
add_executable(mavic_image  src/mavic_image.cpp src/include/mavic_image.h)
add_dependencies(mavic_image ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavic_image mavic_interface)
target_link_libraries(mavic_image ${catkin_LIBRARIES})
]]

set_target_properties(mavic_interface PROPERTIES LINKER_LANGUAGE CXX)

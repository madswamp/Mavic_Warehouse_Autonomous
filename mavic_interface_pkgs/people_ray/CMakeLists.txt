cmake_minimum_required(VERSION 3.0.2)
project(people_ray)

set(OpenCV_DIR /usr/share/opencv4/opencv2)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        image_geometry
        roscpp
        rospy
        std_msgs
        opencv_apps
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS geometry_msgs image_geometry roscpp rospy std_msgs opencv_apps
)

include_directories(
        # include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(people_ray src/people_ray.cpp)
target_link_libraries(people_ray ${catkin_LIBRARIES})
target_link_libraries(people_ray ${OpenCV_LIBS})


cmake_minimum_required(VERSION 2.8.3)

project(xju_pnc)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        roslaunch
        sensor_msgs
        tf2
        tf2_ros
        )

catkin_package(CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
        )

include_directories(include ${catkin_INCLUDE_DIRS})

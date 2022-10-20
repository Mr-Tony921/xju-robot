cmake_minimum_required(VERSION 2.8.3)
project(costmap_plugins)

find_package(catkin REQUIRED COMPONENTS
        costmap_2d
        geometry_msgs
        nav_msgs
        message_generation
        roscpp
        )

add_service_files(
        DIRECTORY srv
        FILES
        keepOutZone.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES keep_out_layer
        CATKIN_DEPENDS costmap_2d
        DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} include/${PROJECT_NAME})

add_library(keep_out_layer src/keep_out_layer.cpp)
target_link_libraries(keep_out_layer ${catkin_LIBRARIES})

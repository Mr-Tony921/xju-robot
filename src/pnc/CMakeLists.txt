cmake_minimum_required(VERSION 2.8.3)
project(xju_pnc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        geometry_msgs
        message_generation
        nav_msgs
        roscpp
        roslaunch
        sensor_msgs
        tf2
        tf2_ros
        )

add_service_files(
        DIRECTORY srv
        FILES
        record_start.srv
        record_stop.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
        actionlib_msgs
        geometry_msgs
        message_runtime
        nav_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
        )

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

add_executable(xju_state_machine
        node/state_machine_node.cpp
        src/state_machine.cpp)
add_dependencies(xju_state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_state_machine ${catkin_LIBRARIES})

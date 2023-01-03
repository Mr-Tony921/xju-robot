cmake_minimum_required(VERSION 2.8.3)
project(xju_pnc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        costmap_2d
        coverage_path_planner
        geometry_msgs
        message_generation
        mbf_msgs
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
        xju_task.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
        actionlib_msgs
        costmap_2d
        coverage_path_planner
        geometry_msgs
        message_runtime
        mbf_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
        )

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

add_executable(xju_mbf_bridge
        node/mbf_bridge_node.cpp)
add_dependencies(xju_mbf_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_mbf_bridge ${catkin_LIBRARIES})

add_executable(xju_state_machine
        node/state_machine_node.cpp
        src/state_machine.cpp
        src/half_struct_planner.cpp)
add_dependencies(xju_state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_state_machine ${catkin_LIBRARIES})

add_executable(xju_wall_follow
        node/wall_follow_node.cpp)
add_dependencies(xju_wall_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_wall_follow ${catkin_LIBRARIES})

install(TARGETS xju_mbf_bridge xju_state_machine xju_wall_follow
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch param rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
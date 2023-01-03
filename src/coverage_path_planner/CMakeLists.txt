cmake_minimum_required(VERSION 2.8.3)
project(coverage_path_planner)

add_compile_options(-std=c++17)

SET(SRC_CPP
    node/PathPlanningNode.cpp
    src/CoveragePathPlanner.cpp
    src/BezierTrajectoryGeneratorWaypoint.cpp
    src/FreeSpaceCalculator.cpp)

## Find depends
find_package(catkin REQUIRED COMPONENTS
        angles
        costmap_2d
        roscpp
        rospy
        std_msgs
        std_srvs
        nav_msgs
        map_msgs
        tf
        geometry_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
        FILES
        GetPathInZone.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime costmap_2d geometry_msgs nav_msgs map_msgs roscpp tf message_generation
        LIBRARIES
        DEPENDS
)

###########
## Build ##
###########
# build exec path_planning_node
add_executable(path_planning_node ${SRC_CPP})
add_dependencies(path_planning_node
                 ${PROJECT_NAME}_generate_messages_cpp
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(path_planning_node PRIVATE
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})
# end build exec path_planning_node

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(
        TARGETS path_planning_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

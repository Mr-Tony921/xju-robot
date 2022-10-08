cmake_minimum_required(VERSION 3.1)
project(xju_explore)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  cv_bridge
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    std_msgs
    visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(xju_explore
  node/explore_node.cpp
  src/explore.cpp
  src/frontier_search.cpp
)
add_dependencies(xju_explore ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_explore ${catkin_LIBRARIES} ${OpenCV_LIBS})

cmake_minimum_required(VERSION 2.8.3)

project(xju_slam)

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

add_executable(xju_relocation node/relocation.cpp)

add_dependencies(xju_relocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(xju_relocation ${catkin_LIBRARIES})

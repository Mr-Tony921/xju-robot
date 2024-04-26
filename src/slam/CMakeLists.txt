cmake_minimum_required(VERSION 2.8.3)

project(xju_slam)

find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		roscpp
		roslaunch
		sensor_msgs
		tf
		tf2
		tf2_ros
		cartographer_ros_msgs
		)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS
		geometry_msgs
		nav_msgs
		roscpp
		sensor_msgs
		tf2
		tf2_ros
		cartographer_ros_msgs
		)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xju_relocation node/relocation.cpp)

add_dependencies(xju_relocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(xju_relocation ${catkin_LIBRARIES})

add_executable(rotate_map node/rotate_map.cpp)

add_dependencies(rotate_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate_map ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(rotate_map PRIVATE
                           include
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS})

install(TARGETS xju_relocation rotate_map
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

install(DIRECTORY launch param rviz map path
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

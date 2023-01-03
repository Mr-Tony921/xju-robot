cmake_minimum_required(VERSION 2.8.3)

project(xju_simu)

find_package(catkin REQUIRED COMPONENTS
		message_generation
		message_runtime
		roscpp
		roslaunch
		)

add_message_files(FILES
		xjupose.msg
		fusion_analysis.msg
		)

generate_messages()

catkin_package(CATKIN_DEPENDS message_generation message_runtime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xju_bridge_node node/xju_bridge_node.cpp src/xju_bridge.cpp)
add_dependencies(xju_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xju_bridge_node ${catkin_LIBRARIES})

install(TARGETS xju_bridge_node
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
install(PROGRAMS
		teleop/teleop_key
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
install(DIRECTORY include
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		)
install(DIRECTORY launch rviz urdf worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

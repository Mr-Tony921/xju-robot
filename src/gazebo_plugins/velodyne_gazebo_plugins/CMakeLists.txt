cmake_minimum_required(VERSION 3.0.2)
project(velodyne_gazebo_plugins)
set(CMAKE_CXX_STANDARD 11)
set(Protobuf_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../gazebo_protoc/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../gazebo_protoc/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/../gazebo_protoc/bin"
)

list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")
message(STATUS "=== CMAKE_PREFIX_PATH is: ${CMAKE_PREFIX_PATH}")

set(protobuf_MODULE_COMPATIBLE ON CACHE BOOL "")
find_package(Protobuf REQUIRED)
#message(STATUS "=== Protobuf_PROTOC_EXECUTABLE: ${Protobuf_PROTOC_EXECUTABLE}")

# INCLUDE_DIRECTORIES(/work/share/gazebo_protoc/include/)
# LINK_DIRECTORIES(/work/share/gazebo_protoc/lib/)
if(CMAKE_CROSSCOMPILING)
  message("DON'T NEED GAZEBO")
else()
  find_package(gazebo_ros REQUIRED)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

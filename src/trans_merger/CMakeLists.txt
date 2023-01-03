cmake_minimum_required(VERSION 2.8.3)
project(trans_merger)
add_compile_options(-std=c++17)
## Find depends
find_package(catkin REQUIRED COMPONENTS
             tf
             roscpp
             pcl_ros
             std_msgs
             sensor_msgs
             geometry_msgs
             laser_geometry)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# and https://github.com/PointCloudLibrary/pcl/issues/3680
# when upgrade pcl to 1.12, in which version this has been fixed,
# we can remove the following set and unset lines.
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
find_package(PCL REQUIRED)
unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
remove_definitions(-DDISABLE_LIBUSB-1.0)
###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
# build exec sensor_preprocessor
add_executable(${PROJECT_NAME} node/trans_merger_node.cpp src/trans_merger.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE
                           ${PCL_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
                           ${EIGEN_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${PROJECT_SOURCE_DIR}/include)
# end build exec sensor_preprocessor

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

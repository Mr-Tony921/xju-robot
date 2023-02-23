cmake_minimum_required(VERSION 2.8.3)
project(xju_perception)
add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS roscpp
        pcl_ros
        tf
        sensor_msgs
        pcl_conversions
        message_generation)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS " -O2 -g -Wall ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(xju_perception src/main.cpp src/ukf.cpp)

target_link_libraries(xju_perception
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_dependencies(xju_perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

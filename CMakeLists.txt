## Minimum Requires CMake Version
cmake_minimum_required(VERSION 2.8.3)
project(project_x_ecobot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add Project CMake Modules to Path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rosbag rostest cv_bridge image_transport tf)
find_package(turtlebot_gazebo REQUIRED)
find_package(OpenCV REQUIRED)

## Catkin Packages
catkin_package()

## Specify additional locations of header files
include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a C++ Executable
add_executable(project_x_ecobot src/VelocityGenerator.cpp src/Point.cpp src/DebrisCollection.cpp)

## Specify Target Libraries
target_link_libraries(project_x_ecobot ${OpenCV_LIBS} ${catkin_LIBRARIES})

## Add GTest Package for ROSTest
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(tests_project_x_ecobot test/PointTest.cpp test/DebrisCollectionTest.cpp test/VelocityGeneratorTest.cpp)
	target_link_libraries(project_x_ecobot ${catkin_LIBRARIES})
	add_dependencies(project_x_ecobot ${OpenCV_INCLUDE_DIRS} ${catkin_EXPORTED_TARGETS})
endif()

## Minimum Requires CMake Version
cmake_minimum_required(VERSION 2.8.3)
project(project_x_ecobot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add Project CMake Modules to Path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs rosbag rostest cv_bridge image_transport tf)
find_package(OpenCV REQUIRED)

## Catkin Packages
catkin_package()

## Specify additional locations of header files
include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a C++ Executable
add_executable(MainNode src/main.cpp src/ImageAnalysis.cpp src/Point.cpp src/StateMachine.cpp src/GreedyAlg.cpp src/LowXAlg.cpp)

option(COVERAGE "Generate Coverage Data" ON)

## Specify Target Libraries
target_link_libraries(MainNode ${OpenCV_LIBS} ${catkin_LIBRARIES})

## Add coverall
if (COVERAGE)
    include(CodeCoverage)
    setup_target_for_coverage(code_coverage ../devel/lib/project_x_ecobot/allTests coverage)
    set(COVERAGE_SRCS app/main.cpp include/Point.hpp include/ImageAnalysis.hpp include/StateMachine.hpp include/IPlanningAlg.hpp include/GreedyAlg.hpp include/LowXAlg.hpp)

    SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS " --coverage -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
endif()

## Add GTest Package for ROSTest
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(allTests test/NodeTest.launch test/main.cpp test/PointTest.cpp test/LowXAlgTest.cpp test/GreedyAlgTest.cpp test/StateMachineTest.cpp test/ImageAnalysisTest.cpp src/Point.cpp src/StateMachine.cpp src/ImageAnalysis.cpp src/GreedyAlg.cpp src/LowXAlg.cpp)
	target_link_libraries(allTests ${OpenCV_LIBS} ${catkin_LIBRARIES})
	add_dependencies(allTests ${catkin_EXPORTED_TARGETS})
endif()


/** @file node_test.cpp
* @brief 2nd level tests for our Ecobot node
*
* Modified from the example
Ryan Bates, ROS.org

Apache Project-X
Copyright 2019 The Apache Software Foundation

This product includes software developed at
The Apache Software Foundation (http://www.apache.org/).

*/

#include <ros/ros.h>
#include <gtest/gtest.h>

/**
* @brief Dummy test
* @param TestSuite This is part of our test suite
* @param dummy_test Dummy test
*/
TEST(TestSuite, dummy_test) {

}


/**
* @brief Runs the test suite
* @param argc passthrough arg for ros_init
* @param argv passthrough arg for ros_init
*/
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_debriscollection");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

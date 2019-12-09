 /*
  *
  * Copyright 2019 Lydia Zoghbi and Ryan Bates.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  */
/**
 *  @file       StateMachineTest.cpp
 *  @author     Ryan Bates and Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Level 2 and Level 1 unit tests for ImageAnalysis class
 *
 */

#include <math.h>
#include <gtest/gtest.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>

#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <iostream>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"
#include "LowXAlg.hpp"
#include "GreedyAlg.hpp"
#include "StateMachine.hpp"
#include "IPlanningAlg.hpp"
#include "ImageAnalysis.hpp"

/**
 *  @brief      Test function for StateMachine class to turn left
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, SetLeftt) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Define geometry velocity messages
	geometry_msgs::Twist velocity;

	velocity.linear.y = 0.9;
	velocity.linear.z = 0.9;
	velocity.angular.x = 0.9;
	velocity.angular.y = 0.9;
	velocity.linear.x = 0.9;
	velocity.angular.z = 0.9;

	// Send command to turn left
	velocity = stateMachine.turnLeft(velocity);

	// Expected output velocities
	EXPECT_EQ(0.0, velocity.linear.y);
	EXPECT_EQ(0.0, velocity.linear.z);
	EXPECT_EQ(0.0, velocity.angular.x);
	EXPECT_EQ(0.0, velocity.angular.y);
	EXPECT_EQ(0.0, velocity.linear.x);
	EXPECT_EQ(0.1, velocity.angular.z);
}

/**
 *  @brief      Test function for StateMachine class to turn Right
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, SetRight) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Define geometry velocity messages
	geometry_msgs::Twist velocity;

	velocity.linear.y = 0.9;
	velocity.linear.z = 0.9;
	velocity.angular.x = 0.9;
	velocity.angular.y = 0.9;
	velocity.linear.x = 0.9;
	velocity.angular.z = 0.9;

	// Send command to turn right
	velocity = stateMachine.turnRight(velocity);

	// Expected output velocities
	EXPECT_EQ(0.0, velocity.linear.y);
	EXPECT_EQ(0.0, velocity.linear.z);
	EXPECT_EQ(0.0, velocity.angular.x);
	EXPECT_EQ(0.0, velocity.angular.y);
	EXPECT_EQ(0.0, velocity.linear.x);
	EXPECT_EQ(-0.1, velocity.angular.z);
}

/**
 *  @brief      Test function for StateMachine class to stop
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, SetStop) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Define geometry velocity messages
	geometry_msgs::Twist velocity;

	velocity.linear.y = 0.9;
	velocity.linear.z = 0.9;
	velocity.angular.x = 0.9;
	velocity.angular.y = 0.9;
	velocity.linear.x = 0.9;
	velocity.angular.z = 0.9;

	// Send command to stop
	velocity = stateMachine.stop(velocity);

	// Expected output velocities
	EXPECT_EQ(0.0, velocity.linear.y);
	EXPECT_EQ(0.0, velocity.linear.z);
	EXPECT_EQ(0.0, velocity.angular.x);
	EXPECT_EQ(0.0, velocity.angular.y);
	EXPECT_EQ(0.0, velocity.linear.x);
	EXPECT_EQ(0.0, velocity.angular.z);
}

/**
 *  @brief      Test function for StateMachine class to move forward straight 
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, SetStraight) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Define geometry velocity messages
	geometry_msgs::Twist velocity;

	velocity.linear.y = 0.9;
	velocity.linear.z = 0.9;
	velocity.angular.x = 0.9;
	velocity.angular.y = 0.9;
	velocity.linear.x = 0.9;
	velocity.angular.z = 0.9;

	// Send command to move forward
	velocity = stateMachine.moveStraight(velocity);

	// Expected output velocities
	EXPECT_EQ(0.0, velocity.linear.y);
	EXPECT_EQ(0.0, velocity.linear.z);
	EXPECT_EQ(0.0, velocity.angular.x);
	EXPECT_EQ(0.0, velocity.angular.y);
	EXPECT_EQ(0.2, velocity.linear.x);
	EXPECT_EQ(0.0, velocity.angular.z);
}

/**
 *  @brief      Test function for StateMachine class for obtaining angles
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, VerifyGiantAngle) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Computed angle normalized between 0 and 2PI
	EXPECT_NEAR((2.0*M_PI), stateMachine.verifyAngle(4.0*M_PI), 0.01);
}

/**
 *  @brief      Test function for StateMachine class for casting negative angles
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, VerifyNegativeAngle) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;
	// Verify that negative angle is cast between 0 and 2PI
	EXPECT_NEAR((2.0*M_PI) - 1.0, stateMachine.verifyAngle(-1.0), 0.01);
}

/**
 *  @brief      Level 2 Test function for StateMachine image callback
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, ImageRGBCallback) {

	// Create constructor for StateMachine class
	StateMachine stateMachine;

	cv::Mat returnedImage;

	// Create nodehandle
	ros::NodeHandle nh;

	// Create fake publisher simulating ROS publishing
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);
	cv::Mat image = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/colored.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	// Publish image
	pub.publish(msg);

	// Run ROS loop and obtain subscribed image
	for (auto i=0; i<5; i++) {
		returnedImage = stateMachine.getImage();
		ros::spinOnce();
	}

	// Find location of detected debris
	ImageAnalysis imageAnalysis;
	imageAnalysis.detectDebris(returnedImage);
	Point location = imageAnalysis.getDebrisImageLocation();

	// Compare output to expected one
	EXPECT_NEAR(320, location.getX(), 10);
	EXPECT_NEAR(310, location.getY(), 10);
}

/**
 *  @brief      Level 2 Test function for StateMachine odometry callback
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, OdometryCallback) {

	double x;
	double y;
	double yaw;

	// Constructor for state machine
	StateMachine stateMachine;

	// Create node handle
	ros::NodeHandle nh;

	// Create publisher for sending odometry messages
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = 0.5;
	odom.pose.pose.position.y = 1.5;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
	odom.pose.pose.orientation = odom_quat;

	// Publish odometry
	odom_pub.publish(odom);

	// Check that machine is running properly
	EXPECT_TRUE(stateMachine.pickupDebris(StateMachine::State::turnTowardsBin, StateMachine::State::approachDebris, 0.46));

	// Run ROS loop
	for (auto i=0; i<5; i++) {
		x = stateMachine.getRobotXPos();
		y = stateMachine.getRobotYPos();
		yaw = stateMachine.getRobotYaw();
		ros::spinOnce();
	}

	// Compare output to expected one
	EXPECT_EQ(0.5, x);
	EXPECT_EQ(1.5, y);
	EXPECT_EQ(0.0, yaw);

}

/**
 *  @brief      Level 2 Test function for StateMachine camera depth callback
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, DepthCallback) {

	// Constructor for StateMachine class
	StateMachine stateMachine;

	double depth;
	bool valueReturned = false;

	// Initialize node handle
	ros::NodeHandle nh;

	// Create publisher
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/depth/image_raw", 1);
	cv::Mat image = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/depth.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32fc1", image).toImageMsg();
	// Publish depth
	pub.publish(msg);

	// Create publisher for image (because depth callback uses implicitly image callback)
	image_transport::Publisher pub2 = it.advertise("/camera/rgb/image_raw", 1);
	cv::Mat image2 = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/colored.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

	// Publish depth
	pub.publish(msg2);

	// Run ROS loop
	for (auto i=0; i<5; i++) {
		depth = stateMachine.getDepth();
		ros::spinOnce();
	}

	if ( !(depth == -1) ) {
		valueReturned = true;
	}

	// Check if output is received
	EXPECT_TRUE(valueReturned);
}

/**
 *  @brief      Level 2 Test function for StateMachine camera deep depth callback
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */
TEST(StateMachine, DeepDepthCallback) {

	// Constructor for StateMachine
	StateMachine stateMachine;

	double depth;
	bool valueReturned = false;

	// Create node handle
	ros::NodeHandle nh;

	// Create publisher for depth topic
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/depth/image_raw", 1);
	cv::Mat image = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/depth.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32fc1", image).toImageMsg();

	// Publish depth information
	pub.publish(msg);

	// Publish image to image callback
	image_transport::Publisher pub2 = it.advertise("/camera/rgb/image_raw", 1);
	cv::Mat image2 = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/colored.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

	// Publish image
	pub.publish(msg2);

	// Run ROS loop
	for (auto i=0; i<5; i++) {
		depth = stateMachine.getRawDepth();
		ros::spinOnce();
	}

	// Check returned depth
	if ( !(depth == - 1) ) {
		valueReturned = true;
	}

	// Check if output is received
	EXPECT_TRUE(valueReturned);
}

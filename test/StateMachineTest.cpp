/**
 *  @file       DebrisCollectionTest.cpp
 *  @author     Ryan Bates
 *  @copyright  Copyright Project-X 2019
 *  @date       11/23/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
Apache Project-X
Copyright 2019 The Apache Software Foundation

This product includes software developed at
The Apache Software Foundation (http://www.apache.org/).

*/

#include <gtest/gtest.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"
#include "ImageAnalysis.hpp"
#include "StateMachine.hpp"


TEST(StateMachine, ImageRGBCallback) {

	StateMachine stateMachine;
	cv::Mat returnedImage;
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);

	cv::Mat image = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/red_1.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	pub.publish(msg);

	for (int i=0; i<5; i++) {
		returnedImage = stateMachine.getImage();
		ros::spinOnce();
	}

	ImageAnalysis imageAnalysis;
	imageAnalysis.detectDebris(returnedImage);
	Point location = imageAnalysis.getDebrisImageLocation();

	EXPECT_NEAR(170, location.getX(), 10);
	EXPECT_NEAR(235, location.getY(), 10);
}



TEST(StateMachine, OdometryCallback) {

	double x;
	double y;
	double yaw;

	StateMachine stateMachine;
	ros::NodeHandle nh;

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

	nav_msgs::Odometry odom;

	odom.pose.pose.position.x = 0.5;
	odom.pose.pose.position.y = 1.5;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
	odom.pose.pose.orientation = odom_quat;

	odom_pub.publish(odom);

	EXPECT_TRUE(stateMachine.pickupDebris(2, 1, 0.46));

	for (int i=0; i<5; i++) {
		x = stateMachine.getRobotXPos();
		y = stateMachine.getRobotYPos();
		yaw = stateMachine.getRobotYaw();
		ros::spinOnce();
	}
	

	EXPECT_EQ(0.5, x);
	EXPECT_EQ(1.5, y);
	EXPECT_EQ(0.0, yaw);
}

TEST(StateMachine, DepthCallback) {
 
	StateMachine stateMachine;
	double depth;
	bool valueReturned = false;
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/depth/image_raw", 1);

	cv::Mat image = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/depth.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32fc1", image).toImageMsg();

	pub.publish(msg);


	image_transport::Publisher pub2 = it.advertise("/camera/rgb/image_raw", 1);

	cv::Mat image2 = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/colored.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

	pub.publish(msg2);

	for (int i=0; i<5; i++) {
		depth = stateMachine.getDepth();
		ros::spinOnce();
	}

	if (! (depth == -1)) {
		valueReturned = true;
	}
		
	EXPECT_TRUE(valueReturned);
}

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
 *  @file       DebrisCollection.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of DebrisCollection class
 *
 */

#include <vector>
#include "ros/ros.h"
#include "DebrisCollection.hpp"
#include <Point.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Twist.h"

// Class constructor
DebrisCollection::DebrisCollection() {

	// Do publishing and subscribing
	// ros::NodeHandle nh;
	sub = nh.subscribe("/camera/rgb/image_raw", 500, &DebrisCollection::imageRGBCallback, this);
	odomSub = nh.subscribe("/odom", 500, &DebrisCollection::odometryCallback, this);
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 500);

	geometry_msgs::Twist velocity;
	velocity.linear.x = 1.0;
	velocity.angular.z = 0;

	ROS_INFO_STREAM("Subscriptions made.");
	ros::Rate loop_rate(1);

	while (ros::ok()) {
		// pub.publish(velocity);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

// Reading image from the robot's camera
void DebrisCollection::imageRGBCallback(const sensor_msgs::ImageConstPtr& message) {
	ROS_INFO_STREAM("Entered image callback");
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		// TODO: error processing
		ROS_INFO_STREAM("Error");
	}

	DebrisCollection::filter(cv_ptr->image);
	// cv::imshow("Window", cv_ptr->image);
	ROS_INFO_STREAM("Image should be displayed");
	// cv::waitKey(1);
  
	// if (we want image) {
	//	 detectDebris(image);
	//}
}

// Reading robot's odometry measurements
void DebrisCollection::odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
	// message->pose.pose.
	ROS_INFO_STREAM("Bot is at " << message->pose.pose.position.x << ", " << message->pose.pose.position.y);
}


// Reading depth information from the robot's camera
std::vector<double> DebrisCollection::DepthCallback(const sensor_msgs::Image &) {}

// Applying HSV filter to detect debrid
cv::Mat DebrisCollection::filter(cv::Mat rawImage) {
 
	cv::Mat hsvImage, thresholdImage;
	// Convert image from RGB to HSV	
	cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);
	// Apply Hue, Saturation and Value thresholds on HSV image
	cv::inRange(hsvImage, cv::Scalar(0, 33, 50), cv::Scalar(6, 255, 153), thresholdImage);
	cv::imshow("FilteredImage", thresholdImage);
	ROS_INFO_STREAM("Image should be displayed");
	cv::waitKey(1);

	return thresholdImage;
}

// Function for detecting debris after applying filter
Point DebrisCollection::detectDebris(cv::Mat filteredImage) {}

// Function for concatenating debris information if detected
void DebrisCollection::addDebris(Point detectedDebris) {}

// Function for removing debris from list after it is scooped
void DebrisCollection::removeDebris() {}

// Sorting the debris by closest to bin 
std::vector<Point> DebrisCollection::sortDebrisLocation(std::vector<Point> * debrisLocations) {}


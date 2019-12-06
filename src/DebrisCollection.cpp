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
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/05/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of DebrisCollection class
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"
#include "DebrisCollection.hpp"

// Class constructor for debris collection
DebrisCollection::DebrisCollection() {

	// Subscribe to RGB images
	sub = nh.subscribe("/camera/rgb/image_raw", 500, &DebrisCollection::imageRGBCallback, this);

	// Subscribe to depth information
	image_transport::ImageTransport it(nh);
	depthSub = it.subscribe("/camera/depth/image_raw", 5, &DebrisCollection::DepthCallback, this);

	// Subscribe to odometry readings
	odomSub = nh.subscribe("/odom", 500, &DebrisCollection::odometryCallback, this);

	// Publish velocities when needed
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 500);




	ROS_INFO_STREAM("Subscriptions made successfully");
	ros::Rate loop_rate(1);

	DebrisCollection::pickupDebris();

}

// Callback function for obtaining RGB images
void DebrisCollection::imageRGBCallback(const sensor_msgs::ImageConstPtr& message) {

	// ROS_INFO_STREAM("Entered image callback");
	// Create image pointer
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		// Report error
		ROS_INFO_STREAM("Error");
	}

	// Call filtering function to start analyzing possiblity of debris existence
	DebrisCollection::filter(cv_ptr->image);
	cv::imshow("Window", cv_ptr->image);
	// ROS_INFO_STREAM("Image should be displayed");
	cv::waitKey(1);
}

// Callback function for obtaining robot's odometry measurements
void DebrisCollection::odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
	message->pose.pose;	
	// ROS_INFO_STREAM("Bot is at " << message->pose.pose.position.x << ", " << message->pose.pose.position.y);
}


// Callback function for obtaining depth information
void DebrisCollection::DepthCallback(const sensor_msgs::ImageConstPtr& depthMessage) {

	// Take image from setterm and call detection function for finding debris centroid


	// Function for obtaining depth without dealing with PCL library

	//double depth = DebrisCollection::ReadDepthData(currentDebrisLocation.getX(), currentDebrisLocation.getY(), depthMessage);
	//currentDebrisLocation = Point(0.0, depth);
	//ROS_INFO_STREAM("Depth of debris:" << depth);	
}

double DebrisCollection::getDepth() {

	return depth;
}


void DebrisCollection::pickupDebris() {

		ROS_INFO_STREAM("Entered pickupDebris");

		geometry_msgs::Twist velocity;
	ros::Rate rate(10);
	while (ros::ok()) {
		//ros::spinOnce();
		//loop_rate.sleep();
	
		ROS_INFO_STREAM("The moments are outputting "<<imageDebrisLocation.getX()<<" and "<<imageDebrisLocation.getY());


		if ((imageDebrisLocation.getX() > 310) && (imageDebrisLocation.getX() < 330)) {

			//if (imageDebrisLocation.getY() > proximityThreshold) {
				//we have it, go throw it away
			
			//} else { //object is not close enough, we need to go straight towards it
				velocity.linear.x = 0.2;
				velocity.linear.y = 0.0;
				velocity.linear.z = 0.0;
				velocity.angular.x = 0.0;
				velocity.angular.y = 0.0;
				velocity.angular.z = 0.0;

				pub.publish(velocity);

			//}

		} else {

			velocity.linear.x = 0.0;
			velocity.linear.y = 0.0;
			velocity.linear.z = 0.0;
			velocity.angular.x = 0.0;
			velocity.angular.y = 0.0;
			velocity.angular.z = 0.1;

			pub.publish(velocity);

		}
	ros::spinOnce();
	rate.sleep();
	}

}



// Applying HSV filter to detect debrid
cv::Mat DebrisCollection::filter(cv::Mat rawImage) {
 
	cv::Mat hsvImage, thresholdImage;

	// Convert image from RGB to HSV	
	cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);

	// Apply Hue, Saturation and Value thresholds on HSV image
	cv::inRange(hsvImage, cv::Scalar(0, 33, 50), cv::Scalar(6, 255, 153), thresholdImage);

	cv::imshow("FilteredImage", thresholdImage);
	// ROS_INFO_STREAM("Image should be displayed");
	cv::waitKey(1);

	// Point randomName = DebrisCollection::detectDebris(thresholdImage);
	DebrisCollection::detectDebris(thresholdImage);
	// ROS_INFO_STREAM("Reading :"<<randomName.getX()<<" and "<<randomName.getY());
	return thresholdImage;
}

// Function for setting image and storing it in black and white
void DebrisCollection::setImage(cv::Mat image) {
	lastSnapshot = image;
}

// Function for getting image back from setter
cv::Mat DebrisCollection::getImage() {
	return lastSnapshot;
}

// Function for detecting debris after applying filter
void DebrisCollection::detectDebris(cv::Mat filteredImage) {

	//ROS_INFO_STREAM("Entered detectDebris");

	// Apply moments function to obtain centroid of debris 
	cv::Moments moment = moments(filteredImage, true);
	cv::Point cvPoint(moment.m10/moment.m00, moment.m01/moment.m00);
	if (cv::countNonZero(filteredImage) < 1) {
		imageDebrisLocation = Point(-1.0, -1.0);
//		Point p(cvPoint.x, cvPoint.y);
	}
	else {
	imageDebrisLocation = Point(cvPoint.x, cvPoint.y);
	}
			// ROS_INFO_STREAM(imageDebrisLocation.getX()<<" : "<<imageDebrisLocation.getY());
}

// Function for concatenating debris information if detected
void DebrisCollection::addDebris(Point detectedDebris) {
	debrisLocation.push_back(detectedDebris);
}

// Function for removing debris from list after it is scooped
void DebrisCollection::removeDebris() {
	debrisLocation.pop_back();
}

// Sorting the debris by closest to bin 
std::vector<Point> DebrisCollection::sortDebrisLocation(std::vector<Point> * debrisLocations) {}

// Obtain depth data without using PCL Library, see link below
// (https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/) 
double DebrisCollection::ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image) {

	typedef union U_FloatParse {
		float float_data;
		unsigned char byte_data[4];
	} U_FloatConvert;

	// If position is invalid return error
	if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
		return -1;

	int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));

	// If data is 4 byte floats (rectified depth image)
	if ((depth_image->step/depth_image->width) == 4) {
		U_FloatConvert depth_data;
		int i, endian_check = 1;
		// If big endian
		if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
		((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
			for (i = 0; i < 4; i++)
				depth_data.byte_data[i] = depth_image->data[index + i];
			// Make sure data is valid (check if NaN)
			if (depth_data.float_data == depth_data.float_data)
				return double(depth_data.float_data);
		return -1;  // If depth data invalid
		}

		// Else, one little endian, one big endian
		for (i = 0; i < 4; i++) 
			depth_data.byte_data[i] = depth_image->data[3 + index - i];
		// Make sure data is valid (check if NaN)
		if (depth_data.float_data == depth_data.float_data)
			return double(depth_data.float_data);
		return -1;  // If depth data invalid
	}

	// Otherwise, data is 2 byte integers (raw depth image)
	double temp_val;

	// If big endian
	if (depth_image->is_bigendian)
		temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];

	// If little endian
	else
		temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);

	// Make sure data is valid (check if NaN)
	if (temp_val == temp_val)
		return temp_val;
	return -1;  // If depth data invalid
}


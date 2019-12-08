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
 *  @file       StateMachine.cpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/05/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of StateMachine class
 *
 */

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

// Class constructor for debris collection
StateMachine::StateMachine(bool startImmediately) {

	x = 0;
	y = 0;

	// Subscribe to RGB images
	imgSub = nh.subscribe("/camera/rgb/image_raw", 500, &StateMachine::imageRGBCallback, this);

	// Subscribe to depth information
	
//it = image_transport::ImageTransport(nh);

	it = new image_transport::ImageTransport(nh);
	depthSub = it->subscribe("/camera/depth/image_raw", 5, &StateMachine::depthCallback, this);

	// Subscribe to odometry readings
	odomSub = nh.subscribe("/odom", 500, &StateMachine::odometryCallback, this);

	// Subscribe to an advertiser to publish velocities when needed
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 500);

	ROS_INFO_STREAM("Subscriptions made successfully");

	// Call function for running turtlebot to pick up the debris
	//StateMachine::pickupDebris();

	if (startImmediately) {
		StateMachine::pickupDebris();
	}
}

// Callback function for obtaining RGB images
void StateMachine::imageRGBCallback(const sensor_msgs::ImageConstPtr& message) {

	// Create image pointer
	cv_bridge::CvImagePtr cv_ptr;

	// Get image, if an error occurs, reports it in the catch
	try {
		cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_INFO_STREAM("Error");
	}
	
	// Call filtering function to start analyzing possiblity of debris existence
	//lastSnapshot = cv_ptr->image;
	//cv::imwrite("colored.png", cv_ptr->image);
	lastSnapshot = imageAnalysis.filter(cv_ptr->image);

	// Show image, uncomment if needed
	//cv::imshow("Window", cv_ptr->image);
	//cv::waitKey(1);
}

// Callback function for obtaining robot's odometry measurements
void StateMachine::odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {

	// Store x and y position of robot in world frame
	x = message->pose.pose.position.x;
	y = message->pose.pose.position.y;

	// Convert from Quaternions to Roll-Pitch-Yaw using tf library
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	// Obtain robot's orientation in Radians in world frame
	orientation = yaw;
}

// Callback function for obtaining depth information without using PCL library
void StateMachine::depthCallback(const sensor_msgs::ImageConstPtr& depthMessage) {
	
	// Create image pointer
	cv_bridge::CvImagePtr cv_ptr;

	// Get image, if an error occurs, reports it in the catch
	try {
		cv_ptr = cv_bridge::toCvCopy(depthMessage, sensor_msgs::image_encodings::TYPE_32FC1);
	} catch (cv_bridge::Exception& e) {
		ROS_INFO_STREAM("Error");
	}
	
	// Call filtering function to start analyzing possiblity of debris existence
	//lastSnapshot = cv_ptr->image;
	// cv::imshow("Window", cv_ptr->image);
	//cv::imwrite("depth.png", cv_ptr->image);
	//cv::waitKey(10);

	// Obtain depth information on a single pixel in image
	depth = StateMachine::readDepthData(imageAnalysis.getDebrisImageLocation().getX(), imageAnalysis.getDebrisImageLocation().getY(), depthMessage);
	//depth = StateMachine::readDepthData(320, 310, depthMessage);
}

// Function for controlling the turlebot
// int endState = -1, int startState = 0
bool StateMachine::pickupDebris(int endState, int startState, double registeredDepth) {

	// Define robot states
	state = startState;

	// Variables for tracking robot's orientation and distance traveled
	double currentOrientation;
	double distanceTraveled;
	double currentDistance;
	//double registeredDepth;
	double angle;

	// Create velocity to publish velocities to turtlebot
	geometry_msgs::Twist velocity;
	ros::Rate rate(10);

	// Start by rotating the robot
	velocity.linear.x = 0.0;
	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.angular.z = 0.1;

	// Publish the velocity to move the robot
	pub.publish(velocity);

	while ((ros::ok()) && !(state == endState)) {

	// Variable for storing distance traveled by robot
	distanceTraveled = sqrt(x*x + y*y);

		// Switch to modify robot's states
		switch(state) {
			// Keep searching for debris
			case 0:
				// If a debris is found in the middle of an image
				if ((imageAnalysis.getDebrisImageLocation().getX() > 310) && (imageAnalysis.getDebrisImageLocation().getX() < 330)) {
					// Find depth of debris
					registeredDepth = depth;
					// Move towards debris
					velocity.linear.x = 0.2;
					velocity.angular.z = 0.0;
					// Switch to state 1
					state = 1;
				}
			break;

			// Move towards debris
			case 1:
				// If the robot gets close enough to the debris
				if (distanceTraveled >= (registeredDepth - 0.45)) {
					// Stop and rotate towards bin
					velocity.linear.x = 0;
					velocity.angular.z = 0.1;
					// Register current position and orientation
					currentOrientation = orientation;
					currentDistance = distanceTraveled;
					// Switch to state 2
					state = 2;
				}
			break;

			// Rotate towards bin
			case 2:
				// Find angle to rotate the robot
				angle = imageAnalysis.getRotationAngle(currentOrientation, currentDistance);
				// If robot turned completely to bin
				if (orientation >= angle) {
					// Move to bin
					velocity.linear.x = 0.2;
					velocity.angular.z = 0;
					// Switch to state 3
					state = 3;
				}
			break;

			// Stop before bin
			case 3:
				// If we reached close enough to the bin
				if ((x <= 0.1) && (y <= 1.4)) {
					velocity.linear.x = 0.0;
					velocity.angular.z = 0.0;
				}
			break;	
		}

	pub.publish(velocity);
	ros::spinOnce();// TODO: see if there is a better loop sleep for a specific rate here
	rate.sleep();
	}
if (state == endState) {
	return true;
} else {
	return false;
}
}

// Obtain depth data without using PCL Library, code obtained from link below
// (https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/) 
double StateMachine::readDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image) {

	typedef union U_FloatParse {
		float float_data;
		unsigned char byte_data[4];
	} U_FloatConvert;

    // If position is invalid
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
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return double(depth_data.float_data);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
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

int StateMachine::getState() {
	return state;
}

cv::Mat StateMachine::getImage() {
	return lastSnapshot;
}


//bool StateMachine::finishOnState(int stateToFinishOn) {
//return false;
//}


double StateMachine::getDepth() {return depth;}
double StateMachine::getRobotXPos() {return x;}
double StateMachine::getRobotYPos() {return y;}
double StateMachine::getRobotYaw() {return orientation;}

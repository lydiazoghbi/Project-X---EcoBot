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
StateMachine::StateMachine(bool startImmediately, bool useLowXAlgorithm) {

	x = 0;
	y = 0;

	if (useLowXAlgorithm) {
		//algorithm = new LowXAlg();
		algorithm.reset(new LowXAlg());
	} else {
		//algorithm = new GreedyAlg();
		algorithm.reset(new GreedyAlg());
	}


	// Subscribe to RGB images
	imgSub = nh.subscribe("/camera/rgb/image_raw", 500, &StateMachine::imageRGBCallback, this);

	// Subscribe to depth information
	
//it = image_transport::ImageTransport(nh);

	//it = new image_transport::ImageTransport(nh);

	it.reset(new image_transport::ImageTransport(nh));
	
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
	
	cv::Mat rawIm = cv_ptr->image;
cv::rectangle(rawIm, cv::Point(0, 0), cv::Point(210, 480), cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_8);

cv::rectangle(rawIm, cv::Point(430, 0), cv::Point(640, 480), cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_8);
	// Call filtering function to start analyzing possiblity of debris existence
	//lastSnapshot = cv_ptr->image;
	//cv::imwrite("colored.png", cv_ptr->image);
	lastSnapshot = imageAnalysis.filter(rawIm);

	// Show image, uncomment if needed
	//cv::imshow("Window", rawIm);
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
	rawDepth = StateMachine::readDepthData(imageAnalysis.getDebrisImageLocation().getX(), imageAnalysis.getDebrisImageLocation().getY(), depthMessage);
	//depth = StateMachine::readDepthData(320, 310, depthMessage);
}

// Function for controlling the turlebot
// int endState = -1, int startState = 0
bool StateMachine::pickupDebris(State endState, State startState, double registeredDepth, double fakeDepth) {




	double angleThreshold = 0.01;//0.03;//0.087;
	Point currentTarget;
	// Define robot states
	state = startState;

	// Variables for tracking robot's orientation and distance traveled
	double currentOrientation;
	double distanceTraveled;
	double currentDistance;
	//double registeredDepth;
	double angle;

	int trashCount = 0;


double targetAngle = 0.0;

				double effectiveOrientation = 0.0;


double targetAngleBin = 0.0;

				double effectiveOrientationBin = 0.0;


	bool haveJustSeenDebris = false;


double holeDistance = 0.35;
				double calculatedDistance = 0;

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
		//ROS_INFO_STREAM("State is " << state);
		// Switch to modify robot's states

if (fakeDepth < 0) {


	
depth = rawDepth;
} else {
depth = fakeDepth;
}


		switch(state) {
			// Keep searching for debris
			case initialScanForDebris:
				// If a debris is found in the middle of an image
				//ROS_INFO_STREAM("	haveJustSeenDebris is " << haveJustSeenDebris << " and trash count is " << );
				ROS_INFO_STREAM("	Trash count is " << trashCount);
				if ((imageAnalysis.getDebrisImageLocation().getX() > 310) && (imageAnalysis.getDebrisImageLocation().getX() < 330)) {

					if (!haveJustSeenDebris) {
						haveJustSeenDebris = true;
						// Find depth of debris
						registeredDepth = depth;
						// Move towards debris
						//velocity.linear.x = 0.2;
						//velocity.angular.z = 0.0;
						// Switch to state 1
						Point newTarget(registeredDepth * cos(orientation), registeredDepth * sin(orientation));
						// take depth and orientation of robot to create point and give to planning algorithm
						algorithm->push(newTarget);
						trashCount++;
						ROS_INFO_STREAM("	We saw new red - registered new target at " << newTarget.getX() << ", " << newTarget.getY() << " at depth " << depth);
					} else {
						ROS_INFO_STREAM("	Still seeing same red object.");
					}
					//state = approachDebris;
				} else {
					haveJustSeenDebris = false;
					ROS_INFO_STREAM("	No red object seen in center.");
				}


				
				// if we have turned 90 degrees
				if (orientation > 1.571) {
					currentTarget = algorithm->pop(Point(x, y));
					if ((currentTarget.getX() < 0.0) && (currentTarget.getY() < 0.0)) {
						state = done;
						ROS_INFO_STREAM("	STATE TRANSITION: no trash objects left, state is now " << done);
						velocity.linear.x = 0.0;
						velocity.angular.z = 0.0;
					} else {
						// get first object from planning algorithm
						// change state to turn towards first object
						velocity.linear.x = 0.0;
						velocity.angular.z = -0.1;
						state = turnTowardsTarget;
						ROS_INFO_STREAM("	STATE TRANSITION: turned past 90 degrees, now to state " << turnTowardsTarget);
					}
				}
			break;

			// Move towards debris
			case approachDebris:
				// If the robot gets close enough to the debris
				ROS_INFO_STREAM("	" << state << ": We are " << distanceTraveled << " out of " << registeredDepth);
				if (distanceTraveled >= (registeredDepth - 0.35)) {
					// Stop and rotate towards bin
					velocity.linear.x = 0;
					velocity.angular.z = 0.1;
					// Register current position and orientation
					currentOrientation = orientation;
					currentDistance = distanceTraveled;
					// Switch to state 2
					state = turnTowardsBin;
					ROS_INFO_STREAM("	STATE TRANSITION: close to target debris, now to state " << turnTowardsBin);
				}
			break;

			// Rotate towards bin
			case turnTowardsBin:
				// invalidate depth
				registeredDepth = 99999999.0;
				// Find angle to rotate the robot

				targetAngleBin = M_PI - atan((1.5 - y) / (0.2 + x));//targetAngleBin = atan((1.5 - y) / (-0.2 - x));
				//if ((x + 0.2) > 0.0) {
				//	targetAngleBin = -1.0 * targetAngleBin;
				//}

				effectiveOrientationBin = orientation;

				effectiveOrientationBin = verifyAngle(effectiveOrientationBin);
				targetAngleBin = verifyAngle(targetAngleBin);
				/*
				while (effectiveOrientationBin < 0) {
					effectiveOrientationBin+=2.0*M_PI;
				}

				while (effectiveOrientationBin > (2.0*M_PI)) {
					effectiveOrientationBin-=2.0*M_PI;
				}

				while (targetAngleBin < 0) {
					targetAngleBin+=2.0*M_PI;
				}

				while (targetAngleBin > (2.0*M_PI)) {
					targetAngleBin-=2.0*M_PI;
				}
				*/

ROS_INFO_STREAM("	" << state << ": We are pointing " << effectiveOrientationBin << " and want to be at " << targetAngleBin);
				//angle = imageAnalysis.getRotationAngle(currentOrientation, currentDistance);
				// If robot turned completely to bin
				//if (orientation >= angle) {
				if (((effectiveOrientationBin - targetAngleBin) * (effectiveOrientationBin - targetAngleBin)) < (angleThreshold * angleThreshold)) {
					// Move to bin
					velocity.linear.x = 0.2;
					velocity.angular.z = 0;
					// Switch to state 3
					state = driveTowardsBin;
					ROS_INFO_STREAM("	STATE TRANSITION: pointed to trash bin, now to state " << driveTowardsBin);
				}
			break;

			// Stop before bin
			case driveTowardsBin:
				// If we reached close enough to the bin
				//if ((x <= 0.2) && (y <= 1.5)) {//if ((x <= 0.1) && (y <= 1.4)) {
				holeDistance = 0.42;//0.5;//.35;
				calculatedDistance = sqrt(((0.2 + x) * (0.2 + x)) + ((1.5 - y) * (1.5 - y)));
				ROS_INFO_STREAM("	" << state << ": We are " << calculatedDistance << " out of " << holeDistance);
				if (calculatedDistance < holeDistance) {


					currentTarget = algorithm->pop(Point(x, y));
					if ((currentTarget.getX() < 0.0) && (currentTarget.getY() < 0.0)) {
						state = done;
						ROS_INFO_STREAM("	STATE TRANSITION: no trash objects left, state is now " << done);
						velocity.linear.x = 0.0;
						velocity.angular.z = 0.0;
					} else {
						velocity.linear.x = 0.0;
						velocity.angular.z = 0.1;
						state = turnTowardsTarget;
						ROS_INFO_STREAM("	STATE TRANSITION: close to trash bin, now to state " << turnTowardsTarget);
					}
				}
			break;	

			case turnTowardsTarget:
				

				targetAngle = atan((currentTarget.getY() - y) / (currentTarget.getX() - x));
				if ((x - currentTarget.getX()) > 0.0) {
					targetAngle = -1.0 * targetAngle;
				}

				effectiveOrientation = orientation;

				effectiveOrientation = verifyAngle(effectiveOrientation);
				targetAngle = verifyAngle(targetAngle);
				/*
				while (effectiveOrientation < 0) {
					effectiveOrientation+=2.0*M_PI;
				}

				while (effectiveOrientation > (2.0*M_PI)) {
					effectiveOrientation-=2.0*M_PI;
				}

				while (targetAngle < 0) {
					targetAngle+=2.0*M_PI;
				}

				while (targetAngle > (2.0*M_PI)) {
					targetAngle-=2.0*M_PI;
				}
				*/



				ROS_INFO_STREAM("	" << state << ": We are pointing " << effectiveOrientation << " and want to be at " << targetAngle);
				if (((effectiveOrientation - targetAngle) * (effectiveOrientation - targetAngle)) < (angleThreshold * angleThreshold)) {
					velocity.linear.x = 0.2;
					velocity.angular.z = 0;
					registeredDepth = depth;
					state = approachDebris;
					ROS_INFO_STREAM("	STATE TRANSITION: pointed to target, now to state " << approachDebris);
				}
				
			break;
			//case done:
			//	ROS_INFO_STREAM("	DONE!");
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
double StateMachine::readDepthData(unsigned int heightPos, unsigned int widthPos, sensor_msgs::ImageConstPtr depthImage) {

	typedef union U_FloatParse {
		float floatData;
		unsigned char byteData[4];
	} U_FloatConvert;

    // If position is invalid
    if ((heightPos >= depthImage->height) || (widthPos >= depthImage->width))
        return -1;
    int index = (heightPos*depthImage->step) + (widthPos*(depthImage->step/depthImage->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depthImage->step/depthImage->width) == 4) {
        U_FloatConvert depthData;
        int i, endianCheck = 1;
        // If big endian
        if ((depthImage->is_bigendian && (*(char*)&endianCheck != 1)) ||  // Both big endian
           ((!depthImage->is_bigendian) && (*(char*)&endianCheck == 1))) { // Both lil endian
            for (auto i = 0; i < 4; i++)
                depthData.byteData[i] = depthImage->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depthData.floatData == depthData.floatData)
                return double(depthData.floatData);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depthData.byteData[i] = depthImage->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depthData.floatData == depthData.floatData)
            return double(depthData.floatData);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int tempVal;
   // If big endian
   if (depthImage->is_bigendian)
       tempVal = (depthImage->data[index] << 8) + depthImage->data[index + 1];
   // If little endian
   else
       tempVal = depthImage->data[index] + (depthImage->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (tempVal == tempVal)
       return tempVal;
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
double StateMachine::verifyAngle(double rawAngle) {
while (rawAngle < 0) {
					rawAngle+=2.0*M_PI;
				}

				while (rawAngle > (2.0*M_PI)) {
					rawAngle-=2.0*M_PI;
				}
return rawAngle;
}

double StateMachine::getDepth() {return depth;}
double StateMachine::getRobotXPos() {return x;}
double StateMachine::getRobotYPos() {return y;}
double StateMachine::getRobotYaw() {return orientation;}

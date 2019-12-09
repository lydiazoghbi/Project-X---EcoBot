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
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of StateMachine class
 *
 */

#include <math.h>

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

// Class constructor for debris collection
StateMachine::StateMachine(bool startImmediately, bool useLowXAlgorithm) {

	// Initialize readings to 0
	x = 0;
	y = 0;

	// Choose which algorithm you want to use, greedy or sorting by distance
	if (useLowXAlgorithm) {
		algorithm.reset(new LowXAlg());
	} else {
		algorithm.reset(new GreedyAlg());
	}

	// Subscribe to RGB images
	imgSub = nh.subscribe("/camera/rgb/image_raw", 500, &StateMachine::imageRGBCallback, this);

	// Subscribe to depth information from RGB-D camera
	it.reset(new image_transport::ImageTransport(nh));
	depthSub = it->subscribe("/camera/depth/image_raw", 5, &StateMachine::depthCallback, this);

	// Subscribe to odometry readings
	odomSub = nh.subscribe("/odom", 500, &StateMachine::odometryCallback, this);

	// Subscribe to an advertiser to publish velocities when needed
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 500);

	ROS_INFO_STREAM("Subscriptions made successfully");

	// If no waiting is needed, start running code (this is added for testing purposes)
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
		ROS_ERROR_STREAM("Error");
	}

	// Extract image
	cv::Mat rawIm = cv_ptr->image;

	// Add rectangles on image's sides to block the robot from seeing multiple debris at once
	cv::rectangle(rawIm, cv::Point(0, 0), cv::Point(210, 480), cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_8);
	cv::rectangle(rawIm, cv::Point(430, 0), cv::Point(640, 480), cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_8);

	// Apply filter to image
	lastSnapshot = imageAnalysis.filter(rawIm);

	// Show image, uncomment if needed for debugging
	// cv::imshow("Window", rawIm);
	// cv::waitKey(1);
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
		ROS_ERROR_STREAM("Error");
	}

	// Obtain depth information on a single pixel in image
	rawDepth = StateMachine::readDepthData(imageAnalysis.getDebrisImageLocation().getX(), imageAnalysis.getDebrisImageLocation().getY(), depthMessage);
}

// Function for controlling the turlebot (this is the essence of the robot)
bool StateMachine::pickupDebris(State endState, State startState, double registeredDepth, double fakeDepth) {

	// Define angular threshold to stop the robot's rotation when needed
	double angleThreshold = 0.01;

	// Define target Point class for storing information about debris
	Point currentTarget;

	// Define robot states
	state = startState;

	// Variables for tracking robot's orientation and distance traveled
	double currentOrientation;
	double distanceTraveled;
	double currentDistance;

	// Number of trash debris detected
	int trashCount = 0;

	// Current and target angles of robot
	double angle;
	double targetAngle = 0.0;

	// Adjusted angles, making sure the angles are between 0 and 2PI instead of -PI and PI
	double effectiveOrientation = 0.0;
	double targetAngleBin = 0.0;
	double effectiveOrientationBin = 0.0;

	// Boolean to keep track of the debris detection to avoid counting one debris multiple times
	bool haveJustSeenDebris = false;

	// Distance to disposal bin
	double holeDistance = 0.35;

	// Distance from robot to disposal bin
	double calculatedDistance = 0;

	// Create velocity to publish velocities to turtlebot
	geometry_msgs::Twist velocity;
	ros::Rate rate(10);

	// Rotate left
	velocity = turnLeft(velocity);
	// Publish the velocity to move the robot
	pub.publish(velocity);

	// If ros is functioning properly, and the state the robot is at is not the final state (after cleaning up)
	while ((ros::ok()) && !(state == endState)) {

	// Variable for storing distance traveled by robot
	distanceTraveled = sqrt(x*x + y*y);
	ROS_DEBUG_STREAM("State is " << state);

	// If loop added to allow for Level 2 testing integration. If in testing mode we send the depth info instead of callback
	if (fakeDepth < 0) {
		depth = rawDepth;
	} else {
	depth = fakeDepth;
	}

	// Switch case to modify robot's states
	switch(state) {
		// Keep searching for debris
		case initialScanForDebris:

			// If a debris is found in the middle of an image
			if ((imageAnalysis.getDebrisImageLocation().getX() > 310) && (imageAnalysis.getDebrisImageLocation().getX() < 330)) {
				// If we have not seen a debris previously
				if (!haveJustSeenDebris) {
					haveJustSeenDebris = true;

					// Find depth of debris
					registeredDepth = depth;
					// Find position of detected debris
					Point newTarget(registeredDepth * cos(orientation), registeredDepth * sin(orientation));
					// Take depth and orientation of robot to create point and give to planning algorithm
					algorithm->push(newTarget);
					trashCount++;
					ROS_DEBUG_STREAM("We saw new red - registered new target at " << newTarget.getX() << ", " << newTarget.getY() << " at depth " << depth);
					} else {
						ROS_DEBUG_STREAM("	Still seeing same red object.");
					}
				} else {
					// If we did see a debris previously, don't count the same debris more than once
					haveJustSeenDebris = false;
					ROS_DEBUG_STREAM("No red object seen in center.");
				}

			// If we have turned 90 degrees
			if (orientation > 1.571) {
				currentTarget = algorithm->pop(Point(x, y));
				if ((currentTarget.getX() < 0.0) && (currentTarget.getY() < 0.0)) {
					state = done;
					ROS_DEBUG_STREAM("STATE TRANSITION: no trash objects left, state is now " << done);
					velocity = stop(velocity);
				} else {
					// Get first object from planning algorithm
					velocity = turnRight(velocity);
					state = turnTowardsTarget;
					ROS_DEBUG_STREAM("	STATE TRANSITION: turned past 90 degrees, now to state " << turnTowardsTarget);
				}
			}
		break;

		// Move towards debris state
		case approachDebris:
			// If the robot gets close enough to the debris
			ROS_DEBUG_STREAM("	" << state << ": We are " << distanceTraveled << " out of " << registeredDepth);
			if (distanceTraveled >= (registeredDepth - 0.35)) {

				// Stop and rotate towards bin
				velocity = turnLeft(velocity);

				// Register current position and orientation
				currentOrientation = orientation;
				currentDistance = distanceTraveled;

				// Switch to state 2
				state = turnTowardsBin;
				ROS_DEBUG_STREAM("	STATE TRANSITION: close to target debris, now to state " << turnTowardsBin);
				}
			break;

		// Rotate towards bin
		case turnTowardsBin:

			// Invalidate depth
			registeredDepth = 99999999.0;

			// Find angle to rotate the robot
			targetAngleBin = M_PI - atan((1.5 - y) / (0.2 + x));
			effectiveOrientationBin = orientation;
			effectiveOrientationBin = verifyAngle(effectiveOrientationBin);
			targetAngleBin = verifyAngle(targetAngleBin);

			ROS_DEBUG_STREAM("	" << state << ": We are pointing " << effectiveOrientationBin << " and want to be at " << targetAngleBin);

			// If we have rotated enough (square is to keep numbers positive)
			if (((effectiveOrientationBin - targetAngleBin) * (effectiveOrientationBin - targetAngleBin)) < (angleThreshold * angleThreshold)) {

				// Move to bin
				velocity = moveStraight(velocity);
				state = driveTowardsBin;
				ROS_DEBUG_STREAM("STATE TRANSITION: pointed to trash bin, now to state " << driveTowardsBin);
				}
		break;

		// Stop before bin
		case driveTowardsBin:

			// If we reached close enough to the bin
			holeDistance = 0.42;
			calculatedDistance = sqrt(((0.2 + x) * (0.2 + x)) + ((1.5 - y) * (1.5 - y)));
			ROS_DEBUG_STREAM("	" << state << ": We are " << calculatedDistance << " out of " << holeDistance);

			// If we reached close enough to the bin
			if (calculatedDistance < holeDistance) {
				currentTarget = algorithm->pop(Point(x, y));
				// If no debris are left, the robot goes to its final state
				if ((currentTarget.getX() < 0.0) && (currentTarget.getY() < 0.0)) {
					state = done;
					ROS_INFO_STREAM("STATE TRANSITION: no trash objects left, state is now " << done);
					velocity = stop(velocity);
				} else {
					// If some debris are left, the robot will go to the next one
					velocity = turnLeft(velocity);
					state = turnTowardsTarget;
					ROS_DEBUG_STREAM("STATE TRANSITION: close to trash bin, now to state " << turnTowardsTarget);
					}
				}
		break;

		// Rotate towards the debris
		case turnTowardsTarget:

			// Getting angle to rotate towards debris
			targetAngle = atan((currentTarget.getY() - y) / (currentTarget.getX() - x));
			if ((x - currentTarget.getX()) > 0.0) {
				targetAngle = -1.0 * targetAngle;
			}
			effectiveOrientation = orientation;
			effectiveOrientation = verifyAngle(effectiveOrientation);
			targetAngle = verifyAngle(targetAngle);

			ROS_DEBUG_STREAM("	" << state << ": We are pointing " << effectiveOrientation << " and want to be at " << targetAngle);

			// If we are close to the correct angle
			if (((effectiveOrientation - targetAngle) * (effectiveOrientation - targetAngle)) < (angleThreshold * angleThreshold)) {
				velocity = moveStraight(velocity);
				registeredDepth = depth;
				state = approachDebris;
				ROS_DEBUG_STREAM("STATE TRANSITION: pointed to target, now to state " << approachDebris);
				}

		break;
	}

	// Publish all computed velocities
	pub.publish(velocity);
	ros::spinOnce();
	rate.sleep();
	}

	// If we reached the final clean state, return true, otherwise false
	if (state == endState) {
		return true;
	} else {
		return false;
	}
}

// Function for stopping the robot
geometry_msgs::Twist StateMachine::stop(geometry_msgs::Twist velocity) {

	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.linear.x = 0.0;
	velocity.angular.z = 0.0;

	return velocity;
}

// Function for turning the robot to the left
geometry_msgs::Twist StateMachine::turnLeft(geometry_msgs::Twist velocity) {

	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.linear.x = 0.0;
	velocity.angular.z = 0.1;

	return velocity;
}

// Function for turning the robot to the right
geometry_msgs::Twist StateMachine::turnRight(geometry_msgs::Twist velocity) {

	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.linear.x = 0.0;
	velocity.angular.z = -0.1;

	return velocity;
}

// Function for moving the robot forward
geometry_msgs::Twist StateMachine::moveStraight(geometry_msgs::Twist velocity) {

	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.linear.x = 0.2;
	velocity.angular.z = 0.0;

	return velocity;
}

// Obtain depth data without using PCL Library, code inspired from link below
// (https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/)
double StateMachine::readDepthData(unsigned int heightPos, unsigned int widthPos, sensor_msgs::ImageConstPtr depthImage) {

	// Defined needed structure
	typedef union U_FloatParse {
		float floatData;
		unsigned char byteData[4];
	} U_FloatConvert;

	// If position is invalid
	if ((heightPos >= depthImage->height) || (widthPos >= depthImage->width))
        	return -1;
		ROS_WARN_STREAM("Position invalid on image");
    	int index = (heightPos*depthImage->step) + (widthPos*(depthImage->step/depthImage->width));
    	// If data is 4 byte floats (rectified depth image)
    	if ((depthImage->step/depthImage->width) == 4) {
     	   U_FloatConvert depthData;

	int endianCheck = 1;

        // If big endian
        if ((depthImage->is_bigendian && (*reinterpret_cast<char*>(&endianCheck) != 1)) ||  // Both big endian
           ((!depthImage->is_bigendian) && (*reinterpret_cast<char*>(&endianCheck) == 1))) { // Both lil endian
            for (auto i = 0; i < 4; i++)
                depthData.byteData[i] = depthImage->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depthData.floatData == depthData.floatData)
		return static_cast<double>(depthData.floatData);
            return -1;  // If depth data invalid
	    ROS_WARN_STREAM("No debris detected");
        }
        // Else, one little endian, one big endian
        for (auto i = 0; i < 4; i++)
            depthData.byteData[i] = depthImage->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depthData.floatData == depthData.floatData)
           return static_cast<double>(depthData.floatData);
        return -1;  // If depth data invalid
	ROS_WARN_STREAM("No debris detected");
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
	ROS_WARN_STREAM("No debris detected");
}

// Function for setting the state of the robot
int StateMachine::getState() {
	return state;
}

// Function for getting images for testing purposes
cv::Mat StateMachine::getImage() {
	return lastSnapshot;
}

// Function for veriyfing that the angle falls between 0 and 2PI
double StateMachine::verifyAngle(double rawAngle) {
	while (rawAngle < 0) {
		rawAngle+=2.0*M_PI;
	}

	while (rawAngle > (2.0*M_PI)) {
		rawAngle -= 2.0*M_PI;
	}

	return rawAngle;
}

// Function for getting the depth information from the depth callback function
double StateMachine::getRawDepth() {
	return rawDepth;
}

// Function for getting the depth information from the depth callback function (assigned differently than getRawDepth)
double StateMachine::getDepth() {
	return depth;
}

// Function for getting robot's x position
double StateMachine::getRobotXPos() {
	return x;
}

// Function for getting the robot's y position
double StateMachine::getRobotYPos() {
	return y;
}

// Function for getting the robot's orientation
double StateMachine::getRobotYaw() {
	return orientation;
}

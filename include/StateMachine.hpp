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
 *  @file       StateMachine.hpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Header file for StateMachine class
 *
 */

#ifndef INCLUDE_STATEMACHINE_HPP_
#define INCLUDE_STATEMACHINE_HPP_

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
#include "IPlanningAlg.hpp"
#include "ImageAnalysis.hpp"

class StateMachine {

	public:

		/**
 		*  @brief      Enum state elements
 		*  @param      None
 		*  @return     Enum
 		*/
		enum State: int {
			placeholderState = -1,
			initialScanForDebris = 0,
			approachDebris = 1,
			turnTowardsBin = 2,
			driveTowardsBin = 3,
			turnTowardsTarget = 4,
			done = 5
		};

		/**
 		*  @brief      Constructor for StateMachine
 		*  @param      Boolean for starting immediately or waiting
 		*  @param      Boolean for choosing which algorithm to use
 		*  @return     None
 		*/
		explicit StateMachine(bool startImmediately = false, bool useLowXAlgorithm = false);

		/**
 		*  @brief      Callback function to obtain the images
 		*  @param      A ros message which is the RGB image
 		*  @return     An RGB image to the subscriber, nothing explicit from function
 		*/
		void imageRGBCallback(const sensor_msgs::ImageConstPtr& message);

		/**
 		*  @brief      Callback function to obtain the odometry
 		*  @param      A ros message which is odometry message
 		*  @return     Turtlebot position and orientation to the subscriber, nothing explicit from 
		*              function
 		*/
		void odometryCallback(const nav_msgs::Odometry::ConstPtr& message);

		/**
 		*  @brief      Callback function to obtain depth information from the images
 		*  @param      A ros message which is the depth reading
 		*  @return     Depth of selected point in image, nothing explicit
 		*/
		void depthCallback(const sensor_msgs::ImageConstPtr& depthMessage);

		/**
 		*  @brief      Function for obtaining depth information at certain image pixel position
 		*  @param      Image pixel height
 		*  @param      Image pixel wdith
 		*  @param      Depth image from RGB-D camera
 		*  @return     Depth of specified pixel on image
 		*/
		double readDepthData(unsigned int heightPos, unsigned int widthPos, sensor_msgs::ImageConstPtr depthImage);

		/**
 		*  @brief      Function for running the main ros loop
 		*  @param      State enum for a placeholderState
 		*  @param      State enum for the end state
 		*  @param      Depth parameter, added for testing purposes
 		*  @param      Depth parameter, added for testing purposes
 		*  @return     Boolean true on successful loop termination
 		*/
		bool pickupDebris(State endState = placeholderState, State startState = initialScanForDebris, double registeredDepth = 0.0, double fakeDepth = -1.0);

		/**
 		*  @brief      Getter function for taking depth as input
 		*  @param      None
 		*  @return     Double depth
 		*/
		double getRawDepth();

		/**
 		*  @brief      Getter function for reading robot's state
 		*  @param      None
 		*  @return     Integer robot's state
 		*/
		int getState();

		/**
 		*  @brief      Getter function for taking in the image
 		*  @param      None
 		*  @return     Returned image
 		*/
		cv::Mat getImage();

		/**
 		*  @brief      Getter function for obtaining robot's x position
 		*  @param      None
 		*  @return     Double x position
 		*/
		double getRobotXPos();

		/**
 		*  @brief      Getter function for obtaining robot's y position
 		*  @param      None
 		*  @return     Double y position
 		*/
		double getRobotYPos();

		/**
 		*  @brief      Getter function for obtaining robot's yaw angle (orientation)
 		*  @param      None
 		*  @return     Orientattion
 		*/
		double getRobotYaw();

		/**
 		*  @brief      Getter function for obtaining depth read from camera
 		*  @param      None
 		*  @return     Double depth value
 		*/
		double getDepth();

		/**
 		*  @brief      Function for casting angle between 0 and 2PI
 		*  @param      An angle in radians
 		*  @return     An angle in radians between 0 and 2PI
 		*/
		double verifyAngle(double rawAngle);

		/**
 		*  @brief      Function to stop robot's motion
 		*  @param      Geometry message velocity
 		*  @return     Geometry message velocity
 		*/
		geometry_msgs::Twist stop(geometry_msgs::Twist velocity);

		/**
 		*  @brief      Function to move the robot forward (straight)
 		*  @param      Geometry message velocity
 		*  @return     Geometry message velocity
 		*/
		geometry_msgs::Twist moveStraight(geometry_msgs::Twist velocity);

		/**
 		*  @brief      Function to turn the robot to the right
 		*  @param      Geometry message velocity
 		*  @return     Geometry message velocity
 		*/
		geometry_msgs::Twist turnRight(geometry_msgs::Twist velocity);

		/**
 		*  @brief      Function to turn the robot to the left
 		*  @param      Geometry message velocity
 		*  @return     Geometry message velocity
 		*/
		geometry_msgs::Twist turnLeft(geometry_msgs::Twist velocity);

	private:
		// Virtual functions
		std::unique_ptr<IPlanningAlg> algorithm;

		// ImageAnalysis object
		ImageAnalysis imageAnalysis;

		// Storage Point for debris position in pixels
		Point imageDebrisLocation;

		// State object
		State state;

		// Orientation of robot
		double orientation;

		// Depth of object
		double depth;

		// Depth of object for timing purposes
		double rawDepth;

		// (x,y) positions of robot
		double x;
		double y;

		// Image storage
		cv::Mat lastSnapshot;

		// Creating a node handle
		ros::NodeHandle nh;

		// Defining a node publisher
		ros::Publisher pub;

		// Defining node subscriber for RGB image callback
		ros::Subscriber imgSub;

		// Defining node subscriber for odometry callback
                ros::Subscriber odomSub;

		// Subscriber elements for depth callback
		image_transport::Subscriber depthSub;
		std::unique_ptr<image_transport::ImageTransport> it;
};

#endif  // INCLUDE_STATEMACHINE_HPP_

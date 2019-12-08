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
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/05/2019
 *  @version    1.0
 *
 *  @brief      Header file for StateMachine class
 *
 */

#ifndef INCLUDE_STATEMACHINE_HPP_
#define INCLUDE_STATEMACHINE_HPP_

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
#include "ImageAnalysis.hpp"

/**
 *  @brief      Elements and members of StateMachine class
 */

class StateMachine {

	private:
		ImageAnalysis imageAnalysis;

		// Storage Point for debris position in pixels
		Point imageDebrisLocation;
		
		int state;

		double orientation;

		double depth;

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

//		ros::Subscriber depthSub;

	 image_transport::Subscriber depthSub;

	image_transport::ImageTransport *it;

//image_transport::ImageTransport it;



	public:

		/**
 		*  @brief      Constructor
 		*  @param      None
 		*  @return     None
 		*/
		StateMachine();

		/**
 		*  @brief      Callback function to obtain the images
 		*  @param      A ros message which is the RGB image
 		*  @return     An RGB image to the subscriber, nothing explicit from function
 		*/
		void imageRGBCallback(const sensor_msgs::ImageConstPtr& message);

		/**
 		*  @brief      Callback function to obtain the images
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
		double readDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image);

		void pickupDebris();

		int getState();
		cv::Mat getImage();
		double getRobotXPos();
		double getRobotYPos();
		double getRobotYaw();

};

#endif  // INCLUDE_STATEMACHINE_HPP_

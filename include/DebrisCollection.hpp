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
 *  @file       DebrisCollection.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/05/2019
 *  @version    1.0
 *
 *  @brief      Header file for DebrisCollection class
 *
 */

#ifndef INCLUDE_DEBRISCOLLECTION_HPP_
#define INCLUDE_DEBRISCOLLECTION_HPP_

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

/**
 *  @brief      Elements and members of DebrisCollection class
 */

class DebrisCollection {

	private:
		// State 0 is searching for debris, state 1 found debris, state 2 going to disposal bin
		int state = 0;

		// Storage Point for debris position in pixels
		Point imageDebrisLocation;
		
		double orientation;
		double distanceTraveled;


		double depth;
		double registeredDepth;

		double x;
		double y;

		// Storage Point for debris position i world frame (orientation, distance)
		//Point currentDebrisLocation;


		int proximityThreshold = 300;

		// Storage vector for all debris locations
		std::vector<Point> debrisLocation;

		// Image storage
		cv::Mat lastSnapshot;

		// Creating a node handle
		ros::NodeHandle nh;

		// Defining a node publisher
		ros::Publisher pub;

		// Defining node subscriber for RGB image callback
		ros::Subscriber sub;
		// Defining node subscriber for odometry callback
                ros::Subscriber odomSub;
		// Defining node subscriber for RGB-D depth callback
		image_transport::Subscriber depthSub;

	public:

		/**
 		*  @brief      Constructor
 		*  @param      None
 		*  @return     None
 		*/
		DebrisCollection();

		/**
 		*  @brief      Constructor for Image setter
 		*  @param      RGB image
 		*  @return     None
 		*/
		void setImage(cv::Mat image);

		/**
 		*  @brief      Getter for Image setter
 		*  @param      None
 		*  @return     RGB image
 		*/
		cv::Mat getImage();

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
		void DepthCallback(const sensor_msgs::ImageConstPtr& depthMessage);

		/**
 		*  @brief      Function for applying filter to an RGB image
 		*  @param      An RGB image
 		*  @return     A black and white filtered image, white being for the debris
 		*/
		cv::Mat filter(cv::Mat rawImage);

		/**
 		*  @brief      Function for detecting the centroid of the debris
 		*  @param      The black and white filtered image
 		*  @return     An (x,y) position for the detected debris, if any
 		*/
		void detectDebris(cv::Mat filteredImage);

		/**
 		*  @brief      Function for concatenating debris into a vector if detected
 		*  @param      The location of a detected debris
 		*  @return     None
 		*/
		void addDebris(Point detectedDebris);

		/**
 		*  @brief      Function for removing debris from list after being scooped
 		*  @param      None
 		*  @return     None
 		*/
		void removeDebris();

		/**
 		*  @brief      Function for sorting debris in the Debris vector
 		*  @param      Debris locations in a vector
 		*  @return     Sorted debris locations in a vector
 		*/
		std::vector<Point> sortDebrisLocation(std::vector<Point> * debrisLocations);

		/**
 		*  @brief      Function for obtaining depth information at certain image pixel position
 		*  @param      Image pixel height
 		*  @param      Image pixel wdith
 		*  @param      Depth image from RGB-D camera
 		*  @return     Depth of specified pixel on image
 		*/
		double ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image);

		void pickupDebris();



		void dumpDebris(Point positionOfReceptacle);

		double goToBin(double currentOrientation, double currentDistance);
};

#endif  // INCLUDE_DEBRISCOLLECTION_HPP_

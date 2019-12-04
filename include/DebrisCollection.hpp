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
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Header file for DebrisCollection class
 *
 */

#ifndef INCLUDE_DEBRISCOLLECTION_HPP_
#define INCLUDE_DEBRISCOLLECTION_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include "Point.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "image_transport/image_transport.h"

/**
 *  @brief      Elements and members of DebrisCollection class
 */

class DebrisCollection {

	private:
		cv::Mat rgbImage;
		double hueThreshold;
		double valueThreshold;
		double saturationThreshold;
		std::vector<Point> debrisLocation;
		cv::Mat lastSnapshot;

		// Create a node handle
		ros::NodeHandle nh;

		// Define a node publisher
		ros::Publisher pub;

		// Define a node subscriber
		ros::Subscriber sub;
                ros::Subscriber odomSub;
		image_transport::Subscriber depthSub;

	public:
		/**
 		*  @brief      Constructor for Image setter
 		*  @param      Image
 		*  @return     None
 		*/
		void setImage(cv::Mat image);

		/**
 		*  @brief      Getter for Image setter
 		*  @param      None
 		*  @return     Image
 		*/
		cv::Mat getImage();

		/**
 		*  @brief      Constructor
 		*  @param      None
 		*  @return     None
 		*/
		DebrisCollection();

		/**
 		*  @brief      Callback function to obtain the images
 		*  @param      A ros message which is the compressed image
 		*  @return     An image of type Mat (openCV type)
 		*/
		void imageRGBCallback(const sensor_msgs::ImageConstPtr& message);

		/**
 		*  @brief      Callback function to obtain the images
 		*  @param      A ros message which is the compressed image
 		*  @return     An image of type Mat (openCV type)
 		*/
		void odometryCallback(const nav_msgs::Odometry::ConstPtr& message);

		/**
 		*  @brief      Callback function to obtain depth information from the images
 		*  @param      A ros message which is the image
 		*  @return     None
 		*/
		void DepthCallback(const sensor_msgs::ImageConstPtr& depthMessage);

		/**
 		*  @brief      Function for applying filter to the read image
 		*  @param      The camera image
 		*  @return     An image of type Mat
 		*/
		cv::Mat filter(cv::Mat rawImage);

		/**
 		*  @brief      Function for detecting whether a debris is spotted or not in an image
 		*  @param      The filtered image
 		*  @return     An (x,y) position for the detected debris, if any
 		*/
		Point detectDebris(cv::Mat filteredImage);

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
 		*  @brief      Function for sorting debris according to a criterion
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
};

#endif //  INCLUDE_DEBRISCOLLECTION_HPP

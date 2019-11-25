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

#include <opencv3/opencv.hpp>
#include <Point.hpp>
#include <ros/ros.h>
#include <vector>

class DebrisCollection {

	private:
		cv::Mat rgbImage;
		double hueThreshold;
		double valueThreshold;
		double saturationThreshold;
		std::vector<Point> debrisLocation;

	public:

		/**
 		*  @brief      Callback function to obtain the images
 		*  @param      A ros message which is the compressed image
 		*  @return     An image of type Mat (openCV type)
 		*/
		cv::Mat imageRGBCallback(const sensor_msgs:::CompressedImage &);

		/**
 		*  @brief      Callback function to obtain depth information from the images
 		*  @param      A ros message which is the image
 		*  @return     A double vector with depth information on each pixel
 		*/
		std::vector<double> DepthCallback(const sensor_msgs::Image &);

		/**
 		*  @brief      Constructor
 		*  @param      None
 		*  @return     None
 		*/
		DebrisCollection();

		/**
 		*  @brief      Function for applying filter to the read image
 		*  @param      None
 		*  @return     An image of type Mat
 		*/
		cv::Mat Filter();

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
		addDebris(Point detectedDebris);

		/**
 		*  @brief      Function for removing debris from list after being scooped
 		*  @param      None
 		*  @return     None
 		*/
		removeDebris();

		/**
 		*  @brief      Function for sorting debris according to a criterion
 		*  @param      Debris locations in a vector
 		*  @return     Sorted debris locations in a vector
 		*/
		std::vector<Point> sortDebrisLocation(std::vector<Point> * debrisLocations); 
}

#endif //  INCLUDE_DEBRISCOLLECTION_HPP

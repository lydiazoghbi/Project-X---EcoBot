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
 *  @file       ImageAnalysis.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Header file for ImageAnalysis class
 *
 */

#ifndef INCLUDE_IMAGEANALYSIS_HPP_
#define INCLUDE_IMAGEANALYSIS_HPP_

#include <math.h>

#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"

class ImageAnalysis {

	private:
		Point imageDebrisLocation;

	public:
		/**
 		*  @brief      Constructor for ImageAnalysis class
		*  @param      None
 		*  @return     None
 		*/
		ImageAnalysis();

		/**
 		*  @brief      Obtaining the angle by which the robot should rotate
		*  @param      Robot's current orientation
		*  @param      Distance from robot to debris location
 		*  @return     The angle by which robot should rotate
 		*/
		double getRotationAngle(double currentOrientation, double currentDistance);

		/**
 		*  @brief      Filter function using HSV
		*  @param      RGB image
 		*  @return     Filtered image
 		*/
		cv::Mat filter(cv::Mat rawImage);

		/**
 		*  @brief      Debris detection function
		*  @param      Filtered image using HSV
 		*  @return     None
 		*/
		void detectDebris(cv::Mat filteredImage);

		/**
 		*  @brief      Getter function
		*  @param      None
 		*  @return     Position of debris in image frame
 		*/
		Point getDebrisImageLocation();
};

#endif  // INCLUDE_IMAGEANALYSIS_HPP_

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
 *  @file       Point.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of VelocityGenerator class
 *
 */

//#include "DebrisCollection.hpp"
#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include "Point.hpp"
#include <iostream>
#include "ImageAnalysis.hpp"
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

ImageAnalysis::ImageAnalysis() {};

double ImageAnalysis::getRotationAngle(double currentOrientation, double currentDistance) {

	// Get Robot's (x,y) position 
	double xRobotPosition = currentDistance * cos(currentOrientation);
	double yRobotPosition = currentDistance * sin(currentOrientation);

	// Compute angle
	double numerator = 1.5 - yRobotPosition;
	double denominator = xRobotPosition + 0.2;
	double twist = atan(numerator/denominator);
	double angle = M_PI - twist;

	return angle;

}

cv::Mat ImageAnalysis::filter(cv::Mat rawImage) {
 
	cv::Mat hsvImage, thresholdImage;

	// Convert image from RGB to HSV	
	cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);

	// Apply Hue, Saturation and Value thresholds on HSV image
	cv::inRange(hsvImage, cv::Scalar(0, 33, 50), cv::Scalar(6, 255, 153), thresholdImage);

	// Show image
	cv::imshow("FilteredImage", thresholdImage);
	cv::waitKey(1);

	// Detect the debris in filtered image
	ImageAnalysis::detectDebris(thresholdImage);

	return thresholdImage;
}

void ImageAnalysis::detectDebris(cv::Mat filteredImage) {

	// Apply moments function to obtain centroid of debris 
	cv::Moments moment = moments(filteredImage, true);
	cv::Point cvPoint(moment.m10/moment.m00, moment.m01/moment.m00);

	// If a debris is not detected in the image return -1 as error
	if (cv::countNonZero(filteredImage) < 1) {
		imageDebrisLocation = Point(-1.0, -1.0);
	}
	// If a debris is detected, return the position of debris in image to get depth
	else {
	imageDebrisLocation = Point(cvPoint.x, cvPoint.y);
	}
}

Point ImageAnalysis::getDebrisImageLocation() {
	return imageDebrisLocation;
}


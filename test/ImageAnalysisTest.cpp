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
 *  @file       ImageAnalysisTest.cpp
 *  @author     Ryan Bates and Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/03/2019
 *  @version    1.0
 *
 *  @brief      Level 1 unit test for ImageAnalysis class
 *
 */

#include <math.h>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"
#include "ImageAnalysis.hpp"


/**
 *  @brief      Test function for Image Analysis class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(ImageAnalysis, AngleComputation) {

	// Construct an ImageAnalysis class
	ImageAnalysis imageAnalysis;

	// Define double variables
	double currentOrientation = 0.5;
	double currentDistance = 1.0;

	// Call function
	double angle = imageAnalysis.getRotationAngle(currentOrientation, currentDistance);

	// Check if values returned match expected output
	EXPECT_NEAR(2.38, angle, 0.1);
}

/**
 *  @brief      Test function for Image Analysis class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(ImageAnalysis, PositiveImageThresholding) {

	// Read image and initialize status
	cv::Mat testImage = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/red_2.png", 1);
	bool status = false;

	// Construct an ImageAnalysis Class and a Point for storage of pixel position
	ImageAnalysis imageAnalysis;

	// Apply filter
	cv::Mat filteredImage = imageAnalysis.filter(testImage);

	// Apply moments function to obtain centroid of debris
	cv::Moments moment = moments(filteredImage, true);
	cv::Point cvPoint(moment.m10/moment.m00, moment.m01/moment.m00);

	// If a debris is not detected return false
	if (cv::countNonZero(filteredImage) < 1) {
		status = false;
	} else {
		// If a debris is detected true
		status = true;
	}

	// Check if values returned match expected output
	ASSERT_TRUE(status);
}

/**
 *  @brief      Test function for Image Analysis class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(ImageAnalysis, NegativeImageThresholding) {

	// Read image and initialize status
	cv::Mat testImage = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/no_object_1.png", 1);
	bool status = false;

	// Construct an ImageAnalysis Class and a Point for storage of pixel position
	ImageAnalysis imageAnalysis;

	// Apply filter
	cv::Mat filteredImage = imageAnalysis.filter(testImage);

	// Apply moments function to obtain centroid of debris
	cv::Moments moment = moments(filteredImage, true);
	cv::Point cvPoint(moment.m10/moment.m00, moment.m01/moment.m00);

	// If a debris is not detected return false
	if (cv::countNonZero(filteredImage) < 1) {
		status = false;
	} else {
		// If a debris is detected true
		status = true;
	}

	// Check if values returned match expected output
	ASSERT_FALSE(status);
}

/**
 *  @brief      Test function for Image Analysis class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(ImageAnalysis, DebrisDetection) {

	// Read image
	cv::Mat testImage = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/red_1.png", 1);

	// Construct an ImageAnalysis Class
	ImageAnalysis imageAnalysis;

	// Filter image and detect debris
	cv::Mat filteredImage = imageAnalysis.filter(testImage);
	imageAnalysis.detectDebris(filteredImage);

	// Get debris location
	Point position = imageAnalysis.getDebrisImageLocation();

	/// Check if values returned match expected output
	ASSERT_NEAR(170.0, position.getX(), 15.0);
	ASSERT_NEAR(235.0, position.getY(), 15.0);
}

/**
 *  @brief      Test function for Image Analysis class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(ImageAnalysis, GetDebrisLocation) {

	// Read image
	cv::Mat testImage = cv::imread("../catkin_ws/src/project_x_ecobot/test/testImages/red_2.png", 1);

	// Construct an ImageAnalysis Class
	ImageAnalysis imageAnalysis;

	// Filter image and detect debris
	cv::Mat filteredImage = imageAnalysis.filter(testImage);
	imageAnalysis.detectDebris(filteredImage);

	// Get debris location
	Point position = imageAnalysis.getDebrisImageLocation();

	/// Check if values returned match expected output
	ASSERT_NEAR(170.0, position.getX(), 15.0);
	ASSERT_NEAR(185.0, position.getY(), 15.0);
}

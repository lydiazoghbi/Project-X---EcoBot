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
 *  @file       DebrisCollection.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of DebrisCollection class
 *
 */

#include <vector>
#include "ros/ros.h"
#include "DebrisCollection.hpp"
#include <Point.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include <iostream>

// Class constructor
DebrisCollection::DebrisCollection() {}

// Reading image from the robot's camera
cv::Mat DebrisCollection::imageRGBCallback(const sensor_msgs::CompressedImage& readings) {}

// Reading depth information from the robot's camera
std::vector<double> DebrisCollection::DepthCallback(const sensor_msgs::Image &) {}

// Applying HSV filter to detect debrid
cv::Mat DebrisCollection::Filter() {}

// Function for detecting debris after applying filter
Point DebrisCollection::detectDebris(cv::Mat filteredImage) {}

// Function for concatenating debris information if detected
void DebrisCollection::addDebris(Point detectedDebris) {}

// Function for removing debris from list after it is scooped
void DebrisCollection::removeDebris() {}

// Sorting the debris by closest to bin 
std::vector<Point> DebrisCollection::sortDebrisLocation(std::vector<Point> * debrisLocations) {}


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

#ifndef INCLUDE_DEBRISCOLLECTION_HPP_
#define INCLUDE_DEBRISCOLLECTION_HPP_

#include <vector>
#include <Point.hpp>
#include <opencv3/opencv.hpp>

// Reading image from the robot's camera
DebrisCollection::imageRGBCallback(const sensor_msgs:::CompressedImage &)

// Reading depth information from the robot's camera
DebrisCollection::DepthCallback(const sensor_msgs::Image &)

// Class constructor
DebrisCollection::DebrisCollection()

// Applying HSV filter to detect debrid
cv::Mat DebrisCollection::Filter()

// Function for detecting debris after applying filter
Point DebrisCollection::detectDebris(cv::Mat filteredImage)

// Function for concatenating debris information if detected
DebrisCollection::addDebris(Point detectedDebris)

// Function for removing debris from list after it is scooped
DebrisCollection::removeDebris()

// Sorting the debris by closest to bin 
std::vector<Point> DebrisCollection::sortDebrisLocation(std::vector<Point> * debrisLocations)

#endif //  INCLUDE_DEBRISCOLLECTION_HPP

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

#include <tf/transform_broadcaster.h>
#include "DebrisCollection.hpp"
#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include "Point.hpp"
#include <iostream>
#include "VelocityGenerator.hpp"

// Constructor for the class
VelocityGenerator::VelocityGenerator() {}

// Function for finding velocity commands given a desired goal position
void VelocityGenerator::computeFK(Point desiredPoint) {}

// Callback function to get robot's current position
void VelocityGenerator::positionCallback() {};

// Callback function to get robot's current orientation
void VelocityGenerator::orientationCallback() {};

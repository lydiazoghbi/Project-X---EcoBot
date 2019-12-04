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
 *  @file       main.cpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/02/2019
 *  @version    1.0
 *
 *  @brief      Main function
 *
 */

#include <iostream>
#include "Point.hpp"
#include "DebrisCollection.hpp"
#include "VelocityGenerator.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

/**
*  @brief      Main loop
*  @param      Argc and argv
*  @return     None, exit status as 0
*/
int main(int argc, char **argv) {
	// Initalize ROS node
	ros::init(argc, argv, "DebrisCollection");

	// Initialize node handle
	ros::NodeHandle nh;

	// Call DebrisCollection constructor and output successful initialization
	DebrisCollection debrisCollection;
	ROS_INFO_STREAM("Node initialized.");

	return 0;
}

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
 *  @file       VelocityGenerator.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       11/25/2019
 *  @version    1.0
 *
 *  @brief      Header file for VelocityGenerator class
 *
 */

#ifndef INCLUDE_VELOCITYGENERATOR_HPP_
#define INCLUDE_VELOCITYGENERATOR_HPP_

#include <tf/transform_broadcaster.h>
#include "DebrisCollection.hpp"
#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include "Point.hpp"
#include <iostream>

class VelocityGenerator {

	private:
		double xVelocity;
		double yVelocity;
		double turnVelocity;
		Point binLocation;
		DebrisCollection debrisCollection;

	public:

		/**
 		*  @brief      Constructor
 		*  @param      None
 		*  @return     None
 		*/
		VelocityGenerator();

		/**
 		*  @brief      Function for computing the velocity commands given desired position
 		*  @param      Desired position of the robot
 		*  @return     None
 		*/
		void computeFK(Point desiredPoint);

		/**
 		*  @brief      Callback function for getting the robot's position
 		*  @param      None
 		*  @return     None
 		*/
		void positionCallback();

		/**
 		*  @brief      Callback function for getting the robot's orientation
 		*  @param      Transform
 		*  @return     None
 		*/
		void orientationCallback();
};

#endif //  INCLUDE_VELOCITYGENERATOR_HPP

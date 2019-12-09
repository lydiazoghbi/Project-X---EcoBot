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
 *  @brief      Header file for ImageAnalysis class
 *
 */

#ifndef INCLUDE_LOWXALG_HPP_
#define INCLUDE_LOWXALG_HPP_

//#include "DebrisCollection.hpp"
#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include "Point.hpp"
#include <iostream>

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
#include "IPlanningAlg.hpp"

#include <vector>

class LowXAlg : IPlanningAlg {
	private:
		std::vector<Point> debrisVector;
	public:
		~LowXAlg();		
		bool sortLowestX(Point a, Point b);
		void createPlan(Point robotLocation);
		void push(Point debrisLocation);
		Point pop(Point robotLocation);
};

#endif //  INCLUDE_LOWXALG_HPP

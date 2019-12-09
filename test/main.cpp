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
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Main function for running all tests
 *
 */

#include <math.h>
#include <gtest/gtest.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>

#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <iostream>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "Point.hpp"
#include "LowXAlg.hpp"
#include "GreedyAlg.hpp"
#include "StateMachine.hpp"
#include "IPlanningAlg.hpp"
#include "ImageAnalysis.hpp"

/*
* @brief main function which runs all test results.
* @param This function takes the commandline arguments as input.
* @return This function returns a 0 just to avoid error.
*/
int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "allTests");

  // Initialize Google Test
  ::testing::InitGoogleTest(&argc, argv);

  // Run all tests
  return RUN_ALL_TESTS();
}

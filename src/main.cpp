#include <iostream>
#include "Point.hpp"
#include "DebrisCollection.hpp"
#include "VelocityGenerator.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

int main(int argc, char **argv) {
 
  ros::init(argc, argv, "DebrisCollection");
  ros::NodeHandle nh;
  DebrisCollection debrisCollection;
  ROS_INFO_STREAM("Node initialized.");
  return 0;
}

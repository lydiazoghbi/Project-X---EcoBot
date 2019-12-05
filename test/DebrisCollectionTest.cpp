/**
 *  @file       DebrisCollectionTest.cpp
 *  @author     Ryan Bates
 *  @copyright  Copyright Project-X 2019
 *  @date       11/23/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
Apache Project-X
Copyright 2019 The Apache Software Foundation

This product includes software developed at
The Apache Software Foundation (http://www.apache.org/).

*/

/*
#include <vector>
#include "ros/ros.h"

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Twist.h"
*/


#include <gtest/gtest.h>
#include <ros/ros.h>
#include "Point.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DebrisCollection.hpp"

/*
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
*/


//#include "project_x_ecobot/include/DebrisCollection.hpp"

//using namespace cv;


TEST(DebrisCollection, Dummy) {
// Initiate node handle
  //ros::NodeHandle nh;


 
 //create an instance of debriscollection
  //DebrisCollection debrisCollection;


// wait for a few ms  


//sub = nh.subscribe("/camera/rgb/image_raw", 500, &DebrisCollection::imageRGBCallback, this);
//pub = nh.advertise<sensor_msgs::ImageConstPtr>("/camera/rgb/image_raw", 500);


//start publishing images to debris collection)

//load image

//sensor_msgs::ImageConstPtr message;
//*message = image;

// load as an image



//cv::Mat image = cv::imread("/home/oooo/catkin_ws/src/project_x_ecobot/test/testImages/red_1.png", 1);



//cv::imshow("Loaded image", image);
  //ROS_INFO_STREAM("Image should be displayed");
  //cv::waitKey(0);


//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();


//ImagePtr imagePtr = ???
//pub.publish(msg);



//do callback (which should then call filter, then addDebris)
//call removedebris and verify that debris has been added


  //EXPECT_STREQ("see if it works", srv.response.output.c_str());
}

TEST(DebrisDetection, TestingImageSnapshot) {
	cv::Mat testImage = cv::imread("../src/project_x_ecobot/test/testImages/red_1.png", 1);
	DebrisCollection collectingDebris;
	Point centroid = collectingDebris.detectDebris(testImage);

	EXPECT_EQ(0.1, centroid.getX());
	EXPECT_EQ(0.2, centroid.getY());

}

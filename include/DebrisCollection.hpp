#ifndef INCLUDE_DEBRISCOLLECTION_HPP_
#define INCLUDE_DEBRISCOLLECTION_HPP_

#include <opencv3/opencv.hpp>
#include <Point.hpp>
#include <ros/ros.h>
#include <vector>

class DebrisCollection {

	private:
		cv::Mat rgbImage;
		double hueThreshold;
		double valueThreshold;
		double saturationThreshold;
		std::vector<Point> debrisLocation;

	public:
		imageRGBCallback(const sensor_msgs:::CompressedImage &);
		image DepthCallback(const sensor_msgs::Image &);
		DebrisCollection();
		cv::Mat Filter();
		Point detectDebris(cv::Mat filteredImage);
		addDebris(Point detectedDebris);
		removeDebris();
		std::vector<Point> sortDebrisLocation(std::vector<Point> * debrisLocations); 
}

#endif //  INCLUDE_DEBRISCOLLECTION_HPP

#ifndef INCLUDE_VELOCITYGENERATOR_HPP_
#define INCLUDE_VELOCITYGENERATOR_HPP_

#include <opencv3/opencv.hpp>
#include <DebrisCollection.hpp>
#include <Point.hpp>
#include <ros/ros.h>
#include <vector>

class VelocityGenerator {

	private:
		double xVelocity;
		double yVelocity;
		double turnVelocity;
		Point binLocation;
		DebrisCollection debrisCollection;

	public:
		VelocityGenerator();
		computeFK(Point desiredPoint);
		positionCallback();
		orientationCallback(const tf &);
}

#endif //  INCLUDE_VELOCITYGENERATOR_HPP

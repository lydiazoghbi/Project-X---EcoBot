
#include "IPlanningAlg.hpp"
#include "LowXAlg.hpp"
#include <algorithm>



LowXAlg::~LowXAlg() {
}
void LowXAlg::createPlan(Point robotLocation) {
	// LowX alg just takes the lowest X at any given point, so does not prepare a plan ahead of time
}

void LowXAlg::push(Point debrisLocation) {
	debrisVector.push_back(debrisLocation);
}


Point LowXAlg::pop(Point robotLocation) {
	double lastRobotX = robotLocation.getX();
	double lastRobotY = robotLocation.getY();
	Point closestPoint = debrisVector.front();
	int closestPointIndex = 0;
	int indexCounter = 0;
	for (auto a : debrisVector) {

		if (a.getX() < closestPoint.getX()) {
			closestPoint = a;
			closestPointIndex = indexCounter;
		}
		indexCounter++;
	}
	debrisVector.erase(debrisVector.begin()+closestPointIndex);
	return closestPoint;
}
/*
		bool sortClosest(Point a, Point b) {
			double squaredADistance = ((a.getX() - lastRobotX) * (a.getX() - lastRobotX)) + ((a.getY() - lastRobotY) * (a.getY() - lastRobotY));

			double squaredBDistance = ((b.getX() - lastRobotX) * (b.getX() - lastRobotX)) + ((b.getY() - lastRobotY) * (b.getY() - lastRobotY));

return squaredADistance < squaredBDistance;

}*/

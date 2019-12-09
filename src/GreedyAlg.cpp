
#include "IPlanningAlg.hpp"
#include "GreedyAlg.hpp"
#include <algorithm>



GreedyAlg::~GreedyAlg() {
}
void GreedyAlg::createPlan(Point robotLocation) {
	// Greedy alg just takes the closest at any given point, so does not prepare a plan ahead of time
}

void GreedyAlg::push(Point debrisLocation) {
	debrisVector.push_back(debrisLocation);
}


Point GreedyAlg::pop(Point robotLocation) {
	double lastRobotX = robotLocation.getX();
	double lastRobotY = robotLocation.getY();
	Point closestPoint = debrisVector.front();
	int closestPointIndex = 0;
	int indexCounter = 0;
	if (debrisVector.size() > 0) {
	for (auto a : debrisVector) {
		double squaredADistance = ((a.getX() - lastRobotX) * (a.getX() - lastRobotX)) + ((a.getY() - lastRobotY) * (a.getY() - lastRobotY));
		double squaredBDistance = ((closestPoint.getX() - lastRobotX) * (closestPoint.getX() - lastRobotX)) + ((closestPoint.getY() - lastRobotY) * (closestPoint.getY() - lastRobotY));

		if (squaredADistance < squaredBDistance) {
			closestPoint = a;
			closestPointIndex = indexCounter;
		}
		indexCounter++;
	}
	debrisVector.erase(debrisVector.begin()+closestPointIndex);
	return closestPoint;
	} else {
		return Point(-1, -1);
	}
}
/*
		bool sortClosest(Point a, Point b) {
			double squaredADistance = ((a.getX() - lastRobotX) * (a.getX() - lastRobotX)) + ((a.getY() - lastRobotY) * (a.getY() - lastRobotY));

			double squaredBDistance = ((b.getX() - lastRobotX) * (b.getX() - lastRobotX)) + ((b.getY() - lastRobotY) * (b.getY() - lastRobotY));

return squaredADistance < squaredBDistance;

}*/

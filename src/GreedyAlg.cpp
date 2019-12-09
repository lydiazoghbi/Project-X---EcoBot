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
 *  @file       LowXAlg.cpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Implementation class of GreedyAlg class
 *
 */

#include <vector>
#include <algorithm>

#include "Point.hpp"
#include "GreedyAlg.hpp"
#include "IPlanningAlg.hpp"

// Constructor for the GreedyAlg class
GreedyAlg::~GreedyAlg() {}

// Greedy alg just takes the closest at any given point, so does not prepare a plan ahead of time
void GreedyAlg::createPlan(Point robotLocation) {}

// Function for concatenaning the location of the debris as they are detected
void GreedyAlg::push(Point debrisLocation) {
	debrisVector.push_back(debrisLocation);
}

// Function for removing the debris from the vector when debris has been picked up
Point GreedyAlg::pop(Point robotLocation) {

	// Robot's (x,y) position
	double lastRobotX = robotLocation.getX();
	double lastRobotY = robotLocation.getY();

	// Find debris closest based on greedy principle
	Point closestPoint = debrisVector.front();

	int closestPointIndex = 0;
	int indexCounter = 0;
	// If the debris vector is not empty, then sort be greedy algorithm
	if (debrisVector.size() > 0) {
	for (auto a : debrisVector) {
		double squaredADistance = ((a.getX() - lastRobotX) * (a.getX() - lastRobotX)) + ((a.getY() - lastRobotY) * (a.getY() - lastRobotY));
		double squaredBDistance = ((closestPoint.getX() - lastRobotX) * (closestPoint.getX() - lastRobotX)) + ((closestPoint.getY() - lastRobotY) * (closestPoint.getY() - lastRobotY));
		// Greedy algorithm condition
		if (squaredADistance < squaredBDistance) {
			closestPoint = a;
			closestPointIndex = indexCounter;
		}
		indexCounter++;
	}
	debrisVector.erase(debrisVector.begin()+closestPointIndex);
	return closestPoint;
	} else {
		// If no debris is detected, then return default "empty" Point (-1, -1)
		return Point(-1, -1);
	}
}

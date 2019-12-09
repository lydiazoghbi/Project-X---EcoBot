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
 *  @file       PointTest.cpp
 *  @author     Ryan Bates and Lydia Zoghbi
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/03/2019
 *  @version    1.0
 *
 *  @brief      Level 1 unit test for Point class
 *
 */

#include <gtest/gtest.h>
#include <Point.hpp>
#include <GreedyAlg.hpp>
/**
 *  @brief      Test function for constructing a Point class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(GreedyAlg, firstDebris) {

	GreedyAlg alg;


	Point robotStartLocation(2.0, 2.0);

	Point debrisNear(1.0, 1.0);
	Point debrisFar(10.0, 10.0);

	alg.push(debrisFar);
	alg.push(debrisNear);

	alg.createPlan(robotStartLocation);

	// no need to plan for the greedy algorithm - it is dynamic

	Point firstTarget = alg.pop(robotStartLocation);

	EXPECT_EQ(firstTarget.getX(), debrisNear.getX());
	EXPECT_EQ(firstTarget.getY(), debrisNear.getY());
}

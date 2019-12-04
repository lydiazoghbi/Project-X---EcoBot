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

/**
 *  @brief      Test function for constructing a Point class
 *  @param      Name of Class to be tested
 *  @param      Type of testing
 *  @return     Pass if the test passes
 */

TEST(Point, Constructor) {

	// Construct the class and assign values
	Point point(0.1, 0.2);

	// Read values
	double xPosition = point.getX();
	double yPosition = point.getY();

	// Check if values returned match expected output
	EXPECT_EQ(0.1, xPosition);
	EXPECT_EQ(0.2, yPosition);
}

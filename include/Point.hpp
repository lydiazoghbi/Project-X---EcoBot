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
 *  @file       Point.hpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Header file for Point class
 *
 */

#ifndef INCLUDE_POINT_HPP_
#define INCLUDE_POINT_HPP_

class Point {
	private:
		double x, y;

	public:

		/**
 		*  @brief      Constructor for Point class
 		*  @param      An initial point x
		*  @param	   An initial point y
 		*  @return     None
 		*/
		explicit Point(double startX = 0, double startY = 0);

		/**
 		*  @brief      Function to obtain x value
 		*  @param      None
 		*  @return     The x value that was stored from the constructor
 		*/
		double getX();

		/**
 		*  @brief      Function to obtain y value
 		*  @param      None
 		*  @return     The y value that was stored from the constructor
 		*/
		double getY();
};

#endif  // INCLUDE_POINT_HPP_

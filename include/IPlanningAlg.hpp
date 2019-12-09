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
 *  @brief      Header file for IPlanningAlg.hpp
 *
 */

#ifndef INCLUDE_IPLANNINGALG_HPP_
#define INCLUDE_IPLANNINGALG_HPP_

#include <vector>
#include "Point.hpp"

class IPlanningAlg {

	public:
		/**
 		*  @brief      Virtual function for creating Plan
 		*  @param      Robot's (x,y) location
 		*  @return     None
 		*/
		virtual void createPlan(Point robotLocation) = 0;

		/**
 		*  @brief      Virtual function pusing debris into debris vector
 		*  @param      Debris current location
 		*  @return     None
 		*/
		virtual void push(Point debrisLocation) = 0;

		/**
 		*  @brief      Virtual function reading and removing top element of debris vector
 		*  @param      Robot's (x,y) location
 		*  @return     None
 		*/
		virtual Point pop(Point robotLocation) = 0;
};

#endif  // INCLUDE_IPLANNINGALG_HPP_

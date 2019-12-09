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
 *  @file       GreedyAlg.hpp
 *  @author     Lydia Zoghbi and Ryan Bates
 *  @copyright  Copyright Apache 2.0 License
 *  @date       12/09/2019
 *  @version    1.0
 *
 *  @brief      Header file for GreedyAlg class
 *
 */

#ifndef INCLUDE_GREEDYALG_HPP_
#define INCLUDE_GREEDYALG_HPP_

#include <vector>
#include <algorithm>

#include "Point.hpp"
#include "IPlanningAlg.hpp"

class GreedyAlg : public IPlanningAlg {

	private:
		std::vector<Point> debrisVector;

	public:
		/**
 		*  @brief      Destructor for the GreedyAlg class
 		*  @param      None
 		*  @return     None
 		*/
		~GreedyAlg();

		/**
 		*  @brief      Plan creation for how to pick up the debris
 		*  @param      Robot's current (x,y) location
 		*  @return     None
 		*/
		void createPlan(Point robotLocation);

		/**
 		*  @brief      Function for adding the debris into a debris vector
 		*  @param      Observed debris location
 		*  @return     None
 		*/
		void push(Point debrisLocation);

		/**
 		*  @brief      Function for extracting the debris on the top of the vector
 		*  @param      Robot's current (x,y) location
 		*  @return     Position of debris
 		*/
		Point pop(Point robotLocation);
};

#endif  // INCLUDE_GREEDYALG_HPP_

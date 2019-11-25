# Project X - EcoBot
[![Build Status](https://travis-ci.org/lydiazoghbi/project-x-ecobot.svg?branch=master)](https://travis-ci.org/lydiazoghbi/project-x-ecobot)
[![Coverage Status](https://coveralls.io/repos/github/lydiazoghbi/project-x-ecobot/badge.svg?branch=master)](https://coveralls.io/github/lydiazoghbi/project-x-ecobot?branch=master)

## License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

## Project Overview
Cleaning up facilities can be a tedious, time-consuming, and boring task. ACME’s engineers often drop papers and other waste material around their robot testing facilities, not just dirt as cleaned by regular Roombas, which need to be collected on a daily basis. To save ACME time and money hiring staff, we are proposing to develop a software for a Turtlebot held in their inventory, as shown in Figure 1, with project code name “Agent X”. The Turtlebot will inspect the area (a simple rectangular area enclosed by 4 walls) using an RGB-D camera, localizing and identifying objects that need to be picked up. While some of the trash will be recyclable, other pieces will not, so a stretch requirement involves the development of an algorithm to help Turtlebot distinguish between the two, and pick the waste material using an attached scoop in two separate batches. After the robot collects all trash, it will dump it into smart waste bins recessed into the floor, the exact location of which are known to the robot beforehand. As of now, the waste bins are permanently open, however, if time permits, we will add proximity sensors to them, which would get triggered as a Turtlebot approaches them to open up automatically for the robot to dump in the trash. The software will ensure the robot succeeds in its task, while maintaining the robot testing area at ACME clean and ready for next day’s experiments. Additionally, the software (such as specific ROS nodes, etc.) can be utilized for other applications and even be extended to outdoor environments as well, increasing its value for ACME, and turning it into a potentially commercializable product.

## Poject Demonstration
To be completed.

## Agile Iterative Process (AIP)
The link to the AIP spreadsheet is found [here](https://docs.google.com/spreadsheets/d/1pjMSu-9o1hziTQA7fHETMP-O-lQ8yR-F1kx0K6N_Jzk/edit#gid=0).
The link to the Sprint's planning notes is found [here](https://docs.google.com/document/d/1TawzA07NZ3r3lwr7NOKdyT1iYrHRVG9vpzaS6Y3dToU/edit?ts=5dd9c61d).

## Developers' Information
* Ryan Bates (rjb3vp)*
Ryan is an embedded software engineer with five years of experience at Northrop Grumman.  
Upon graduation in December he aims to find work bringing software processes and skills to the interdisciplinary field of robotics.
* Lydia Zoghbi (lydiazoghbi)*
A master's recipient and a Dean's Fellowship PhD student at the University of Maryland, 
Lydia is a member of IMAPS and is an active researcher with biomedical robotic machine learning, 
including development of an autonomous robotic ultrasound system at MRE.  
When not busy with her studies or research, she enjoys spending time with her boyfriend, cat, and tasty ice cream.

## Dependencies
As of now, the program will require Ubuntu 16.04, ROS Kinetic Kame, Gazebo 7.x (part of ros-kinetic-desktop-full package), Turtlebot3 and openCV3.

## Build Instructions
To be completed.

## Running the Demo Using Launch
To be completed.

## Recording and Playing Bag Files
To be completed.

## Running GTests
To be completed.

## Known Issues and Bugs
To be completed.

## Generating Doxygen Documentation
To be completed.

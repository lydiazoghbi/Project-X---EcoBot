# Project X - EcoBot
[![Build Status](https://travis-ci.org/lydiazoghbi/project_x_ecobot.svg?branch=Sprint-3)](https://travis-ci.org/lydiazoghbi/project_x_ecobot)
[![Coverage Status](https://coveralls.io/repos/github/lydiazoghbi/project_x_ecobot/badge.svg?branch=Sprint-3)](https://coveralls.io/github/lydiazoghbi/project_x_ecobot?branch=Sprint-3)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0.
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

## Project Overview
Cleaning up facilities can be a tedious, time-consuming, and boring task. ACME’s engineers often drop papers and other waste material around their robot testing facilities, not just dirt as cleaned by regular Roombas, which need to be collected on a daily basis. To save ACME time and money hiring staff, we are proposing to develop a software for a Turtlebot held in their inventory, as shown in Figure 1, with project code name “Agent X”. The Turtlebot will inspect the area (a simple rectangular area enclosed by 4 walls) using an RGB-D camera, localizing and identifying objects that need to be picked up. While some of the trash will be recyclable, other pieces will not, so a stretch requirement involves the development of an algorithm to help Turtlebot distinguish between the two, and pick the waste material using an attached scoop in two separate batches. After the robot collects all trash, it will dump it into smart waste bins recessed into the floor, the exact location of which are known to the robot beforehand. The software will ensure the robot succeeds in its task, while maintaining the robot testing area at ACME clean and ready for next day’s experiments. Additionally, the software (such as specific ROS nodes, etc.) can be utilized for other applications and even be extended to outdoor environments as well, increasing its value for ACME, and turning it into a potentially commercializable product.

## Developers' Information
* Ryan Bates (rjb3vp)*
Ryan is an embedded software engineer with five years of experience at Northrop Grumman.  
Upon graduation in December he aims to find work bringing software processes and skills to the interdisciplinary field of robotics.
* Lydia Zoghbi (lydiazoghbi)*
A master's recipient and a Dean's Fellowship PhD student at the University of Maryland, 
Lydia is an active researcher with biomedical robotic machine learning, 
including development of an autonomous robotic ultrasound system at the Medical Robotics and Equipment Lab.  
When not busy with her studies or research, she enjoys spending time with her boyfriend, cat, and tasty ice cream.

## Agile Iterative Process (AIP)
This algorithm was developed using the AIP process, which provides us some work flexibility for changing requirements. We used a Test Driven Development (TDD), whereby a system test is first developed, followed by unit tests (mind that they are supposed to all fail at the beginning), and then the codes are developed to meet requirements and tests passing. At the end of each Sprint (1 week in the case of this work), a review session is held between the developers, and plans are drawn out for the upcoming Sprint.
The link to the AIP spreadsheet can be found [here](https://docs.google.com/spreadsheets/d/1pjMSu-9o1hziTQA7fHETMP-O-lQ8yR-F1kx0K6N_Jzk/edit#gid=0).
The link to the Sprint's planning notes can be found [here](https://docs.google.com/document/d/1TawzA07NZ3r3lwr7NOKdyT1iYrHRVG9vpzaS6Y3dToU/edit?ts=5dd9c61d).

## Dependencies and Installation
This section will walk you through the full list of dependencies and how to install them on your machine. You will need an Ubuntu 16.04 operating system. We know it is tempting to just go ahead and install the more recent Ubuntu 18 version, but please bear with us, as we only guarantee this algorithm to run on Ubuntu 16. You might be able to run the code on more recent systems, but the chances of it working are fairly slim. You will need to have the ROS Kinetic full desktop version:
* Follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS Kinetic. It is highly recommended to install the full desktop version.
* Install catkin from this [link](http://wiki.ros.org/catkin)
The code depends on Gazebo 7.x, but it's your lucky day! It comes with the ROS Kinetic full desktop distribution, so you don't need to install anything for it.

The code depends on Turtlebot, which can be installed by running the following commands from the terminal:
```
sudo apt-get instal ros-kinetic-turtelbot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz launchers
```
You will need to have lcov installed, run in a terminal:
```
sudo apt-get install lcov
```
Lastly, you will need to have OpenCV installed on your machine. The following commands should theoretically work, however we are not responsible for any anomalies or errors that might pop up linked to the specificity of your machine. In a terminal, type the following:
```
sudo apt -y update
sudo apt -y upgrade
```
This will update and upgrade your system. Next you need to install the dependencies for OpenCV itself. Type the following:
```
sudo apt -y remove x264 libx264-dev
sudo apt-get install build-essential checkinstall cmake pkg-config yasm
sudo apt-get install git gfortran
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
sudo apt-get install libtiff5-dev 
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
sudo apt-get install libprotobuf-dev protobuf-compiler
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
If you reached this step you've finished the hardest part. Now clone the OpenCV repository, build a directory and RUN!:
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
```
For compiling the code, first find the number of CPU cores you have on your machine. Type nproc in a terminal, and in the following code, instead of -j4, put the number that was just outputted (so if you're lucky and you have 12 cores like I do, you type -j12). The run the rest in the terminal:
```
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```
If you're not confident with your installation because you chose to ignore some previous errors and hope for the best, this [link](https://www.learnopencv.com/install-opencv3-on-ubuntu/) provides at the end a small test example which you can run to make sure you have everything set up correctly.

## Poject Demonstration
You can see here our turtlebot in cleaning action. The environment of the robot was designed using Solidworks to provide us some flexibility with the specific requirements of our idea, and exported as a URDF package. A scooper was attached to the robot via 3 rectangular blocks. They were added by directy modifying the turtlebot's URDF, and saving it to this repository. The code was changed to have the launch file call the turtlebot in this repository, and not the one from your system. The robot first scans the entire region in search for debris, and as they are detected, they are stored in a vector. A Kinect RGB-D camera is used to obtain the images, and a simple HSV filter (for simplicity's sake) is applied to detect the debris. Once a debris is detected, a callback function determines the depth of the debris, which effectively represents the distance between the robot and the debris. The orientation of the robot and depth of the debris are used as guiding information to pick it up. The robot's odometry readings are used to localize its position in the world frame. For simplicity's sake, we assume a no-slip condition. The robot will move towards the debris, rotate towards the disposal bin, and head towards it until the debris is thrown out. It will then rotate to the other debris, and follow the same routine until all debris are disposed of.

## Build Instructions
To run this algorithm yourself, you need to first create your catkin workspace. Type the following commands in your terminal:
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make
```
Now to clone this repo and build it:
```
cd ~/catkin_ws/src/
git clone --recursive https://github.com/lydiazoghbi/project_x_ecobot.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
If you have installed everything correctly, you should see a successful build. Otherwise, it's kind of hard for me to predict what errors you might face.

## Running the Demo Using Launch
After following the build instructions, to run the demo using our launch file, type in a terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch project_x_ecobot cleaner.launch
```
This will automatically start up Gazebo, and the robot will start its cleaning operation.

## Recording and Playing Bag Files
The demo using the launch file in the previous section is set to automatically disable rosbag recodings. If you wish to record a rosbag file, type in a new terminal (after closing all other simulations):
```
roslaunch project_x_ecobot cleaner.launch record:=1
```
This will record the rosbag file until you decide to terminate it using Ctrl+C. It will store the rosbag in the /results directory in this repository. If you want to replay our presented rosbag files, then you first need to decompress them. So type in a terminal:
```
cd ~/catkin_ws/src/project_x_ecobot/results
rosbag decompress *.bag
```
This will create an additional rosbag file (named walker.bag), and rename the original compressed file to walker.ori.bag. To play the rosbag file type the following after killing all other nodes (note, a Gazebo simulation will not open):
```
roscore
```
In a new terminal, type:
```
cd ~/catkin_ws/src/turtlebot_walker/results
rosbag play ./walker.bag
```
If you wish to inspect the topics, type in a new terminal:
```
rosptopic echo <topic_name>
```
Note that the camera readings are disabled for the recording from the launch file, since the size of the data saved will be humongous. 

## Running GTests
The tests coverage is shown on the Coveralls badge at the top of the repository, however if you want to run them yourself and look for the tests that pass, then simply run in a terminal:
```
cd ~/catkin_ws/
catkin_make run_tests_project_x_ecobot
```
You should see the name of the tests, and their status (whether they pass or fail).

## Known Issues and Bugs
Please be aware that the system's performance varies greatly from one computer to the other. So what works on one machine, may show lags, delays and malfunctions on a slower machine. One potential problem is the error accumulation in the robot's odometry readings. Since we are not accounting for slipping, as the robot collects more and more debris, more inaccuracy is added to its performance. This should be in the future mitigated with a robust localization algorithm.

## Generating Doxygen Documentation
To generate the Doxygen documentation, type the following:
```
sudo apt install doxygen
```
Chances are high that you aready have it, but nevertheless, never a bad idea to double check. Then run:
```
cd ~/catkin_ws/src/
doxygen DoxyFile
```
This will generate doxygen files in the /docs folder. To view them in a browser, type:
```
cd /docs/html
firefox index.html
``` 

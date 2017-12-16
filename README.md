# Ping-Pong_Picker
[![Build Status](https://travis-ci.org/ShaotuJia/Ping-Pong_Picker.svg?branch=master)](https://travis-ci.org/ShaotuJia/Ping-Pong_Picker)
[![Coverage Status](https://coveralls.io/repos/github/ShaotuJia/Ping-Pong_Picker/badge.svg?branch=master)](https://coveralls.io/github/ShaotuJia/Ping-Pong_Picker?branch=master)

## LICENSE
GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>

Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.

## Overview
TurtleBot is a low-cost, personal robot kit with open-source software.  With TurtleBot, we will be able to build a robot that can drive around your house, see in 3D, and have enough horsepower to create exciting applications. It is possible to modify a TurtleBot to a ping-pong picker which can pick up ping-pong from the ground. To do this, we can attach vacuum components and a basket to the TurtleBot. While the TurtleBot is moving, the balls on the ground will be vacuum to the basket. This project is to implement a ROS package to let TurtleBot walk on the ping-pong court. At the final of this project, the TurtleBot will be able to walk on table tennis court following a setup route.  

## Enacting and Measuring SIP
The backlog, time_log, and defect_log are written in google sheet. [click_here](https://docs.google.com/spreadsheets/d/1oc-uXwMSMHGoVznmhH2XxiHXGW3xBzAiTr5E5UuIOBQ/edit?usp=sharing)
- Backlog: the project task plan
- Time_log: the record of actual works
- Defect_log: the record of fix defects

## Final Presentation: 
The final presentation vidoe explains this project in detail. 
Please click this link to presentation video. [Presentation_Video](https://youtu.be/3VUBzabYdZE) 
Please click this link to presentation slides. [Presentation_Slides](https://docs.google.com/presentation/d/10CILncwVDDl1R7wfRp30VpyXKJg2g7Du5HfB-6QjVRU/edit?usp=sharing) 

## Requirement and Package Dependency:
Since the Travis does not support Ubuntu 16.04 and ROS Kinetic I made this package fully compatible with ROS Indigo. This package can be run in either ROS Kinetic and ROS Indigo. 
- System and Implementation
  - Operating System: Ubuntu 16.04 or Ubuntu 14.04
  - Robot Operating System: ROS Kinetic or ROS Indigo
  - Build Tool: Catkin
  - Programming Language: C++ with C++ 11/14 Features
- ROS Package Dependency:
  - roscpp
  - rostest
  - rosbag
  - TurtleBot Gazebo
  - tf
- Messages:
  - std_msgs
  - Kobuki_msgs
  - gazebo_msgs
## Build Package
If you do not have catkin workspace please using following command to create a workspace at first.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Next, clone repository from github and build it using catkin
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ShaotuJia/Ping-Pong_Picker.git
$ cd ~/catkin_ws
$ catkin_make
```

## Ping-pong Court in Gazebo World
This image shows the world that TurtleBot will work on.
![ping_pong_court_world](https://user-images.githubusercontent.com/16978713/34065472-f44523a8-e1cf-11e7-81e9-e783a043fb6b.png)

## Package Demo Tutorial
This package provides a demo to show how to use this package. At first, we need to set up several flag point (Red Point on below image). And then, we need form these flag point in a sequency. This sequency will be the route of TurtleBot. Once TurtleBot finish this route, we use go_home function to move TurtleBot to the origin and prepare for future work. 
![ping_pong_picker_demo](https://user-images.githubusercontent.com/16978713/34065438-9501fbaa-e1cf-11e7-9f23-3238d899743d.png)

Please use below commands to run this demo.
If you do not want to record rosbag, please using these commands. 
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch ping_pong_picker picker.launch 
```
If you want to record rosbag, please using these commands. This will record rosbag for 300 secs. 
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch ping_pong_picker picker.launch baging:=1 duration:=300
```
## Rostest and Code Coverage
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ catkin_make run_tests
```
The coveralls currently has issue to show the detail of code coverage and its code coverage is much different to the local report. Thus, please see the code coverage report on local computer.

If you have not install coveralls and lcov, please use following commands.
```
$ sudo apt-get install lcov
$ gem install coveralls-lcov
```
Generate code coverage report:
```
$ cd ~/catkin_ws/build
$ lcov --capture --directory ping_pong_picker/ --output-file coverage.info
$ lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
$ genhtml coverage.info --output-directory out
$ firefox out/index.html 
```
## Final Deliverables:
- [x] The implemented package with desired functions in initial design and first iteration
- [x] The unit testing for implementation using ROSTEST
- [x] A tutorial of how to use this implemented package
- [x] A sample video to show how this package working on the TurtleBot and the built gazebo world
- [x] Product Backlog, time log, and iteration log for tracking the whole implementation procedures
- [x] Doxygen Documentation

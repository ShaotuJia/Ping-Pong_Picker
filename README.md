# Ping-Pong_Picker
[![Build Status](https://travis-ci.org/ShaotuJia/Ping-Pong_Picker.svg?branch=master)](https://travis-ci.org/ShaotuJia/Ping-Pong_Picker)
## Abstract
This is the final project for ENPM808X: Software Development for Robotics. This project will achieve the motion part of picker-robot based on turtlebot. Finally, the turtlebot will walk on table tennis court without collision. 

## Enacting and Measuring SIP
The backlog, time_log, and defect_log are written in google sheet. [click_here](https://docs.google.com/spreadsheets/d/1oc-uXwMSMHGoVznmhH2XxiHXGW3xBzAiTr5E5UuIOBQ/edit?usp=sharing)
- Backlog: the project task plan
- Time_log: the record of actual works
- Defect_log: the record of fix defects

## LICENSE
GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>

Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.

## Requirement and Package Dependency:
- System and Implementation
  - Operating System: Ubuntu 16.04
  - Robot Operating System: ROS Kinetic
  - Build System: Catkin
  - Programming Language: C++ with C++ 11/14 Features
- ROS Package Dependency:
  - TurtleBot
  - Gazebo
  - Navigation Stack
  - Other packages about ROS messages

## Motivation
Table Tennis is a popular sport in China. According to recent data, all over the nation, there are
2000 professional players, 30000 amateurs, and 83 million people who play table tennis twice a
week. During table tennis training, like tennis, the Multi-ball Training method is an efficient way
to help players improve fast (Figure 1). However, during Multi-ball training, balls will be
everywhere on the playground; all the white points on the ground of Figure 1 are ping-pong.
Thus, it is also a tried work to pick up ping-pong during training. To solve this problem, in this
project, I would like to design a mobile ping-pong picker based on TurtleBot.

## Design Outline
TurtleBot can be mechanically modified to a ping-pong picker by adding a vacuum component
and a basket. Ping-pong is small and light, thus, it is possible to be picked up by specially
designed vacuum. In this project, I only focus on the motion planning of TurtleBot in this
application and the mechanical design would be another project.

### Initial design
In the initial design, the TurtleBot will move on the playground like Roomba. It will start to move
along the wall since players like to kick balls towards walls during training. And then, go inside
the playground. Once the bumper of TurtleBot hit any obstacles, it turns to another direction.
This design can be used when players finish training, and there is no player playing on the
playground.

### First Iteration
The first iteration is to add functions to let TurtleBot avoid moving players. In this application, the
navigation stack will be used to plan the TurtleBot motion and also avoid moving obstacles. In
this case, the designed picker can be used during players training since it will not disturb
players.

## Development Process:
This project will focus on initial design and first iteration. The object detection in the second
iteration is quite complicated. Thus, it will be left to the future work.
- Build gazebo world based on actual table tennis playground
- Implement the initial design for TurtleBot
- Add moving obstacles in gazebo world
- Implement the first iteration to let TurtleBot move without colliding moving obstacles.

## Final Deliverables:
- [ ] The implemented package with desired functions in initial design and first iteration
- [ ] The unit testing for implementation using ROSTEST
- [ ] A tutorial of how to use this implemented package
- [ ] A sample video to show how this package working on the TurtleBot and the built gazebo world
- [ ] Product Backlog, time log, and iteration log for tracking the whole implementation procedures
- [ ] Doxygen Documentation

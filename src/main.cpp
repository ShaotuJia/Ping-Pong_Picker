/*
 * @file main.cpp
 * @brief This is the main file to let turtlebot move in ping-pong court to pick up balls
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */

#include <ros/ros.h>
#include <memory>
#include <vector>
#include "turtlebot_walker/walker.hpp"
#include "turtlebot_walker/mode.hpp"

/**
 * @brief This is the main function to run node walker
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "walk_turtlebot");	///< Initialize node with name "walk_turtlebot"
	auto mode = std::make_unique<Mode>();
	geometry_msgs::Point flag = mode->set_flag(1,7);
    auto reach1 = mode->go_flag(flag);


#if 0
	Walk walker;	///< declare a object for class Walk
	walker.set_initial_pose(0, 0);	///< set up initial pose for turtlebot
	walker.set_linear(0.2);	///< set up linear velocity when moving forward
	walker.set_angular(0.1);	///< set up angular velocity when hitting obstacles
	//walker.set_up_position();	///< locate turtlebot in desired location
	//walker.set_up_worktime(1);	///< set up the work time limit
	//walker.linear_move(10);	///< move the turtlebot with a simple walk algorithm like roomba
	//walker.rotate(3.5);
	//walker.where_turtle();
	//bool isReach = walker.rotate(0);
	auto reach1 = walker.linear_move(1,7);
	auto reach2 = walker.linear_move(7,7);

#endif

}



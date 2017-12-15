/*
 * @file main.cpp
 * @brief This is the main file to let turtlebot move in ping-pong court to pick up balls
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 * @description This is a demo to show how turtlebot walk in a ping-pong court.
 * The turtlebot will walk start from the origin and go to several desired point;
 * finally go back to the origin.
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
	std::unique_ptr<Mode> mode;

	geometry_msgs::Point flag_1 = mode->set_flag(0.3,1); 	///< set flag 1; left up corner
	geometry_msgs::Point flag_2 = mode->set_flag(0.3,9.7);	///< set flag 2; right up corner
	geometry_msgs::Point flag_3 = mode->set_flag(9.7,9.5);	///< set flag 3; right down corner
	geometry_msgs::Point flag_4 = mode->set_flag(9.7,0.3);	///< set flag 4; left down corner
	geometry_msgs::Point flag_5 = mode->set_flag(1, 0.5);	///< set flag 5; left up corner
	std::vector<geometry_msgs::Point> flag_seq = {flag_1, flag_2, flag_3, flag_4, flag_5};		///< set up this route
	auto finish = mode->go_route(flag_seq);		///< go this route
	auto isHome = mode->go_home();		///< go origin

}



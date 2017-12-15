/*
 * @file mode.cpp
 * @brief This file is to let user move turtlebot simply by set up designed mode
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/SetModelState.h>
#include <memory>
#include <vector>
#include <cmath>
#include "turtlebot_walker/walker.hpp"
#include "turtlebot_walker/mode.hpp"

/**
 * @brief This function is to set up a flag that turtlebot will go to
 * @param x The coordinate in x-direction
 * @param y The coordinate in y-direction
 * @return point
 */
geometry_msgs::Point Mode::set_flag(const double& x, const double& y) {

	geometry_msgs:: Point flag;
	flag.x = x;
	flag.y = y;
	return flag;
}

/**
 * @brief This function is to go to the desired point
 * @param flag The desired point turtlebot will go to
 * @return isReach whether the turtlebot reach the desired point
 */
bool Mode::go_flag(geometry_msgs::Point flag) {

	Walk walker;	///< declare a object for class Walk
	walker.set_linear(0.2);	///< set up linear velocity when moving forward
	walker.set_angular(0.1);	///< set up angular velocity when hitting obstacles
	walker.set_straight_tolerance(0.5);  ///< set up tolerance in straigth distance
	auto isReach = walker.linear_move(flag.x,flag.y);
	return isReach;
}

/**
 * @brief This function go a series of flag
 * @param flag_seq A series point turtlebot will go to
 * @return
 */
bool Mode::go_route(std::vector<geometry_msgs::Point> flag_seq) {
	auto isFinish = false;
	for (auto flag:flag_seq) {
		 isFinish = go_flag(flag);
	}
	return isFinish;
}

/**
 * @breif This function let turtlebot go to the origin from current location
 */
bool Mode::go_home() {
	geometry_msgs::Point origin = set_flag(0, 0);
	auto isHome = go_flag(origin);
	return isHome;
}


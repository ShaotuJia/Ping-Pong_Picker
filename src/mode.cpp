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
 */
void Mode::set_flag(const double& x, const double& y) {
	flag.x = x;
	flag.y = y;
}

/**
 * @brief This function is to set up a series of flag that turtlebot will go to
 * @param flag_seq
 */
void Mode::set_flag_seq(std::vector<geometry_msgs::Point> seq) {
	flag_seq = seq;
}

/**
 * @brief This function is to obtain the flag point
 */
geometry_msgs::Point Mode::get_flag() {
	return flag;
}

/**
 * @brief This function is to obtain the flag_seq
 */

std::vector<geometry_msgs::Point> Mode::get_flag_seq() {
	return flag_seq;
}



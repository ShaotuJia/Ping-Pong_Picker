/*
 * @file mode.hpp
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


#ifndef INCLUDE_TURTLEBOT_WALKER_MODE_HPP_
#define INCLUDE_TURTLEBOT_WALKER_MODE_HPP_

class Mode {

 public:
	geometry_msgs::Point set_flag(const double&, const double&);
    bool go_flag(geometry_msgs::Point);
	bool go_route(std::vector<geometry_msgs::Point>);
	bool go_home();
};




#endif /* INCLUDE_TURTLEBOT_WALKER_MODE_HPP_ */

/*
 * @file main.cpp
 * @brief This is the main file to run turtlebot like roomba
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */

#include <ros/ros.h>
#include "turtlebot_walker/walker.hpp"

/**
 * @brief This is the main function to run node walker
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "walk_turtlebot");	///< Initialize node with name "walk_turtlebot"
	Walk walker;	///< declare a object for class Walk
	walker.set_initial_pose(0, 0);	///< set up initial pose for turtlebot
	walker.set_linear(0.4);	///< set up linear velocity when moving forward
	walker.set_angular(1);	///< set up angular velocity when hitting obstacles
	walker.set_up_position();	///< locate turtlebot in desired location
	//walker.set_up_worktime(1);	///< set up the work time limit
	//walker.linear_move(10);	///< move the turtlebot with a simple walk algorithm like roomba
	//walker.rotate(3.5);
	//walker.where_turtle();

	tf::TransformListener listener;
	ros::Rate rate(10);
	while(ros::ok()) {
		tf::StampedTransform transform;
	    listener.waitForTransform("/base_footprint", "/odom",ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/base_footprint","/odom",ros::Time(0),transform);
		ROS_INFO("get tf %f",transform.getOrigin().x());
		rate.sleep();
	}


}



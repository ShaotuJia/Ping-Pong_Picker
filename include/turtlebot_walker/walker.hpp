/*
 * @file walker.hpp
 * @brief This file declares the class Walk for a turtlebot walk to desired location in ping-pong court
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/SetModelState.h>
#include <memory>

class Walk {
private:
	geometry_msgs::Twist linear_velo;	///< The velocity of turtlebot move forward
	geometry_msgs::Twist angular_velo;	///< The angular velocity of rotation once the turtlebot collide.
	ros::NodeHandle n;	///< nodehandle for class walk
	bool need_turn = false;	///< check whether turtlebot need to turn to avoid obstacle
	geometry_msgs::Point position;	///< the initial position of turtlebot
	geometry_msgs::Point goal;		///< the goal we want turtlebot to go
	geometry_msgs::Point current_pose;	///< the current the position of turtlebot
	tf::Quaternion current_orientation; ///< the current orientation of turtlebot
	double rotate_tolerance = 0.05;		///< the tolerance of turtlebot rotate
	double straight_tolerance = 1;	///< The tolerance of turtlebot move straight
	double orginal_orientation = 1.59;
	double desired_angle = 0;	///< the angle turtlebot need to turn


public:
	bool linear_move(double x, double y);
	bool rotate(double angle);
	void collision(const kobuki_msgs::BumperEvent::ConstPtr& bumper_state);
	void set_linear(const double&);
	void set_angular(const double&);
	geometry_msgs::Twist reset_current_angular(geometry_msgs::Twist);
	void set_initial_pose(const double&, const double&);
	void set_up_position();
	geometry_msgs::Transform where_turtle();
	geometry_msgs::Point get_current_pose();
	tf::Quaternion get_current_orientation();
	double diff_dist();
	double diff_angle();
	bool isdiffAngle(tf::Quaternion current_orientation, double angle);
	void set_up_goal(double x, double y);
	bool whether_reverse(tf::Quaternion current_orientation);
	geometry_msgs::Point get_initial_pose();
	geometry_msgs::Point get_goal();
	geometry_msgs::Twist get_linear_velo();
	geometry_msgs::Twist get_angular_velo();
	void set_straight_tolerance(const double&);
	void set_rotate_tolerance(const double&);
	double get_straight_tolerance();
	double get_rotate_tolerance();
};



#endif  // INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_

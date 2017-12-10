/*
 * @file walker.hpp
 * @brief This file declares the class Walk for a simple walk algorithm
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
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/SetModelState.h>
#include <memory>

class Walk {
private:
	geometry_msgs::Twist linear_velo;	///< The velocity of turtlebot move forward
	geometry_msgs::Twist angular_velo;	///< The angular velocity of rotation once the turtlebot collide.
	ros::NodeHandle n;	///< nodehandle for class walk
	int work_time  = 0;	///< the turtlebot will stop moving once reach the work time limit
	bool need_turn = false;	///< check whether turtlebot need to turn to avoid obstacle
	geometry_msgs::Point position;



public:
	void linear_move(double time_limit);
	void rotate(double angle);
	void collision(const kobuki_msgs::BumperEvent::ConstPtr& bumper_state);
	void set_linear(const double&);
	void set_angular(const double&);
	void set_initial_pose(const double&, const double&);
	void set_up_position();
	void set_up_worktime(int time);
	//tf::StampedTransform where_turtle();
	void where_turtle();
};



#endif  // INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_

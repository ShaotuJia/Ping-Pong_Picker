/*
 * @file walker.cpp
 * @brief This file implements a class that moves turtlebot to desired point
 * when given the 2-D coordinate of goal
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
#include <cmath>
#include "turtlebot_walker/walker.hpp"

/**
 * @brief This function is the interface to set the velocity that moving turtlebot forward
 * @param x The linear velocity in forward direction
 */
void Walk::set_linear(const double& x) {
	linear_velo.linear.x = x;
	linear_velo.linear.y = 0;
	linear_velo.linear.z = 0;
	linear_velo.angular.x = 0;
	linear_velo.angular.y = 0;
	linear_velo.angular.z = 0;
}

/**
 * @brief This function is to obtain the linear velocity of turtlebot
 * @return linear_velocity
 */
geometry_msgs::Twist Walk::get_linear_velo() {
	return linear_velo;
}

/**
 * @brief This function is the interface to set the angular velocity of turtlebot
 * when it needs to avoid obstacles
 * @param r The angular velocity around z_direction
 */
void Walk::set_angular(const double& r) {
	angular_velo.linear.x = 0;
	angular_velo.linear.y = 0;
	angular_velo.linear.z = 0;
	angular_velo.angular.x = 0;
	angular_velo.angular.y = 0;
	angular_velo.angular.z = r;
}

/**
 * @breif This function is reset angular velocity into positive direction before each movement
 */
geometry_msgs::Twist Walk::reset_current_angular(geometry_msgs::Twist current_angular) {
	if (current_angular.angular.z < 0) {
		current_angular.angular.z = std::abs(current_angular.angular.z);
	}
	return current_angular;
}

/**
 * @brief This function is to obtain the angular velocity of turtlebot
 * @return angualr_velocity
 */
geometry_msgs::Twist Walk::get_angular_velo() {
	return angular_velo;
}

/**
 * @brief This function is the interface to set up the initial position of turtlebot
 * @param x The position in x - direction
 * @param y The position in y - direction
 */
void Walk::set_initial_pose(const double& x, const double& y) {
	position.x = x;
	position.y = y;
}

/**
 * @brief This function is to obtain the initial position of turtlebot
 * return position
 */
geometry_msgs::Point Walk::get_initial_pose() {
	return position;
}

/**
 * @brief Set up the straight tolerance; the tolerance between desired goal and actual goal
 * @param straight tolerance
 */
void Walk::set_straight_tolerance(const double& tolerance) {
	straight_tolerance = tolerance;
}

/**
 * @brief Obtain the straight tolerance
 * @return straight_tolerance
 */
double Walk::get_straight_tolerance() {
	return straight_tolerance;
}

/**
 * @brief Set up the tolerance in rotation
 * @param angular_tolerance
 */
void Walk::set_rotate_tolerance(const double& tolerance) {
	rotate_tolerance = tolerance;
}

/**
 * @brief Get rotate tolerance
 * @return rotate_tolerance
 */
double Walk::get_rotate_tolerance() {
	return rotate_tolerance;
}

/**
 * @brief This is a callback function to subscribe the topic /mobile_base/events/bumper
 * @param bumper_state The message from subscribed topic
 */
void Walk::collision(const kobuki_msgs::BumperEvent::ConstPtr& bumper_state) {
	auto state = bumper_state->state;
	if (state == 0) {
		need_turn = false;
	} else {
		need_turn = true;
	}

}

/**
 * @brief The function is to set up the initial position of turtlebot in gazebo world
 * @warning set up position will cause the exchange of x, y coordinate;
 * for example, you want to go to point (1,7); after set_up_position(), the coordinate
 * will be (7,1)
 */
void Walk::set_up_position() {
	// Wait for service gazebo/set_model_state
	ros::service::waitForService("gazebo/set_model_state");

	// call service to set up the initial position of turtlebot
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>\
			("gazebo/set_model_state");

	gazebo_msgs::SetModelState srv; // declare a service SetModelState

	// define model name; here turtlebot is named "mobile_base"
	srv.request.model_state.model_name = "mobile_base";

	// define the relative coordinate of model; here only has translation.
	srv.request.model_state.pose.position = position;

	// define the reference coordinate(reference frame) for transformation
	srv.request.model_state.reference_frame = "world";
	client.call(srv);	// call service

	// If service is called successfully send ROS_INFO; otherwise send ROS_ERROR
	if (srv.response.success) {
		ROS_INFO("Success to set up turtlebot position");
	} else {
		ROS_ERROR("Fail to set up turtlebot position");
	}
}

/**
 * @brief This function is to set up the goal point of turtlebot
 * @param x the coordinate in x-direction
 * @param y the coordinate in y-direction
 */
void Walk::set_up_goal(double x, double y) {
	goal.x = x;
	goal.y = y;
}

/**
 * @brief This function is an interface to obtain the position of goal
 * @return goal The goal position
 */
geometry_msgs::Point Walk::get_goal() {
	return goal;
}

/**
 * @brief the function is to get the linear distance between goal and current position
 * @return dist the distance between two points
 */
double Walk::diff_dist() {
	double dist = std::sqrt((goal.x-current_pose.x)*(goal.x-current_pose.x) + \
				(goal.y - current_pose.y)*(goal.y - current_pose.y));
	return dist;
}

/**
 * @brief This function is get the desired angle for turtlebot towards goal
 * @return angle the angle between two points
 */
double Walk::diff_angle() {
	double angle = std::atan2(goal.y-current_pose.y, goal.x-current_pose.x);
	desired_angle = angle; 		///< define the member in class
	return angle;
}


/**
 * @brief Check whether the current angle is desired angle
 * @param current_orientation This the orientation of turtlebot at current moment
 * @param angle The desired angle
 *
 */
bool Walk::isdiffAngle(tf::Quaternion current_orientation, double angle) {
	double Rc, Pc, Yc;	// Initialize the three variable for the angle in RPY
	tf::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc);	// transfer orientation from quaternion to RPY

	// if the difference between current angle and desired angle is under tolerance
	// return true, otherwise false
	if (std::abs(Yc - angle) < rotate_tolerance) {
		return false;
	} else {

		return true;
	}
}

/**
 * @brief This function is to check whether turtlebot need to rotate in reverse
 * to quickly go to the desired angle
 * @param current_orientation of turtlebot
 */
bool Walk::whether_reverse(tf::Quaternion current_orientation) {
	double Rc2, Pc2, Yc2;
	tf::Matrix3x3(current_orientation).getRPY(Rc2, Pc2, Yc2);

	ROS_INFO("Yc = %f", Yc2*57.3);
		ROS_INFO("desired_angle = %f", desired_angle*57.3);
		ROS_INFO("angular velo = %f", angular_velo.angular.z);

	// conditions for whether reverse angular velocity
	if (desired_angle > 0 && Yc2 > 0 && (Yc2 - desired_angle) > 0) {
		return true;
		ROS_INFO("!!reverse angular velocity!!");
	} else if (desired_angle < 0 && Yc2 < 0 && ((Yc2+6.28) - (desired_angle+6.28)) > 0) {
		return true;
		ROS_INFO("!!reverse angular velocity!!");
	} else if (desired_angle < 0 && Yc2 > 0 && (Yc2 - desired_angle) < 3.14) {
		return true;
		ROS_INFO("!!reverse angular velocity!!");
	} else if (desired_angle > 0 && Yc2 < 0 && ((Yc2+6.28)-desired_angle) < 3.14) {
		return true;
		ROS_INFO("!!reverse angular velocity!!");
	} else {
		return false;
	}
}



/**
 * @brief The function that moves turtlebot forward and rotate turtlebot once hitting obstacles
 * @param x The coordinate in x-direction
 * @param y The coordinate in y-direction
 */
bool Walk::linear_move(double x, double y) {

	// wait for topic /mobile_base/commands/velocity start
	auto message = ros::topic::waitForMessage<geometry_msgs::Twist>("/mobile_base/commands/velocity",ros::Duration(5));

	// publisher to publish velocity for turtlebot
	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	// subscriber to listen to topic /mobile_base/events/bumper
	ros::Subscriber bumper = n.subscribe("/mobile_base/events/bumper", 1000, &Walk::collision, this);

	ros::Rate loop_rate(10);		// rate of publishing is 1 Hz

	// set up the goal point
	set_up_goal(x, y);

	// Listen to tf
	tf::TransformListener listener;

	// distance between current position and desire position
	double dist = 1000;

	// current angular velocity
	geometry_msgs::Twist current_angular = angular_velo;

	while (ros::ok() && (dist > straight_tolerance)) {

		// get the tf between odom and base_footprint
		tf::StampedTransform transform;
	    listener.waitForTransform("/odom","/base_footprint",ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/odom","/base_footprint",ros::Time(0),transform);
		current_pose.x = transform.getOrigin().x();
		current_pose.y = transform.getOrigin().y();
		current_orientation = transform.getRotation();

		ROS_INFO("x = %f", current_pose.x);
		ROS_INFO("y = %f", current_pose.y);

		// get desired angle
		double angle_to_rotate = diff_angle();
		// check whether the turtlebot is in the desired orientation
		bool isAngleDiff = isdiffAngle(current_orientation, angle_to_rotate);
		// if current orientation is different to desired angle then rotate
		if (isAngleDiff) {
			bool reverse = false;
			reverse = whether_reverse(current_orientation); // check whether need reverse

			current_angular = reset_current_angular(current_angular);	// reset angular to positive

			if (reverse) {

				current_angular.angular.z = -angular_velo.angular.z;
			}
			move_pub.publish(current_angular);	// publish rotate command

			// check whether the turtlebot reverses its angular velocity
			if (reverse) {
				ROS_INFO("need reverse !!!!");
				ROS_INFO("current_angluar = %f", current_angular.angular.z);
			} else {
				ROS_INFO("NO reverse !!!");
				ROS_INFO("current_angluar = %f", current_angular.angular.z);
			}

		} else {
			move_pub.publish(linear_velo);	// publish linear movement command
		}


	    double dist = diff_dist();       // find the distance from current position to goal
		ROS_INFO("dist = %f", dist);
		ros::spinOnce();
		loop_rate.sleep();

		// stop move once reach the goal
		if (dist < straight_tolerance) {

			break;
		}

		// stop once obstacle
		if (need_turn) {
			ROS_WARN_STREAM("TurtleBot find obstacle at ( " << current_pose.x << " , " << current_pose.y);
			break;
		}

	}

	// check distance from current point to goal
	double dist_to_goal = diff_dist();

	if (dist_to_goal < straight_tolerance) {
		return true;
	} else {
		return false;
	}

}

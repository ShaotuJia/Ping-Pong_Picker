/*
 * @file walker.cpp
 * @brief This file implements the class members for simple walker algorithm
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
 * @brief This function is the interface to set up the initial position of turtlebot
 * @param x The position in x - direction
 * @param y The position in y - direction
 */
void Walk::set_initial_pose(const double& x, const double& y) {
	position.x = x;
	position.y = y;
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
 * @brief This function is to set up the work time for each move; once time reaches the
 * work time limit, the turtlebot will stop;
 * @param time This is the time limit of moving
 */
void Walk::set_up_worktime(int time) {
	work_time = time;
}

/**
 * @brief This function is to get the position of current turtlebot
 * @return pose
 */
geometry_msgs::Point Walk::get_current_pose() {
	return current_pose;
}

/**
 * @brief This function is to get the orientation of current turtlebot
 * @return orientation
 */
tf::Quaternion Walk::get_current_orientation() {
	return current_orientation;
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
 * @brief This function is check whether the turtlebot towards desired angle
 * @return bool
 */
bool Walk::isSameOrient(tf::Quaternion current_orientation, \
		tf::Quaternion desired_orientation) {
	double Rc, Pc, Yc;
	tf::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc);

	double Rd, Pd, Yd;
	tf::Matrix3x3(desired_orientation).getRPY(Rd, Pd, Yd);

	if (std::abs(Yc -Yd) < rotate_tolerance) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Check whether the current angle is desired angle
 */
bool Walk::isdiffAngle(tf::Quaternion current_orientation, double angle) {
	double Rc, Pc, Yc;
	tf::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc);

	// inverse rotation direction
	//Yc = -Yc;

	ROS_INFO("Yc = %f", Yc);
	ROS_INFO("desired_angle = %f", desired_angle);
	ROS_INFO("angle = %f", angle);
	ROS_INFO("angular velo = %f", angular_velo.angular.z);

	if (std::abs(Yc - angle) < rotate_tolerance) {
		return false;
	} else {

		return true;
	}
}

/**
 * @brief This function is to check whether turtlebot need to rotate in reverse
 * to quickly go to the desired angle
 */
void Walk::whether_reverse(tf::Quaternion current_orientation) {
	double Rc, Pc, Yc;
	tf::Matrix3x3(current_orientation).getRPY(Rc, Pc, Yc);

	// inverse rotation direction
	//Yc = -Yc;
	// condition is true, reverse angular velocity direction
	if (desired_angle > Yc) {
		angular_velo.angular.z = -angular_velo.angular.z;
	}

	ROS_INFO("angular_velo %f", angular_velo.angular.z);



}


/**
 * @brief This function is find the location of turtlebot by listening tf
 * @return current position of turtlebot
 */
void Walk::where_turtle() {
	tf::TransformListener listener;
	ros::Rate rate(10);
	while(ros::ok()) {
		tf::StampedTransform transform;
	    listener.waitForTransform("/odom","/base_link",ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/odom","/base_link",ros::Time(0),transform);
		current_pose.x = transform.getOrigin().x();
		current_pose.y = transform.getOrigin().y();
		current_orientation = transform.getRotation();
		rate.sleep();
	}
}
/**
 * @brief The function that moves turtlebot forward and rotate turtlebot once hitting obstacles
 * @param x The coordinate in x-direction
 * @param y The coordinate in y-direction
 */
bool Walk::linear_move(double x, double y) {

	// check whether turtlebot towards to the goal

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


	// origin of each time move
	double move_origin_x = current_pose.x;
	double move_origin_y = current_pose.y;

	while (ros::ok() && (dist > straight_tolerance)) {

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
			//whether_reverse(current_orientation);	// check whether need reverse
			//bool isTowards = rotate(angle_to_rotate);
			move_pub.publish(angular_velo);	// publish rotate command
		} else {
			move_pub.publish(linear_velo);	// publish linear movement command
		}

		double dist = diff_dist();
		ROS_INFO("dist = %f", dist);
		ros::spinOnce();
		loop_rate.sleep();

		// stop move once reach the goal
		if (dist < straight_tolerance) {
			break;
		}

	}

}

#if 0
/**
 * @brief This function let turtlebot rotate to a desired angle
 * @param angle The desired angle in Radians
 */
bool Walk::rotate(double angle) {
	// publisher to publish velocity for turtlebot
	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	// subscriber to listen to topic /mobile_base/events/bumper
	ros::Subscriber bumper = n.subscribe("/mobile_base/events/bumper", 1000, &Walk::collision, this);

	// listen to tf
	tf::TransformListener listener;

	// convert Yaw angle in unit radian to Quaternion
	//tf::Quaternion angle_Q = tf::createQuaternionFromYaw(angle);

	//ROS_INFO("quaternion %f",angle_Q.getW());

	ros::Rate loop_rate(20);		// rate of publishing is 1 Hz

	//where_turtle();

	bool isdiff = true;

	while (ros::ok() && isdiff) {
		tf::StampedTransform transform;
	    listener.waitForTransform("/base_footprint", "/odom",ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/base_footprint","/odom",ros::Time(0),transform);
		current_pose.x = transform.getOrigin().x();
		current_pose.y = transform.getOrigin().y();
		current_orientation = transform.getRotation();



		// check whether the turtlebot is in the desired orientation
		isdiff = isdiffAngle(current_orientation, angle);

		move_pub.publish(angular_velo);	// publish rotate command
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

#endif

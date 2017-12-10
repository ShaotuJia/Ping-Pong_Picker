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
 * @brief This function is to set up the work time for each move; once time reaches the
 * work time limit, the turtlebot will stop;
 * @param time This is the time limit of moving
 */
void Walk::set_up_worktime(int time) {
	work_time = time;
}

/**
 * @brief This function is find the location of turtlebot by listening tf
 * @return current position of turtlebot
 */
void where_turtle() {
	tf::TransformListener listener;
	ros::Rate rate(10);
	while(ros::ok()) {
		tf::StampedTransform transform;
		listener.lookupTransform("/base_footprint","/odom",ros::Time(0),transform);
		ROS_INFO("x = %f", transform.getOrigin().x());
		rate.sleep();
	}

}
/**
 * @brief The function that moves turtlebot forward and rotate turtlebot once hitting obstacles
 */
void Walk::linear_move(double time_limit) {

	// publisher to publish velocity for turtlebot
	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	// subscriber to listen to topic /mobile_base/events/bumper
	ros::Subscriber bumper = n.subscribe("/mobile_base/events/bumper", 1000, &Walk::collision, this);
	//tf::StampedTransform position = where_turtle();
	ros::Rate loop_rate(10);		// rate of publishing is 1 Hz
	int counter = 0;	// check whether this move reaches the time limit
	while (ros::ok() && ((counter/10)<time_limit)) {

		move_pub.publish(linear_velo);	// publish linear movement command
		ros::spinOnce();
		loop_rate.sleep();
		counter ++;

	}

}

void Walk::rotate(double angle) {
	// publisher to publish velocity for turtlebot
	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	// subscriber to listen to topic /mobile_base/events/bumper
	ros::Subscriber bumper = n.subscribe("/mobile_base/events/bumper", 1000, &Walk::collision, this);

	ros::Rate loop_rate(10);		// rate of publishing is 1 Hz

	double counter = 0;	// check whether this move reaches the time limit
	while (ros::ok() && (counter < angle)) {

		move_pub.publish(angular_velo);	// publish rotate command
		ros::spinOnce();
		loop_rate.sleep();
		counter += 0.1;
	}

}

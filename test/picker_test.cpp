/*
 * @file picker_test.cpp
 * @brief This rostest is to test functions in ping_pong_pikcer package
 * @author Shaotu Jia
 * @copyright Copyright (C) 2007 Free Software Foundation, Inc.
 * @details GNU GENERAL PUBLIC LICENSE. Version 3, 29 June 2007
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */
#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/SetModelState.h>
#include <memory>
#include "turtlebot_walker/walker.hpp"
#include "turtlebot_walker/mode.hpp"

// initialize a shared_ptr for NodeHandle for each test unit.
std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief Test the set up of initial turtlebot position
 */
TEST(TEST_Walk, initial_pose) {
	Walk walker;
	walker.set_initial_pose(0,0);	///< set up initial pose for turtlebot
	geometry_msgs::Point initial_pose = walker.get_initial_pose();
	EXPECT_EQ(0, initial_pose.x);

}

/**
 * @brief Test the set up of the goal which turtlebot will move to
 */
TEST(Test_Walk, goal_set_up) {
	Walk walker;
	int goal_x = 10;
	int goal_y = 10;
	walker.set_up_goal(goal_x, goal_y);
	geometry_msgs::Point target = walker.get_goal();
	EXPECT_EQ(target.x, goal_x);
	EXPECT_EQ(target.y, goal_y);
}

/**
 * @brief Test the set up of linear and angular velocity
 */
TEST(Test_Walk, linear_angular_velocity) {
	Walk walker;
	double linear_velocity = 0.2;
	double angular_velocity = 0.1;
	walker.set_linear(linear_velocity);
	walker.set_angular(angular_velocity);
	geometry_msgs::Twist turtle_linear = walker.get_linear_velo();
	geometry_msgs::Twist turtle_angular = walker.get_angular_velo();

	EXPECT_EQ(linear_velocity, turtle_linear.linear.x);
	EXPECT_EQ(angular_velocity, turtle_angular.angular.z);

}

/**
 * @brief Test the set up of tolerance
 */
TEST(Test_Walk, staright_rotate_tolerance) {
	Walk walker;
	double straight = 2;
	double rotate = 0.1;
	walker.set_straight_tolerance(straight);
	walker.set_rotate_tolerance(rotate);
	double straight_result = walker.get_straight_tolerance();
	double rotate_result = walker.get_rotate_tolerance();

	EXPECT_EQ(straight, straight_result);
	EXPECT_EQ(rotate, rotate_result);
}

/**
 * @brief This is to test the function set flag in Mode class
 */
TEST(Test_Mode, set_flag) {

	Mode mode;
	double x = 1;
	double y = 2;
	geometry_msgs::Point flag = mode.set_flag(x, y);
	EXPECT_EQ(x, flag.x);
	EXPECT_EQ(y, flag.y);

}

/**
 * @brief This is to test the go_route function
 */
TEST(Test_Movement, ALL_Move_functions) {
	Mode mode;
	Walk walker;
	walker.set_initial_pose(0, 0);	///< set up initial pose for turtlebot
	walker.set_up_position();		///< set up the initial position
	geometry_msgs::Point flag_1 = mode.set_flag(7,1); 	///< set flag 1;

	std::vector<geometry_msgs::Point> flag_seq = {flag_1};		///< set up this route
	auto finish = mode.go_route(flag_seq);		///< go this route
	EXPECT_FALSE(finish);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "picker_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

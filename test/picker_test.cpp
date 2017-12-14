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


// initialize a shared_ptr for NodeHandle for each test unit.
std::shared_ptr<ros::NodeHandle> nh;


TEST(TEST_initial_pose, initial_pose) {
	Walk walker;
	walker.set_initial_pose(0,0);	///< set up initial pose for turtlebot
	geometry_msgs::Point initial_pose = walker.get_initial_pose();
	EXPECT_EQ(0, initial_pose.x);

}

TEST(Test_Goal, goal) {
	Walk walker;
	int goal_x = 10;
	int goal_y = 10;
	walker.set_up_goal(goal_x, goal_y);
	geometry_msgs::Point target = walker.get_goal();
	EXPECT_EQ(target.x, goal_x);
	EXPECT_EQ(target.y, goal_y);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "picker_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

void armVerticalCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
	  ros::AsyncSpinner spinner(2);
	  spinner.start();

	  moveit::planning_interface::MoveGroupInterface arm("arm");

	  //motion planning
	  arm.setPoseReferenceFrame("base_link");

	  arm.setGoalOrientationTolerance(0.1); //double tolerance
	  arm.setGoalPositionTolerance(0.05); //double tolerance
	  arm.setNumPlanningAttempts(10); //unsigned int num_planning_attempts
	  arm.setPlanningTime(10.0); //double seconds

	  arm.setNamedTarget("vertical");
	  if (!arm.move()) {
	    ROS_WARN("Could not move to prepare pose");
	  }
  }
}

void armFrontCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
	  ros::AsyncSpinner spinner(2);
	  spinner.start();

	  moveit::planning_interface::MoveGroupInterface arm("arm");

	  //motion planning
	  arm.setPoseReferenceFrame("base_link");

	  arm.setGoalOrientationTolerance(0.1); //double tolerance
	  arm.setGoalPositionTolerance(0.2); //double tolerance
	  arm.setNumPlanningAttempts(10); //unsigned int num_planning_attempts
	  arm.setPlanningTime(10.0); //double seconds

	  ROS_INFO("Moving to prepare pose");
	  geometry_msgs::PoseStamped pose;
	  pose.header.frame_id = "base_link";
	  pose.pose.position.x = 0.1;
	  pose.pose.position.y = 0.0;
	  pose.pose.position.z = 0.2;
	  pose.pose.orientation.x = 0.0;
	  pose.pose.orientation.y = 0.707106;
	  pose.pose.orientation.z = 0.0;
	  pose.pose.orientation.w = 0.707106;

	  arm.setPoseTarget(pose);
	  if (!arm.move()) {
	    ROS_WARN("Could not move to prepare pose");
	  }
  }
}

void gripperOpenCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
	  ros::AsyncSpinner spinner(2);
	  spinner.start();

	  //gripper
	  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
	      "/crane_plus_gripper/gripper_command",
	      "true");
	  gripper.waitForServer();

	  ROS_INFO("Opening gripper");
	  control_msgs::GripperCommandGoal goal;
	  goal.command.position = 0.1;
	  gripper.sendGoal(goal);
	  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
	  if (!finishedBeforeTimeout) {
	    ROS_WARN("Gripper open action did not complete");
	  }
  }
}

void gripperCloseCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
	  ros::AsyncSpinner spinner(2);
	  spinner.start();

	  //gripper
	  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
	      "/crane_plus_gripper/gripper_command",
	      "true");
	  gripper.waitForServer();

	  ROS_INFO("Closing gripper");
	  control_msgs::GripperCommandGoal goal;
	  goal.command.position = 0.015;
	  gripper.sendGoal(goal);
	  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
	  if (!finishedBeforeTimeout) {
	    ROS_WARN("Gripper close action did not complete");
	  }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_manipulation");

  ros::NodeHandle n, n1, n2, n3;

  ros::Subscriber sub = n.subscribe("/arm_move_vertical", 1, armVerticalCallback);
  ros::Subscriber sub1 = n1.subscribe("/arm_move_front", 1, armFrontCallback);
  ros::Subscriber sub2 = n2.subscribe("/arm_gripper_open", 1, gripperOpenCallback);
  ros::Subscriber sub3 = n3.subscribe("/arm_gripper_close", 1, gripperCloseCallback);

  ros::spin();

  return 0;
}


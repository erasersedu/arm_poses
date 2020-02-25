#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("arm");

  //motion planning
  arm.setPoseReferenceFrame("base_link");

  arm.setGoalOrientationTolerance(0.1); //double tolerance
  arm.setGoalPositionTolerance(0.05); //double tolerance
  arm.setNumPlanningAttempts(10); //unsigned int num_planning_attempts
  arm.setPlanningTime(10.0); //double seconds

  //gripper
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
      "/crane_plus_gripper/gripper_command",
      "true");
  gripper.waitForServer();

  arm.setNamedTarget("vertical");
  if (!arm.move()) {
    ROS_WARN("Could not move to prepare pose");
    return 1;
  }

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
    return 1;
  }

  ROS_INFO("Opening gripper");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.1;
  gripper.sendGoal(goal);
  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper open action did not complete");
    return 1;
  }

  ROS_INFO("Closing gripper");
  goal.command.position = 0.015;
  gripper.sendGoal(goal);
  gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper close action did not complete");
    return 1;
  }

  ros::shutdown();
  return 0;
}

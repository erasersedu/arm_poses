#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

void armPoseCallback(const std_msgs::Bool::ConstPtr& msg)
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

	  //gripper
	  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
	      "/crane_plus_gripper/gripper_command",
	      "true");
	  gripper.waitForServer();

	  arm.setNamedTarget("vertical");
	  if (!arm.move()) {
	    ROS_WARN("Could not move to prepare pose");
	  }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_manipulation");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/move_arm2pose", 1, armPoseCallback);

  ros::spin();

  return 0;
}


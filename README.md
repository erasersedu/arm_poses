# arm_poses


From: https://gbiggs.github.io/ros_moveit_rsj_tutorial/manipulators_and_moveit.html . Follow the diagrams there to build the same arm configuration but don't install anything yet; after you had build your arm, follow the installation instructions given bellow:


1. Install MoveIt:

Ubuntu 16.04:

> sudo apt-get install ros-kinetic-moveit-*


Ubuntu 18.04

> sudo apt-get install ros-melodic-moveit-*


2. Download owr ThirdParty repository. Follow the instructions in:

https://github.com/erasersedu/third_party

3. Download this repository

> cd

> mkdir -p ~/erasersedu_ws/src/

> cd ~/erasersedu_ws/src

> git clone https://github.com/erasersedu/arm_poses.git

> cd ~/erasersedu_ws

> catkin_make

> source devel/setup.bash

*

# To Use It

> roslaunch arm_poses arm_poses.launch

If you see any motor in red, restart the previous command.


Then, in other terminal/node:

> rostopic pub /arm_move_vertical Bool "data:true"

> rostopic pub /arm_move_front Bool "data:true"

> rostopic pub /arm_gripper_open Bool "data:true"

> rostopic pub /arm_gripper_close Bool "data:true"

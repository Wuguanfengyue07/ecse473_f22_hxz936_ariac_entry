# ecse473_f22_hxz936_ariac_entry

# Overview

The program's primary objective is to control a robotic arm to perform various tasks like picking, placing, moving 

objects, and interacting with AGVs (Automated Guided Vehicles) and sensors. It subscribes to different ROS topics to 

receive data from sensors and publishes commands to control the arm and the gripper.

# Key Classes and Methods

Arm Class: Manages the arm's movements and gripper control.
  
  go_to_joint_state(ArmJointState, ros::Duration): Moves the arm to a specific joint state.
  
  move_linear_actuator(double): Moves the linear actuator to a specified position.
  
  pickup_part(geometry_msgs::Point, ...): Picks up a part from a specified location.
  
  rotate_end_effector(double): Rotates the end effector to a specified angle.
  
  set_vacuum_enable(bool): Enables or disables the vacuum gripper.

subscribeToBinTopic and subscribeToTopic Functions: Subscribes to camera topics for bin, AGV, and quality sensor data.

Callbacks: Handles incoming data from various ROS topics, including joint states and gripper states.

# Features

Arm Control: Ability to move the robotic arm to specific joint states or positions.

Linear Actuator Movement: Control of the linear actuator for vertical movement.

Vacuum Gripper Control: Enable or disable the vacuum gripper to pick and place objects.

Camera Integration: Subscribes to logical camera topics for detecting objects and their locations.

AGV Interaction: Communicate with AGVs to control their actions.

Shipment Handling: Manage incoming shipment orders and execute tasks accordingly.

# How to Run

Ensure that the ARIAC environment and ROS are set up correctly.

Clone the project into your workspace.

Build the project using the "catkin_make" command.

Run the respective ROS nodes to start the system.

1."roscore &"

2.->"Roslaunch ecse_373_ariac ecse_373_ariac.launch"
  
  ->"rosrun ariac_entry entry.cpp"(should use in new terminal)
  
  Or use the "roslaunch ariac_entry ariac.launch" command, which has the same effect  

# Dependencies

ROS Noetic.

ARIAC environment set up

C++11 compiler




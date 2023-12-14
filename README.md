# ecse473_f22_hxz936_ariac_entry

# Some Module Descriptions

1. Order Processing Module
The processOrder function is responsible for handling incoming orders, extracting the product type, and requesting services to determine the product's location
Input: Order of type Order.
Output: The storage unit and camera frame where the product is located.

2. Calculating Target Position Module
The calculateGoalPose function calculates the target pose of the robotic arm based on the provided camera frame and logical camera images.
Input: Camera frame of the storage unit.
Output: Target pose of the robotic arm (geometry_msgs::PoseStamped type).

3. Setting and Publishing Joint Trajectory Module
The setAndPublishJointTrajectory function sets the joint trajectory for the robotic arm and publishes it to a ROS topic.
Input: Target pose and current joint states.
Output: Joint trajectory published to a ROS topic.

# Usage

Ensure your ROS environment is correctly set up.

Clone the project into your workspace.

Build the project using the catkin_make command.

Run the respective ROS nodes to start the system.

"roscore &"

->"Roslaunch ecse_373_ariac ecse_373_ariac.launch"

->"rosrun ariac_entry entry.cpp"(should use in new terminal)

# Dependencies

ROS Noetic.

Must use with package ecse_373_ariac and package ik_service, these packages should be contained in same workspace.


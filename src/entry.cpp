#include <ros/console.h>
#include <ros/ros.h>
#include <array>
#include <string>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/AGVControl.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/SubmitShipment.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <array>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <boost/thread.hpp>
#include <cmath>
#include <sstream>
#include <unordered_map>


using osrf_gear::LogicalCameraImage;
using ArmJointState = std::array<double, 7>;

class Arm {
public:
  Arm(const std::string &name);
  ~Arm() = default;

  bool go_to_joint_state(ArmJointState joint_state,
                         ros::Duration duration = ros::Duration(1.0));

  bool move_linear_actuator(double position);
  bool move_linear_actuator_relative(double position);
  bool move_arm(ArmJointState joint_state);

  bool go_to_local_pose(geometry_msgs::Point point);
  bool go_to_home_pose();

  bool pickup_part(geometry_msgs::Point point,
                   geometry_msgs::Point camera_point, double rotation = 0.0,
                   bool left = true, bool agv = false, bool pickup = true);

  bool rotate_end_effector(double rotation);

  bool set_vacuum_enable(bool enable);

private:
  void joint_state_callback(
      const boost::shared_ptr<sensor_msgs::JointState const> &msg);

  void gripper_state_callback(
      const boost::shared_ptr<osrf_gear::VacuumGripperState const> &msg);

  geometry_msgs::Pose joint_state_to_pose(ArmJointState joint_state);

private:
  std::string m_name;

  ros::NodeHandle m_nh;

  // state orderings
  std::vector<std::string> m_urk_ordering;

  // current state
  ArmJointState m_current_joint_state;
  geometry_msgs::PoseStamped m_current_pose_local;
  osrf_gear::VacuumGripperState m_current_gripper_state;

  // subscribers
  ros::Subscriber m_joint_state_sub;
  ros::Subscriber m_gripper_sub;

  // publishers
  ros::Publisher m_trajectory_pub;

  // services
  ros::ServiceClient m_gripper_client;

  // action server
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      m_trajectory_as;
};

template <size_t N>
using LogicalCameraArray = std::array<LogicalCameraImage, N>;

using LogicalCameraPtr = const boost::shared_ptr<
    const osrf_gear::LogicalCameraImage_<std::allocator<void>>>;

using ImageCallback = boost::function<void(LogicalCameraPtr)>;

constexpr int k_buffer_size = 16;

tf2_ros::Buffer g_tf_buf;

// global data
std::queue<osrf_gear::Shipment> g_shipment_queue;
std::string g_agv1_state;
std::string g_agv2_state;

LogicalCameraArray<6> g_bin_images;
LogicalCameraArray<2> g_agv_images;
LogicalCameraArray<2> g_quality_images;


void adjust_pose(geometry_msgs::PoseStamped &pose) {
  pose.pose.position.z += 0.1;

  pose.pose.orientation.w = 0.707;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.z = 0.0;
}

void print_pose(const geometry_msgs::Pose &pose) {
  ROS_DEBUG("%0f %0f %0f", pose.position.x, pose.position.y, pose.position.z);
}

geometry_msgs::TransformStamped
get_robot_to_frame(const std::string &to_frame) {
  geometry_msgs::TransformStamped tf;
  try {
    tf = g_tf_buf.lookupTransform("arm1_base_link", to_frame, ros::Time(0.0),
                                  ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Transform fetch error: %s", ex.what());
  }
  return tf;
}

double yaw_from_pose(const geometry_msgs::Pose &pose) {
  // Extract yaw angle from the quaternion orientation
  double yaw = atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

  ROS_WARN("Yaw for pose: %f", yaw);

  return yaw;
}



void order_callback(const osrf_gear::Order &order) {
  static int n = 0;
  ROS_INFO("Recieved order %d", n++);

  for (const auto &s : order.shipments) {
    // loop over every product in order
    g_shipment_queue.push(s);
    ROS_INFO("Adding shipment to queue");
  }
}


void agv1_callback(const std_msgs::StringConstPtr &msg) {
  g_agv1_state = msg->data;
}

void agv2_callback(const std_msgs::StringConstPtr &msg) {
  g_agv2_state = msg->data;
}


void subscribeToBinTopic(
    ros::NodeHandle& nh, 
    int binNumber, 
    std::array<ros::Subscriber, 6>& subscribers, 
    std::array<LogicalCameraImage, 6>& images
) {
    const std::string topic = "/ariac/logical_camera_bin" + std::to_string(binNumber);
    const ImageCallback callback = [&, binNumber](LogicalCameraPtr img) {
        images[binNumber - 1] = *img;
    };

    subscribers[binNumber - 1] = nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 16, callback);
}


void subscribeToTopic(ros::NodeHandle& nh, const std::string& topic, int index, 
    std::array<ros::Subscriber, 2>& subscribers, std::array<LogicalCameraImage, 2>& images) {
    const ImageCallback callback = [&, index](LogicalCameraPtr img) {
        images[index] = *img;
    };
    subscribers[index] = nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 16, callback);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_node");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();

  tf2_ros::TransformListener tfListener(g_tf_buf);

  auto material_location_client =
      nh.serviceClient<osrf_gear::GetMaterialLocations>(
          "/ariac/material_locations");

  material_location_client.waitForExistence(ros::Duration(-1.0));

  auto agv1_client = nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  auto agv2_client = nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  agv1_client.waitForExistence(ros::Duration(-1.0));
  agv2_client.waitForExistence(ros::Duration(-1.0));

  auto shipment_submision_client =
      nh.serviceClient<osrf_gear::SubmitShipment>("/ariac/submit_shipment");
  shipment_submision_client.waitForExistence();

  Arm arm("arm1");

  auto order_sub = nh.subscribe("/ariac/orders", 2, order_callback);

  auto agv1_state_sub = nh.subscribe("/ariac/agv1/state", 32, agv1_callback);
  auto agv2_state_sub = nh.subscribe("/ariac/agv2/state", 32, agv2_callback);


  std::array<ros::Subscriber, 6> bin_camera_subs;
  std::array<ros::Subscriber, 2> agv_camera_subs;
  std::array<ros::Subscriber, 2> quality_camera_subs;

  ROS_INFO("Subscribing to bin cameras");

  for (int i = 1; i <= 6; ++i) {
    subscribeToBinTopic(nh, i, bin_camera_subs, g_bin_images);
  }

  ROS_INFO("Subscribing to agv and  quality cameras");

  for (int i = 1; i <= 2; ++i) {
      std::string agv_topic = "/ariac/logical_camera_agv" + std::to_string(i);
      subscribeToTopic(nh, agv_topic, i - 1, agv_camera_subs, g_agv_images);
  }

  for (int i = 1; i <= 2; ++i) {
      std::string quality_topic = "/ariac/quality_control_sensor_" + std::to_string(i);
      subscribeToTopic(nh, quality_topic, i - 1, quality_camera_subs, g_quality_images);
  }



  ros::ServiceClient begin_client =
      nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  begin_client.waitForExistence();

  std_srvs::Trigger begin_comp;

  const bool send_success = begin_client.call(begin_comp);

  if (!send_success) {
    ROS_ERROR("Competition service call failed!  Goodness Gracious!!");
    return 1;
  }

  const bool start_success = begin_comp.response.success;

  if (!start_success) {
    ROS_WARN("Competition service returned failure: %s",
             begin_comp.response.message.c_str());
  } else {
    ROS_INFO("Competition service called successfully: %s",
             begin_comp.response.message.c_str());
  }

  ROS_INFO("Before loop");

  arm.go_to_home_pose();
  ros::Duration(1.0).sleep();
  arm.rotate_end_effector(0.0);
  ros::Duration(1.0).sleep();

  ros::Rate r(10);
  while (ros::ok()) {

    if (g_shipment_queue.empty()) {
      r.sleep();
      continue;
    }

    ROS_INFO("Putting together shipment");

    auto current_shipment = g_shipment_queue.front();
    g_shipment_queue.pop();

    const std::string shipment_type = current_shipment.shipment_type;
    const std::string agv_id = current_shipment.agv_id;

    double agv_lin;
    int agv_num;
    std::string agv_camera_frame;

    if (agv_id == "agv1") {
      agv_lin = 2.25;
      agv_num = 1;
      agv_camera_frame = "logical_camera_agv1_frame";
    } else {
      agv_lin = -2.25;
      agv_num = 2;
      agv_camera_frame = "logical_camera_agv2_frame";
    }

    for (const auto &part : current_shipment.products) {
      std::string product_type = part.type;

      osrf_gear::GetMaterialLocations material_loc;
      material_loc.request.material_type = product_type;
      if (!material_location_client.call(material_loc)) {
        ROS_ERROR("Could not call material location service");
        continue;
      }

      int bin_num;
      std::string unit_id;

      for (const osrf_gear::StorageUnit &su :
           material_loc.response.storage_units) {
        if (!std::strstr(su.unit_id.c_str(), "bin")) {
          continue;
        }
        char c = su.unit_id[3];
        bin_num = (int(c) - int('0')) - 1;
        unit_id = su.unit_id;
        break;
      }
      const LogicalCameraImage &img = g_bin_images[bin_num];
      const std::string camera_frame = "logical_camera_" + unit_id + "_frame";

      geometry_msgs::Pose camera_pose, part_pose, blank_pose;
      geometry_msgs::Pose offset_pose, goal_pose;

      goal_pose.position.x = 0;
      goal_pose.position.y = 0;
      goal_pose.position.z = 0;

      camera_pose = img.pose;
      part_pose = img.models.front().pose;



      auto tf = get_robot_to_frame(camera_frame);
      tf2::doTransform(blank_pose, offset_pose, tf);

      bool left = bin_num > 2;

      const double linear_offset = 0.6;
      if (left) {
        arm.move_linear_actuator_relative(offset_pose.position.y -
                                          linear_offset);
      } else {
        arm.move_linear_actuator_relative(offset_pose.position.y +
                                          linear_offset);
      }

      ros::Duration(1.0).sleep();

      tf = get_robot_to_frame(camera_frame);

      tf2::doTransform(part_pose, goal_pose, tf);
      tf2::doTransform(blank_pose, camera_pose, tf);

      double begin_rotation = yaw_from_pose(part_pose);

      ROS_INFO("Picking up part with rotation: %f", begin_rotation);

      // pickup part
      arm.pickup_part(goal_pose.position, camera_pose.position, begin_rotation,
                      left, false, true);
      ros::Duration(1.0).sleep();

      // move to agv
      arm.move_linear_actuator(agv_lin);
      ros::Duration(1.0).sleep();

      if (agv_id == "agv1") {
        while (ros::ok() && g_agv1_state != "ready_to_deliver")
          ros::Duration(0.1).sleep();
      } else {
        while (ros::ok() && g_agv2_state != "ready_to_deliver")
          ros::Duration(0.1).sleep();
      }

      geometry_msgs::Pose agv_camera_pose, agv_part_pose;
      tf = get_robot_to_frame(agv_camera_frame);

      std::string tray_frame;
      {
        std::stringstream ss;
        ss << "kit_tray_";
        ss << agv_num;
        tray_frame = ss.str();
      }
      auto tray_tf = get_robot_to_frame(tray_frame);

      tf2::doTransform(blank_pose, agv_camera_pose, tf);
      tf2::doTransform(part.pose, agv_part_pose, tray_tf);

    
      agv_part_pose.position.z += 0.05;
 
      double end_rotation = yaw_from_pose(agv_part_pose);


      arm.pickup_part(agv_part_pose.position, agv_camera_pose.position,
                      end_rotation, agv_id != "agv1", true, false);

      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Done with shipment");

    osrf_gear::AGVControl submit;
    submit.request.shipment_type = shipment_type;

    if (agv_id == "agv1") {
      if (!agv1_client.call(submit)) {
        ROS_ERROR("AGV1 client could not be callled");
      } else if (!submit.response.success) {
        ROS_ERROR("Submission from AGV1 unsuccessful with message: %s",
                  submit.response.message.c_str());
      }
    } else {
      if (!agv2_client.call(submit)) {
        ROS_ERROR("AGV2 client could not be callled");
      } else if (!submit.response.success) {
        ROS_ERROR("Submission from AGV2 unsuccessful with message: %s",
                  submit.response.message.c_str());
      }
    }
  }

  return 0;
}



std::string pose_to_string(const geometry_msgs::Pose &pose) {
  std::stringstream ss;

  ss << "(";
  ss << pose.position.x;
  ss << ", ";
  ss << pose.position.y;
  ss << ", ";
  ss << pose.position.z;
  ss << ")";

  return ss.str();
}

Arm::Arm(const std::string &name)
    : m_nh(), m_name(name),
      m_trajectory_as("/ariac/" + name + "/arm/follow_joint_trajectory", true) {
  std::string base_topic("/ariac/" + name);
  std::string joint_state_topic = base_topic + "/joint_states";

  m_urk_ordering.reserve(7);
  m_urk_ordering.emplace_back("linear_arm_actuator_joint");
  m_urk_ordering.emplace_back("shoulder_pan_joint");
  m_urk_ordering.emplace_back("shoulder_lift_joint");
  m_urk_ordering.emplace_back("elbow_joint");
  m_urk_ordering.emplace_back("wrist_1_joint");
  m_urk_ordering.emplace_back("wrist_2_joint");
  m_urk_ordering.emplace_back("wrist_3_joint");

  m_trajectory_pub = m_nh.advertise<trajectory_msgs::JointTrajectory>(
      base_topic + "/arm/command", 2);

  m_joint_state_sub = m_nh.subscribe<sensor_msgs::JointState>(
      joint_state_topic, 32, &Arm::joint_state_callback, this);

  m_gripper_sub = m_nh.subscribe<osrf_gear::VacuumGripperState>(
      base_topic + "/gripper/state", 32, &Arm::gripper_state_callback, this);

  m_gripper_client = m_nh.serviceClient<osrf_gear::VacuumGripperControl>(
      base_topic + "/gripper/control");

  m_gripper_client.waitForExistence(ros::Duration(-1.0));

  ROS_INFO("Waiting for trajectory action server");
  m_trajectory_as.waitForServer();
  ROS_INFO("Trajectory server running");


}

bool Arm::set_vacuum_enable(bool enable) {
  osrf_gear::VacuumGripperControl srv;
  srv.request.enable = enable;

  m_gripper_client.call(srv);

  return srv.response.success;
}

void Arm::gripper_state_callback(
    const boost::shared_ptr<osrf_gear::VacuumGripperState const> &msg) {
  ROS_INFO_STREAM_THROTTLE(10.0, "Recieved grippper state. Enabled="
                                     << msg->enabled
                                     << ", Attached=" << msg->attached);
  m_current_gripper_state = *msg;
}

void Arm::joint_state_callback(
    const boost::shared_ptr<sensor_msgs::JointState const> &msg) {

  ArmJointState new_joint_state;


  std::unordered_map<std::string, int> name_to_index;
  for (int j = 0; j < msg->name.size(); ++j) {
      name_to_index[msg->name[j]] = j;
  }

  int count = 0;
  for (int i = 0; i < m_urk_ordering.size(); ++i) {
      auto it = name_to_index.find(m_urk_ordering[i]);
      if (it != name_to_index.end()) {
          new_joint_state[i] = msg->position[it->second];
          count++;
      }
  }


  if (count < m_urk_ordering.size()) {
    return;
  }

  m_current_joint_state = new_joint_state;

  {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < m_urk_ordering.size(); ++i) {
      if (i != 0)
        ss << ", ";
      ss << m_current_joint_state[i];
    }
    ss << "]";

    ROS_INFO_STREAM_THROTTLE(10.0, "Current joint state: " << ss.str());
  }



  double T[4][4];

  ur_kinematics::forward(m_current_joint_state.data() + 1, (double *)&T);

  m_current_pose_local.pose = joint_state_to_pose(m_current_joint_state);
  m_current_pose_local.header.stamp = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(10.0, "Current position (XYZ): " << pose_to_string(
                                     m_current_pose_local.pose));
}

bool Arm::go_to_joint_state(ArmJointState joint_state, ros::Duration duration) {
  static uint64_t count = 0;
  static uint64_t as_count = 0;


  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();

  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.header.frame_id = "/world";

  joint_trajectory.joint_names = m_urk_ordering;

  joint_trajectory.points.resize(2);

  const ros::Duration dur_offset(0.1);

  joint_trajectory.points[0].time_from_start = dur_offset;
  joint_trajectory.points[1].time_from_start = duration + dur_offset;

  for (int i = 0; i < m_urk_ordering.size(); ++i) {
    joint_trajectory.points[0].positions.push_back(m_current_joint_state[i]);
    joint_trajectory.points[1].positions.push_back(joint_state[i]);
  }


  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

  joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
  joint_trajectory_as.action_goal.header.seq = as_count++;
  joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
  joint_trajectory_as.action_goal.header.frame_id = "/world";

  joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
  joint_trajectory_as.action_goal.goal_id.id = std::to_string(as_count - 1);

  m_trajectory_as.sendGoal(joint_trajectory_as.action_goal.goal);
  bool finished_before_timeout =
      m_trajectory_as.waitForResult(ros::Duration(120.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = m_trajectory_as.getState();
    ROS_INFO("Trajectory finished with state %s with text: %s",
             state.toString().c_str(), state.getText().c_str());
  } else {
    ROS_WARN("Trajectory did not finish before timeout");
  }

  return true;
}

bool Arm::move_linear_actuator(double position) {
  ArmJointState joint_state;
  for (int i = 0; i < joint_state.size(); ++i) {
    joint_state[i] = m_current_joint_state[i];
  }
  joint_state[0] = position;

  const double duration_per_meter = 1.0;
  double dist = std::abs(position - m_current_joint_state[0]);

  if (dist < 0.01) {
    dist = 0.01;
  }

  ROS_INFO("Moving linear actuator to position %f (distance of %f meters) in "
           "%f seconds",
           position, dist, dist * duration_per_meter);

  ros::Duration duration(dist * duration_per_meter);

  return go_to_joint_state(joint_state, duration);
}

bool Arm::move_arm(ArmJointState joint_state) {
  // set linear actuator to what it was before
  joint_state[0] = m_current_joint_state[0];
  // set rotation of end effector to same as it was before
  joint_state[6] = m_current_joint_state[6];

  const auto current_pose = m_current_pose_local.pose;
  const auto goal_pose = joint_state_to_pose(joint_state);

  const double x = current_pose.position.x - goal_pose.position.x;
  const double y = current_pose.position.y - goal_pose.position.y;
  const double z = current_pose.position.z - goal_pose.position.z;

  double dist = std::sqrt(x * x + y * y + z * z);

  if (dist < 0.01) {
    dist = 0.5;
  }

  const double duration_per_distance = 1.5;

  ROS_INFO("Move arm %f meters in %f seconds.", dist,
           dist * duration_per_distance);

  ros::Duration duration(dist * duration_per_distance);

  return go_to_joint_state(joint_state, duration);
}

bool Arm::pickup_part(geometry_msgs::Point point,
                      geometry_msgs::Point camera_point, double rotation,
                      bool left, bool agv, bool pickup) {
  point.z += 0.02;

  geometry_msgs::Point hover, waypoint;
  hover = point;
  hover.z += 0.1;

  if (!agv) {
    waypoint.x = camera_point.x;
    if (left) {
      waypoint.y = camera_point.y - 0.4;
    } else {
      waypoint.y = camera_point.y + 0.4;
    }
  } else {
    waypoint.x = camera_point.x - 0.4;
    waypoint.y = camera_point.y;
  }
  waypoint.z = camera_point.z - 0.3;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  go_to_home_pose();
  ros::Duration(0.1).sleep();
  go_to_local_pose(waypoint);
  ros::Duration(0.1).sleep();
  go_to_local_pose(hover);

  rotate_end_effector(rotation);

  ros::Duration(0.1).sleep();

  if (pickup) {
    set_vacuum_enable(true);
  }
  ros::Duration(0.1).sleep();

  go_to_local_pose(point);

  if (!pickup) {
    // if dropping off part, disable vacuum
    set_vacuum_enable(false);
  }
  // wait for correct vacuum state
  ros::Duration(0.3).sleep();
  while (ros::ok() && m_current_gripper_state.attached != pickup)
    ros::Duration(0.1).sleep();

  go_to_local_pose(hover);
  ros::Duration(0.1).sleep();
  rotate_end_effector(0.0);
  go_to_local_pose(waypoint);
  ros::Duration(0.1).sleep();
  go_to_home_pose();
  ros::Duration(0.1).sleep();

  return true;
}

bool Arm::rotate_end_effector(double rotation) {
  ArmJointState joint_state;

  for (int i = 0; i < 6; ++i) {
    joint_state[i] = m_current_joint_state[i];
  }

  while (rotation > 2 * M_PI)
    rotation -= 2 * M_PI;
  while (rotation < 0)
    rotation += 2 * M_PI;

  ROS_INFO("Rotating end effector to angle: %0f radians", rotation);

  joint_state[6] = rotation;
  return go_to_joint_state(joint_state, ros::Duration(1.5));
}

bool Arm::move_linear_actuator_relative(double position) {
  // ArmJointState joint_state;
  const double abs_pos = m_current_joint_state[0] + position;
  return move_linear_actuator(abs_pos);
}

bool Arm::go_to_local_pose(geometry_msgs::Point point) {

  double T_des[4][4];
  double q_des[8][6];

  T_des[0][0] = 0.0;
  T_des[0][1] = -1.0;
  T_des[0][2] = 0.0;
  T_des[1][0] = 0.0;
  T_des[1][1] = 0.0;
  T_des[1][2] = 1.0;
  T_des[2][0] = -1.0;
  T_des[2][1] = 0.0;
  T_des[2][2] = 0.0;
  T_des[3][0] = 0.0;
  T_des[3][1] = 0.0;
  T_des[3][2] = 0.0;

  T_des[0][3] = point.x;
  T_des[1][3] = point.y;
  T_des[2][3] = point.z;
  T_des[3][3] = 1.0;

  const int num_sols =
      ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

  if (num_sols < 1) {
    ROS_ERROR("Not enough IK solutions");
    return false;
  }

  std::stringstream ss;
  ss << "\n";
  for (int i = 0; i < num_sols; ++i) {
    ss << "Solution #" << i << ": ";
    for (int j = 0; j < 6; ++j) {
      ss << q_des[i][j] << " ";
    }
    ss << "\n";
  }

  int sol_idx = 0;
  for (int i = 0; i < num_sols; ++i) {
    if (q_des[i][3] > M_PI && q_des[i][1] > M_PI && q_des[i][2] < M_PI) {
      sol_idx = i;
      break;
    }
  }

  ROS_INFO_STREAM("Choosing IK solution #" << sol_idx);

  ROS_INFO_STREAM_THROTTLE(60.0, "IK Joint state array: " << ss.str());

  ArmJointState joint_state;
  joint_state[0] = 0.0;
  for (int i = 0; i < 6; ++i) {
    joint_state[i + 1] = q_des[sol_idx][i];
  }

  return move_arm(joint_state);
}

geometry_msgs::Pose Arm::joint_state_to_pose(ArmJointState joint_state) {
  double T[4][4];

  ur_kinematics::forward(joint_state.data() + 1, (double *)&T);

  geometry_msgs::Pose pose;
  pose.position.x = T[0][3];
  pose.position.y = T[1][3];
  pose.position.z = T[2][3];

  return pose;
}

bool Arm::go_to_home_pose() {
  geometry_msgs::Point goal;
  goal.x = -0.3;
  goal.y = 0.2;
  goal.z = 0.3;
  return go_to_local_pose(goal);
}

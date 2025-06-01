#pragma once

#include "ismpc_interfaces/msg/sim_data.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <dart/dart.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

typedef std::chrono::high_resolution_clock Time;

namespace ismpc {
namespace ros {

/**
 * @brief ROS2 node that bridges DART physics simulation with ROS2 ecosystem
 *
 * Provides integration between DART (Dynamic Animation and Robotics Toolkit)
 * physics simulation and ROS2, enabling robot simulation and control.
 */

class DartBridgeNode : public rclcpp::Node {
public:
  DartBridgeNode();

private:
  rclcpp::TimerBase::SharedPtr simTimer;
  std::chrono::_V2::system_clock::time_point start;

  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr robot;

  // Publishers
  rclcpp::Publisher<ismpc_interfaces::msg::SimData>::SharedPtr sim_data_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Logger logger = rclcpp::get_logger("DartBridgeNode");

  /**
   * @brief Constructs and initializes a DART simulation world
   */
  void constructWorld();

  /**
   * @brief Updates the DART simulation world
   *
   * This method is called periodically to update the DART simulation state.
   * It is bound to a timer that triggers at a specified interval.
   */
  void simulationCallback();

  /**
   * @brief Publishes the current state of the robot from simulation
   *
   * This method publishes the state of the robot to a ROS topic.
   */
  void publishSimData();

  /**
   * @brief Publishes visualization markers for the robot
   *
   * This method publishes visualization markers to a ROS topic for debugging
   * and visualization purposes.
   */
  void publishMarkers();
  void publishWorldFrame();

  const std::unordered_map<std::string, double> INITIAL_CONFIG = {
      {"CHEST_P", 0.0},      {"CHEST_Y", 0.0},      {"NECK_P", 0.0},
      {"NECK_Y", 0.0},       {"R_HIP_Y", 0.0},      {"R_HIP_R", -3.0},
      {"R_HIP_P", -25.0},    {"R_KNEE_P", 50.0},    {"R_ANKLE_P", -25.0},
      {"R_ANKLE_R", 3.0},    {"L_HIP_Y", 0.0},      {"L_HIP_R", 3.0},
      {"L_HIP_P", -25.0},    {"L_KNEE_P", 50.0},    {"L_ANKLE_P", -25.0},
      {"L_ANKLE_R", -3.0},   {"R_SHOULDER_P", 4.0}, {"R_SHOULDER_R", -8.0},
      {"R_SHOULDER_Y", 0.0}, {"R_ELBOW_P", -25.0},  {"L_SHOULDER_P", 4.0},
      {"L_SHOULDER_R", 8.0}, {"L_SHOULDER_Y", 0.0}, {"L_ELBOW_P", -25.0}};
};

} // namespace ros
} // namespace ismpc
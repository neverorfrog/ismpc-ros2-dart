#pragma once

#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ismpc_ros_client/mpc_interface.h"

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
typedef std::chrono::milliseconds ms;

namespace ismpc {
namespace ros {

class RosWalkEngine : public rclcpp::Node {
public:
  RosWalkEngine();
  void update();

private:
  rclcpp::TimerBase::SharedPtr timer;
  std::chrono::_V2::system_clock::time_point start;
  MpcInterface mpc_interface_;

  // Logging
  rclcpp::Logger logger = rclcpp::get_logger("RosWalkEngine");
  void logState();
};

} // namespace ros
} // namespace ismpc

#pragma once

#include "ismpc_ros_client/mpc_interface.h"
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
typedef std::chrono::milliseconds ms;

namespace ismpc {
namespace ros {

class RosWalkEngine : public rclcpp::Node {
public:
  RosWalkEngine();

private:
  rclcpp::TimerBase::SharedPtr timer;
  std::chrono::_V2::system_clock::time_point start;
  MpcInterface mpc_interface_;

  /**
 @brief Called every delta seconds to update the MPC state.
 */
  void update();

  // Logging
  rclcpp::Logger logger = rclcpp::get_logger("RosWalkEngine");
  void logState();
};

} // namespace ros
} // namespace ismpc

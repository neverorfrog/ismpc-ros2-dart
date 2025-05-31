#include "ismpc_ros_client/node.h"

namespace ismpc {
namespace ros {

RosWalkEngine::RosWalkEngine() : Node("walk_engine"), mpc_interface_() {
  // Update callback executed every 15ms
  timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                  std::bind(&RosWalkEngine::update, this));
  start = Time::now();
}

void RosWalkEngine::update() {
  mpc_interface_.mpcStep();

  std::chrono::_V2::system_clock::time_point iteration_start = Time::now();
  RCLCPP_INFO(logger, "-------------------------");
  RCLCPP_INFO(logger, "Current Time: %f", mpc_interface_.getFrameInfo().tk);
  RCLCPP_INFO(logger, "Current Step: %d", mpc_interface_.getFrameInfo().k);
  RCLCPP_INFO(logger, "Reference Velocity: (%f, %f, %f)",
              mpc_interface_.getReference().velocity.vx,
              mpc_interface_.getReference().velocity.vy,
              mpc_interface_.getReference().velocity.omega);
  RCLCPP_INFO(logger, "Left Foot Position: (%f, %f, %f)",
              mpc_interface_.getState().left_foot.pose.translation.x(),
              mpc_interface_.getState().left_foot.pose.translation.y(),
              mpc_interface_.getState().left_foot.pose.translation.z());
  RCLCPP_INFO(logger, "Right Foot Position: (%f, %f, %f)",
              mpc_interface_.getState().right_foot.pose.translation.x(),
              mpc_interface_.getState().right_foot.pose.translation.y(),
              mpc_interface_.getState().right_foot.pose.translation.z());
  RCLCPP_INFO(logger, "-------------------------\n\n");
}

} // namespace ros
} // namespace ismpc

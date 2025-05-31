#include "ismpc_ros_client/node.h"

namespace ismpc {
namespace ros {

RosWalkEngine::RosWalkEngine() : Node("walk_engine"), mpc_interface_() {
  // Update callback executed every delta*1000 ms
  auto dt = std::chrono::duration<double>(mpc_interface_.getParams().mpc.delta);
  auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt);
  timer =
      this->create_wall_timer(dt_ms, std::bind(&RosWalkEngine::update, this));
  start = Time::now();
}

void RosWalkEngine::update() {
  mpc_interface_.step();

  std::chrono::_V2::system_clock::time_point iteration_start = Time::now();
  RCLCPP_INFO(logger, "-------------------------");
  RCLCPP_INFO(logger, "Current Time: %f", mpc_interface_.getFrameInfo().tk);
  RCLCPP_INFO(logger, "Current Step: %d", mpc_interface_.getFrameInfo().k);
  RCLCPP_INFO(logger, "Reference Velocity: (%f, %f, %f)",
              mpc_interface_.getReference().velocity.vx,
              mpc_interface_.getReference().velocity.vy,
              mpc_interface_.getReference().velocity.omega);
  RCLCPP_INFO(
      logger, "Support Phase: %s",
      mpc_interface_.getFootstepPlan().support_phase ==
              ismpc::SupportPhase::DOUBLE
          ? "Double"
      : mpc_interface_.getFootstepPlan().footsteps.front().support_foot ==
              ismpc::Foot::left
          ? "Single Left"
          : "Single Right");
  RCLCPP_INFO(logger, "Left Foot Position: (%f, %f, %f)",
              mpc_interface_.getState().left_foot.pose.translation.x(),
              mpc_interface_.getState().left_foot.pose.translation.y(),
              mpc_interface_.getState().left_foot.pose.translation.z());
  RCLCPP_INFO(logger, "Right Foot Position: (%f, %f, %f)",
              mpc_interface_.getState().right_foot.pose.translation.x(),
              mpc_interface_.getState().right_foot.pose.translation.y(),
              mpc_interface_.getState().right_foot.pose.translation.z());
  RCLCPP_INFO(logger, "Desired Left Foot Position: (%f, %f, %f)",
              mpc_interface_.getState().desired_left_foot.pose.translation.x(),
              mpc_interface_.getState().desired_left_foot.pose.translation.y(),
              mpc_interface_.getState().desired_left_foot.pose.translation.z());
  RCLCPP_INFO(
      logger, "Desired Right Foot Position: (%f, %f, %f)",
      mpc_interface_.getState().desired_right_foot.pose.translation.x(),
      mpc_interface_.getState().desired_right_foot.pose.translation.y(),
      mpc_interface_.getState().desired_right_foot.pose.translation.z());
  RCLCPP_INFO(logger, "-------------------------\n\n");
}

} // namespace ros
} // namespace ismpc

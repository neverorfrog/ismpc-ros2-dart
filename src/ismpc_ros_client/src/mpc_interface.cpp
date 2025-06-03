#include "ismpc_ros_client/mpc_interface.h"

#include <rclcpp/logger.hpp>

namespace ismpc {
namespace ros {
MpcInterface::MpcInterface()
    : params_(),
      frame_info_(),
      reference_(params_),
      plan_(params_),
      state_(params_),
      initial_state_(params_),
      planner_(frame_info_, reference_, state_, plan_, params_),
      mc_provider_(frame_info_, state_, plan_, params_),
      foot_trajectory_generator_(frame_info_, state_, plan_, params_),
      kalman_filter_(params_),
      mpc_controller_(frame_info_, state_, plan_, params_) {}

void MpcInterface::step() {
    // Filtering state (lip data was just received)
    kalman_filter_.update(state_);

    // Foot Planning
    planner_.update(plan_);
    mc_provider_.update(plan_);
    foot_trajectory_generator_.update(state_);

    // MPC Execution
    mpc_controller_.update(state_);

    // Torso Task
    const auto lf_rotvec = Math::matrixToRotvec(state_.desired_left_foot.pose.rotation.matrix());
    const auto rf_rotvec = Math::matrixToRotvec(state_.desired_right_foot.pose.rotation.matrix());
    const auto torso_rotmat = Math::rotvecToMatrix((lf_rotvec + rf_rotvec) / 2.0);
    state_.desired_torso.pose.rotation = ismpc::RotationMatrix(torso_rotmat);

    const auto lf_rotvec_dot = state_.desired_left_foot.ang_vel;
    const auto rf_rotvec_dot = state_.desired_right_foot.ang_vel;
    const auto torso_rotvec_dot = (lf_rotvec_dot + rf_rotvec_dot) / 2.0;
    state_.desired_torso.ang_vel = torso_rotvec_dot;

    const auto lf_rotvec_ddot = state_.desired_left_foot.ang_acc;
    const auto rf_rotvec_ddot = state_.desired_right_foot.ang_acc;
    state_.desired_torso.ang_acc = (lf_rotvec_ddot + rf_rotvec_ddot) / 2.0;

    // Advance time
    frame_info_.tk += params_.mpc.delta;
    frame_info_.k += 1;
}

void MpcInterface::updateStateFromLipData(const ismpc_interfaces::msg::LipData &lip_data) {
    state_.lip.com_pos = ConversionUtils::toEigenVector(lip_data.com_pos);
    state_.lip.com_vel = ConversionUtils::toEigenVector(lip_data.com_vel);
    state_.lip.com_acc = ConversionUtils::toEigenVector(lip_data.com_acc);
    state_.lip.zmp_pos = ConversionUtils::toEigenVector(lip_data.zmp_pos);
    state_.left_foot_contact = lip_data.left_contact;
    state_.right_foot_contact = lip_data.right_contact;
}

void MpcInterface::updateStateFromTransforms(const geometry_msgs::msg::TransformStamped &torso_tf,
                                             const geometry_msgs::msg::TransformStamped &base_tf,
                                             const geometry_msgs::msg::TransformStamped &lsole_tf,
                                             const geometry_msgs::msg::TransformStamped &rsole_tf) {
    state_.left_foot.pose = ConversionUtils::toCustomPose(lsole_tf);
    state_.base.pose = ConversionUtils::toCustomPose(base_tf);
    state_.right_foot.pose = ConversionUtils::toCustomPose(rsole_tf);
    state_.torso.pose = ConversionUtils::toCustomPose(torso_tf);
}

}  // namespace ros
}  // namespace ismpc
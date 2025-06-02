#include "ismpc_ros_client/node.h"

namespace ismpc {
namespace ros {

RosWalkEngine::RosWalkEngine() : Node("walk_engine"), mpc_interface_() {
    // Update callback executed every delta*1000 ms
    auto dt = std::chrono::duration<double>(100);
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt);
    timer = this->create_wall_timer(dt_ms, std::bind(&RosWalkEngine::log, this));

    // Subscribers
    lip_data_sub = this->create_subscription<ismpc_interfaces::msg::LipData>(
        "lip_data", 10, std::bind(&RosWalkEngine::lipCallback, this, std::placeholders::_1));

    // Publishers
    desired_state_pub = this->create_publisher<ismpc_interfaces::msg::DesiredState>("desired_state", 10);
}

void RosWalkEngine::log() {
    std::chrono::_V2::system_clock::time_point iteration_start = Time::now();
    RCLCPP_INFO(logger, "-------------------------");
    RCLCPP_INFO(logger, "Current Time: %f", mpc_interface_.getFrameInfo().tk);
    RCLCPP_INFO(logger, "Current Step: %d", mpc_interface_.getFrameInfo().k);
    RCLCPP_INFO(logger, "Reference Velocity: (%f, %f, %f)", mpc_interface_.getReference().velocity.vx,
                mpc_interface_.getReference().velocity.vy, mpc_interface_.getReference().velocity.omega);
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
    RCLCPP_INFO(logger, "Desired Right Foot Position: (%f, %f, %f)",
                mpc_interface_.getState().desired_right_foot.pose.translation.x(),
                mpc_interface_.getState().desired_right_foot.pose.translation.y(),
                mpc_interface_.getState().desired_right_foot.pose.translation.z());
    RCLCPP_INFO(logger, "-------------------------\n\n");
}

void RosWalkEngine::lipCallback(const ismpc_interfaces::msg::LipData::SharedPtr msg) {
    // Convert ROS message to internal representation
    ismpc_interfaces::msg::LipData lip_data;
    lip_data.header = msg->header;
    lip_data.com_pos = msg->com_pos;
    lip_data.com_vel = msg->com_vel;
    lip_data.com_acc = msg->com_acc;

    // Once the new lip data is received, update the MPC state
    mpc_interface_.step(lip_data);
    publishDesiredState(mpc_interface_.getState());
}

void RosWalkEngine::publishDesiredState(const ismpc::State& state) {
    auto desired_state_msg = ismpc_interfaces::msg::DesiredState();
    desired_state_msg.header.stamp = this->get_clock()->now();

    desired_state_msg.lip.com_pos = ConversionUtils::toRosVector(state.lip.com_pos);
    desired_state_msg.lip.com_vel = ConversionUtils::toRosVector(state.lip.com_vel);
    desired_state_msg.lip.com_acc = ConversionUtils::toRosVector(state.lip.com_acc);
    desired_state_msg.lip.zmp_pos = ConversionUtils::toRosVector(state.lip.zmp_pos);

    auto left_foot_msg = ismpc_interfaces::msg::EndEffector();
    auto right_foot_msg = ismpc_interfaces::msg::EndEffector();
    auto torso_msg = ismpc_interfaces::msg::EndEffector();

    left_foot_msg.name = "lsole";
    left_foot_msg.is_foot = true;
    left_foot_msg.in_contact = state.left_foot_contact;
    left_foot_msg.pose = ConversionUtils::toRosPose(state.left_foot.pose);
    left_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.left_foot.lin_vel);
    left_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.left_foot.ang_vel);
    left_foot_msg.acceleration.linear = ConversionUtils::toRosVector(state.left_foot.lin_acc);
    left_foot_msg.acceleration.angular = ConversionUtils::toRosVector(state.left_foot.ang_acc);
    desired_state_msg.left_foot = left_foot_msg;

    right_foot_msg.name = "rsole";
    right_foot_msg.is_foot = true;
    right_foot_msg.in_contact = state.right_foot_contact;
    right_foot_msg.pose = ConversionUtils::toRosPose(state.right_foot.pose);
    right_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.right_foot.lin_vel);
    right_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.right_foot.ang_vel);
    right_foot_msg.acceleration.linear = ConversionUtils::toRosVector(state.right_foot.lin_acc);
    right_foot_msg.acceleration.angular = ConversionUtils::toRosVector(state.right_foot.ang_acc);
    desired_state_msg.right_foot = right_foot_msg;

    torso_msg.name = "torso";
    torso_msg.is_foot = false;
    torso_msg.in_contact = false;  // Torso is not a foot
    torso_msg.pose.orientation = ConversionUtils::toRosPose(state.torso.pose).orientation;
    torso_msg.velocity.angular = ConversionUtils::toRosVector(state.torso.ang_vel);
    torso_msg.acceleration.angular = ConversionUtils::toRosVector(state.torso.ang_acc);
    desired_state_msg.torso = torso_msg;

    desired_state_pub->publish(desired_state_msg);
}

}  // namespace ros
}  // namespace ismpc

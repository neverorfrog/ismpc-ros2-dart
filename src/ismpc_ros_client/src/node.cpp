#include "ismpc_ros_client/node.h"

#include <ismpc_ros_utils/utils.h>

namespace ismpc {
namespace ros {

RosWalkEngine::RosWalkEngine() : Node("walk_engine"), mpc_interface_() {
    auto dt = std::chrono::duration<double>(mpc_interface_.getParams().mpc.delta);
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt);
    timer = this->create_wall_timer(dt_ms, std::bind(&RosWalkEngine::step, this));

    // Initialize TF2 components
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    lip_data_sub = this->create_subscription<ismpc_interfaces::msg::LipData>(
        "lip_data", 10, std::bind(&RosWalkEngine::updateStateFromLipData, this, std::placeholders::_1));

    // Publishers
    state_pub = this->create_publisher<ismpc_interfaces::msg::State>("state", 10);
}

void RosWalkEngine::step() {
    updateStateFromTransforms();
    auto now = this->get_clock()->now();

    // Check if we have lip data and if it's recent
    if (latest_lip_data_) {
        // Use the header timestamp from the message
        auto msg_time = rclcpp::Time(latest_lip_data_->header.stamp, this->get_clock()->get_clock_type());
        auto time_since_last_data = now - msg_time;

        if (time_since_last_data.seconds() <= 0.5) {
            mpc_interface_.updateStateFromLipData(*latest_lip_data_);
        } else {
            RCLCPP_WARN(logger, "Lip data is too old (%.3f seconds). Skipping MPC step.",
                        time_since_last_data.seconds());
            return;
        }
    } else {
        RCLCPP_WARN(logger, "No lip data received yet. Skipping MPC step.");
        return;
    }

    mpc_interface_.step();
    publishState(mpc_interface_.getState());
}

void RosWalkEngine::updateStateFromLipData(const ismpc_interfaces::msg::LipData::SharedPtr msg) {
    // Convert ROS message to internal representation
    latest_lip_data_ = std::make_unique<ismpc_interfaces::msg::LipData>();
    latest_lip_data_->header = msg->header;
    latest_lip_data_->com_pos = msg->com_pos;
    latest_lip_data_->com_vel = msg->com_vel;
    latest_lip_data_->com_acc = msg->com_acc;
    last_lip_data_timestamp_ = this->get_clock()->now();
}

void RosWalkEngine::updateStateFromTransforms() {
    // Retrieve transforms from tf2 and update the robot state
    geometry_msgs::msg::TransformStamped torso_tf, base_tf, lsole_tf, rsole_tf;
    try {
        torso_tf = tf_buffer_->lookupTransform("world", "torso", tf2::TimePointZero);
        base_tf = tf_buffer_->lookupTransform("world", "body", tf2::TimePointZero);
        lsole_tf = tf_buffer_->lookupTransform("world", "l_sole", tf2::TimePointZero);
        rsole_tf = tf_buffer_->lookupTransform("world", "r_sole", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(logger, "Could not get transforms: %s", ex.what());
        return;
    }

    // Update the state with the retrieved transforms
    mpc_interface_.updateStateFromTransforms(torso_tf, base_tf, lsole_tf, rsole_tf);
}

void RosWalkEngine::publishState(const ismpc::State& state) {
    auto state_msg = ismpc_interfaces::msg::State();
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.k = mpc_interface_.getFrameInfo().k;
    state_msg.tk = mpc_interface_.getFrameInfo().tk;

    auto lip_msg = ismpc_interfaces::msg::LipData();
    auto desired_lip_msg = ismpc_interfaces::msg::LipData();

    lip_msg.header = state_msg.header;
    lip_msg.com_pos = ConversionUtils::toRosVector(state.lip.com_pos);
    lip_msg.com_vel = ConversionUtils::toRosVector(state.lip.com_vel);
    lip_msg.com_acc = ConversionUtils::toRosVector(state.lip.com_acc);
    lip_msg.zmp_pos = ConversionUtils::toRosVector(state.lip.zmp_pos);
    state_msg.lip = lip_msg;
    desired_lip_msg.header = state_msg.header;
    desired_lip_msg.com_pos = ConversionUtils::toRosVector(state.desired_lip.com_pos);
    desired_lip_msg.com_vel = ConversionUtils::toRosVector(state.desired_lip.com_vel);
    desired_lip_msg.com_acc = ConversionUtils::toRosVector(state.desired_lip.com_acc);
    desired_lip_msg.zmp_pos = ConversionUtils::toRosVector(state.desired_lip.zmp_pos);
    state_msg.desired_lip = desired_lip_msg;

    auto left_foot_msg = ismpc_interfaces::msg::EndEffector();
    auto desired_left_foot_msg = ismpc_interfaces::msg::EndEffector();

    left_foot_msg.name = "lsole";
    left_foot_msg.is_foot = true;
    left_foot_msg.in_contact = state.left_foot_contact;
    left_foot_msg.pose = ConversionUtils::toRosPose(state.left_foot.pose);
    left_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.left_foot.lin_vel);
    left_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.left_foot.ang_vel);
    left_foot_msg.acceleration.linear = ConversionUtils::toRosVector(state.left_foot.lin_acc);
    left_foot_msg.acceleration.angular = ConversionUtils::toRosVector(state.left_foot.ang_acc);
    state_msg.left_foot = left_foot_msg;
    desired_left_foot_msg.name = "lsole";
    desired_left_foot_msg.is_foot = true;
    desired_left_foot_msg.pose = ConversionUtils::toRosPose(state.desired_left_foot.pose);
    desired_left_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.desired_left_foot.lin_vel);
    desired_left_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.desired_left_foot.ang_vel);
    desired_left_foot_msg.acceleration.linear = ConversionUtils::toRosVector(state.desired_left_foot.lin_acc);
    desired_left_foot_msg.acceleration.angular
        = ConversionUtils::toRosVector(state.desired_left_foot.ang_acc);
    state_msg.desired_left_foot = desired_left_foot_msg;

    auto right_foot_msg = ismpc_interfaces::msg::EndEffector();
    auto desired_right_foot_msg = ismpc_interfaces::msg::EndEffector();

    right_foot_msg.name = "rsole";
    right_foot_msg.is_foot = true;
    right_foot_msg.in_contact = state.right_foot_contact;
    right_foot_msg.pose = ConversionUtils::toRosPose(state.right_foot.pose);
    right_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.right_foot.lin_vel);
    right_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.right_foot.ang_vel);
    right_foot_msg.acceleration.linear = ConversionUtils::toRosVector(state.right_foot.lin_acc);
    right_foot_msg.acceleration.angular = ConversionUtils::toRosVector(state.right_foot.ang_acc);
    state_msg.right_foot = right_foot_msg;
    desired_right_foot_msg.name = "rsole";
    desired_right_foot_msg.is_foot = true;
    desired_right_foot_msg.pose = ConversionUtils::toRosPose(state.desired_right_foot.pose);
    desired_right_foot_msg.velocity.linear = ConversionUtils::toRosVector(state.desired_right_foot.lin_vel);
    desired_right_foot_msg.velocity.angular = ConversionUtils::toRosVector(state.desired_right_foot.ang_vel);
    desired_right_foot_msg.acceleration.linear
        = ConversionUtils::toRosVector(state.desired_right_foot.lin_acc);
    desired_right_foot_msg.acceleration.angular
        = ConversionUtils::toRosVector(state.desired_right_foot.ang_acc);
    state_msg.desired_right_foot = desired_right_foot_msg;

    auto torso_msg = ismpc_interfaces::msg::EndEffector();
    auto desired_torso_msg = ismpc_interfaces::msg::EndEffector();

    torso_msg.name = "torso";
    torso_msg.is_foot = false;
    torso_msg.pose.orientation = ConversionUtils::toRosPose(state.torso.pose).orientation;
    torso_msg.velocity.angular = ConversionUtils::toRosVector(state.torso.ang_vel);
    torso_msg.acceleration.angular = ConversionUtils::toRosVector(state.torso.ang_acc);
    state_msg.torso = torso_msg;
    desired_torso_msg.name = "torso";
    desired_torso_msg.is_foot = false;
    desired_torso_msg.pose.orientation = ConversionUtils::toRosPose(state.desired_torso.pose).orientation;
    desired_torso_msg.velocity.angular = ConversionUtils::toRosVector(state.desired_torso.ang_vel);
    desired_torso_msg.acceleration.angular = ConversionUtils::toRosVector(state.desired_torso.ang_acc);
    state_msg.desired_torso = desired_torso_msg;

    auto base_msg = ismpc_interfaces::msg::EndEffector();
    auto desired_base_msg = ismpc_interfaces::msg::EndEffector();
    base_msg.name = "base";
    base_msg.is_foot = false;
    base_msg.pose.orientation = ConversionUtils::toRosPose(state.base.pose).orientation;
    base_msg.velocity.angular = ConversionUtils::toRosVector(state.base.ang_vel);
    base_msg.acceleration.angular = ConversionUtils::toRosVector(state.base.ang_acc);
    state_msg.base = base_msg;
    desired_base_msg.name = "base";
    desired_base_msg.is_foot = false;
    desired_base_msg.pose.orientation = desired_torso_msg.pose.orientation;
    desired_base_msg.velocity.angular = desired_torso_msg.velocity.angular;
    desired_base_msg.acceleration.angular = desired_torso_msg.acceleration.angular;
    state_msg.desired_base = desired_base_msg;

    state_pub->publish(state_msg);
}

}  // namespace ros
}  // namespace ismpc

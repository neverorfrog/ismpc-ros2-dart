#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ismpc_cpp/representations/state.h"
#include "ismpc_interfaces/msg/end_effector.hpp"
#include "ismpc_interfaces/msg/lip_data.hpp"
#include "ismpc_interfaces/msg/state.hpp"
#include "ismpc_ros_client/mpc_interface.h"
#include "ismpc_ros_utils/utils.h"

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
    MpcInterface mpc_interface_;

    /**
     * @brief Update MPC
     */
    void step();

    // Subscribers
    rclcpp::Subscription<ismpc_interfaces::msg::LipData>::SharedPtr lip_data_sub;
    std::unique_ptr<ismpc_interfaces::msg::LipData> latest_lip_data_;
    rclcpp::Time last_lip_data_timestamp_;
    void updateStateFromLipData(const ismpc_interfaces::msg::LipData::SharedPtr msg);

    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief Retrieves transforms from tf2 and updates the robot state
     */
    void updateStateFromTransforms();

    // Publishers
    rclcpp::Publisher<ismpc_interfaces::msg::State>::SharedPtr state_pub;
    void publishState(const ismpc::State& state);

    // Logging
    rclcpp::Logger logger = rclcpp::get_logger("RosWalkEngine");
};

}  // namespace ros
}  // namespace ismpc

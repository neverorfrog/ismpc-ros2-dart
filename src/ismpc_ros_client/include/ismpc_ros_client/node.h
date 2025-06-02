#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ismpc_interfaces/msg/lip_data.hpp"
#include "ismpc_interfaces/msg/desired_state.hpp"
#include "ismpc_interfaces/msg/end_effector.hpp"
#include "ismpc_ros_client/mpc_interface.h"
#include "ismpc_cpp/representations/state.h"
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

    // Subscribers
    rclcpp::Subscription<ismpc_interfaces::msg::LipData>::SharedPtr lip_data_sub;
    void lipCallback(const ismpc_interfaces::msg::LipData::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<ismpc_interfaces::msg::DesiredState>::SharedPtr desired_state_pub;
    void publishDesiredState(const ismpc::State& state);

    // Logging
    rclcpp::Logger logger = rclcpp::get_logger("RosWalkEngine");
    void log();
};

}  // namespace ros
}  // namespace ismpc

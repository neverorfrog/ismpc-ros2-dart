#include "dart_ros_bridge/node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ismpc::ros::DartBridgeNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting walk engine");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

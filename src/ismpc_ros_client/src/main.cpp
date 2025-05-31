#include "ismpc_ros_client/node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ismpc::ros::RosWalkEngine>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting walk engine");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

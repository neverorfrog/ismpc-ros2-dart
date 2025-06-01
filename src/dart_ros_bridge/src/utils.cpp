#include "dart_ros_bridge/utils.h"

namespace ismpc {
namespace ros {

TransformStamped ConversionUtils::toRosTransform(const Eigen::Isometry3d &iso, const std::string &frame) {
    TransformStamped t;
    t.header.frame_id = "world";
    t.header.stamp = rclcpp::Clock().now();
    t.child_frame_id = frame;

    t.transform.translation.x = iso.translation().x();
    t.transform.translation.y = iso.translation().y();
    t.transform.translation.z = iso.translation().z();

    Eigen::Quaterniond q(iso.rotation());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
}

Vector3 ConversionUtils::toRosVector(const Eigen::Vector3d &vec) {
    Vector3 v;
    v.x = vec.x();
    v.y = vec.y();
    v.z = vec.z();
    return v;
}

}  // namespace ros
}  // namespace ismpc
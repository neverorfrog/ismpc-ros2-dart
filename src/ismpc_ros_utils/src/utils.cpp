#include "ismpc_ros_utils/utils.h"

#include <ismpc_cpp/tools/math/rotation_matrix.h>

#include <string>

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

geometry_msgs::msg::Pose ConversionUtils::toRosPose(const Pose3 &pose) {
    geometry_msgs::msg::Pose p;
    p.position.x = pose.translation.x();
    p.position.y = pose.translation.y();
    p.position.z = pose.translation.z();

    const auto &rot_quaternion = Eigen::Quaterniond(pose.rotation.matrix());
    p.orientation.x = rot_quaternion.x();
    p.orientation.y = rot_quaternion.y();
    p.orientation.z = rot_quaternion.z();
    p.orientation.w = rot_quaternion.w();

    return p;
}

Pose3 ConversionUtils::toCustomPose(const geometry_msgs::msg::TransformStamped &transform) {
    Pose3 p;
    p.translation.x() = transform.transform.translation.x;
    p.translation.y() = transform.transform.translation.y;
    p.translation.z() = transform.transform.translation.z;

    Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                         transform.transform.rotation.y, transform.transform.rotation.z);
    p.rotation = ismpc::RotationMatrix(q.toRotationMatrix());

    return p;
}

Eigen::Matrix3d ConversionUtils::toEigenMatrix(const geometry_msgs::msg::Quaternion &quat) {
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    return q.toRotationMatrix();
}

geometry_msgs::msg::Vector3 ConversionUtils::toRosVector(const Eigen::Vector3d &vec) {
    geometry_msgs::msg::Vector3 v;
    v.x = vec.x();
    v.y = vec.y();
    v.z = vec.z();
    return v;
}

Eigen::Vector3d ConversionUtils::toEigenVector(const geometry_msgs::msg::Vector3 &vec) {
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}
Eigen::Vector3d ConversionUtils::toEigenVector(const geometry_msgs::msg::Point &vec) {
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

}  // namespace ros
}  // namespace ismpc
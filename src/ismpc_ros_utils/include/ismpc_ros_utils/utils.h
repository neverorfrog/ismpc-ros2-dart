#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ismpc_cpp/tools/math/pose3.h"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;

namespace ismpc {
namespace ros {

class ConversionUtils {
   public:
    /**
     * @brief Converts a DART Eigen Isometry3d to a ROS
     * geometry_msgs::msg::Transform
     * @param iso The Eigen Isometry3d to convert
     * @return The converted ROS Transform message
     */
    static TransformStamped toRosTransform(const Eigen::Isometry3d &iso, const std::string &frame);

    /**
     * @brief Converts a Pose3 to a ROS geometry_msgs::msg::Pose
     * @param pose The Pose3 to convert
     * @return The converted ROS Pose message
     */
    static Pose toRosPose(const Pose3 &pose);
    static Pose3 toCustomPose(const geometry_msgs::msg::TransformStamped &transform);
    static Eigen::Matrix3d toEigenMatrix(const geometry_msgs::msg::Quaternion &quat);

    /**
     * @brief Converts a DART Eigen Vector3d to a ROS geometry_msgs::msg::Vector3
     * @param vec The Eigen Vector3d to convert
     * @return The converted ROS Vector3 message
     */
    static geometry_msgs::msg::Vector3 toRosVector(const Eigen::Vector3d &vec);
    static Eigen::Vector3d toEigenVector(const geometry_msgs::msg::Vector3 &vec);
    static Eigen::Vector3d toEigenVector(const geometry_msgs::msg::Point &vec);

    /**
     * @brief Printing utils
     */
    template <typename T>
    static std::string vectorToString(const std::vector<T> &vec, const std::string &delimiter = ", ") {
        std::string result;
        for (size_t i = 0; i < vec.size(); ++i) {
            result += std::to_string(vec[i]);
            if (i < vec.size() - 1) {
                result += delimiter;
            }
        }
        return result;
    }
};

}  // namespace ros
}  // namespace ismpc
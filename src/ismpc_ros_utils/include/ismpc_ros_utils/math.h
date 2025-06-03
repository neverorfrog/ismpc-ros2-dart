#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include "ismpc_ros_utils/utils.h"


using geometry_msgs::msg::Pose;


namespace ismpc {
namespace ros {

class Math {
public:
    /**
     * Compute position difference between two poses
     */
    static Eigen::Vector3d positionDifference(const Pose& current, const Pose& desired);

    /**
     * Compute rotation difference as rotation vector between two poses
     */
    static Eigen::Vector3d rotationDifference(const Pose& current, const Pose& desired);

    /**
     * Compute pose difference (rotation + position) between two poses
     * Returns 6D vector: [rotation_diff, position_diff]
     */
    static Eigen::VectorXd poseDifference(const Pose& current, const Pose& desired);

    /**
     * Convert rotation matrix to rotation vector (axis-angle representation)
     */
    static Eigen::Vector3d matrixToRotvec(const Eigen::Matrix3d& rotation_matrix);
    static Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d& rotation_vector);

};

}  // namespace ros
}  // namespace ismpc
#include "ismpc_ros_utils/math.h"

#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ismpc_ros_utils/utils.h"

namespace ismpc {
namespace ros {

Eigen::Vector3d Math::positionDifference(const Pose& current, const Pose& desired) {
    Eigen::Vector3d pos_current = ConversionUtils::toEigenVector(current.position);
    Eigen::Vector3d pos_desired = ConversionUtils::toEigenVector(desired.position);
    return pos_desired - pos_current;
}

Eigen::Vector3d Math::rotationDifference(const Pose& current, const Pose& desired) {
    Eigen::Matrix3d rot_desired = ConversionUtils::toEigenMatrix(desired.orientation);
    Eigen::Matrix3d rot_current = ConversionUtils::toEigenMatrix(current.orientation);

    Eigen::Matrix3d R_error = rot_desired * rot_current.transpose();  // TODO: correct?????

    return matrixToRotvec(R_error);
}

Eigen::VectorXd Math::poseDifference(const Pose& current, const Pose& desired) {
    Eigen::Vector3d pos_diff = positionDifference(current, desired);
    Eigen::Vector3d rot_diff = rotationDifference(current, desired);

    Eigen::VectorXd result(6);
    result.head<3>() = rot_diff;
    result.tail<3>() = pos_diff;

    return result;
}

Eigen::Vector3d Math::matrixToRotvec(const Eigen::Matrix3d& rotation_matrix) {
    Eigen::AngleAxisd angle_axis(rotation_matrix);
    return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix3d Math::rotvecToMatrix(const Eigen::Vector3d& rotation_vector) {
    double angle = rotation_vector.norm();
    if (angle < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d axis = rotation_vector / angle;
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

}  // namespace ros
}  // namespace ismpc
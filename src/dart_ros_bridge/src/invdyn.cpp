#include "dart_ros_bridge/invdyn.h"

#include <Eigen/src/Core/Matrix.h>
#include <ismpc_ros_utils/utils.h>

#include <sstream>

namespace ismpc {
namespace ros {

InverseDynamics::InverseDynamics(const dart::dynamics::SkeletonPtr& robot,
                                 const dart::simulation::WorldPtr& world)
    : robot_(robot), world_(world) {
    l_foot_ = robot->getBodyNode("l_sole");
    r_foot_ = robot->getBodyNode("r_sole");
    torso_ = robot->getBodyNode("torso");
    base_ = robot->getBodyNode("body");
    initial_q_ = robot->getPositions();
    initial_qdot_ = robot->getVelocities();

    num_contacts_ = 2;
    num_contact_dims_ = 6 * num_contacts_;  // 6D contact forces (fx, fy, fz, mx, my, mz)
    n_vars_ = 2 * robot->getNumDofs() + num_contact_dims_;
    n_eq_constraints_ = robot->getNumDofs();
    n_ineq_constraints_ = 8 * num_contacts_ + 2;

    qp_ = std::make_unique<QP<double>>(n_vars_, n_eq_constraints_, n_ineq_constraints_);

    // TODO: Make configurable
    joint_selection_.resize(robot->getNumDofs(), robot->getNumDofs());
    joint_selection_(6, 6) = 0;    // CHEST_P
    joint_selection_(7, 7) = 0;    // CHEST_Y
    joint_selection_(8, 8) = 1;    // L_SHOULDER_P
    joint_selection_(9, 9) = 1;    // L_SHOULDER_R
    joint_selection_(10, 10) = 1;  // L_SHOULDER_Y
    joint_selection_(11, 11) = 1;  // L_ELBOW_P
    joint_selection_(12, 12) = 1;  // NECK_Y
    joint_selection_(13, 13) = 1;  // NECK_P
    joint_selection_(14, 14) = 1;  // R_SHOULDER_P
    joint_selection_(15, 15) = 1;  // R_SHOULDER_R
    joint_selection_(16, 16) = 1;  // R_SHOULDER_Y
    joint_selection_(17, 17) = 1;  // R_ELBOW_P
    joint_selection_(18, 18) = 0;  // L_HIP_YAW
    joint_selection_(19, 19) = 0;  // L_HIP_ROLL
    joint_selection_(20, 20) = 0;  // L_HIP_PITCH
    joint_selection_(21, 21) = 0;  // L_KNEE_PITCH
    joint_selection_(22, 22) = 0;  // L_ANKLE_PITCH
    joint_selection_(23, 23) = 0;  // L_ANKLE_ROLL
    joint_selection_(24, 24) = 0;  // R_HIP_YAW
    joint_selection_(25, 25) = 0;  // R_HIP_ROLL
    joint_selection_(26, 26) = 0;  // R_HIP_PITCH
    joint_selection_(27, 27) = 0;  // R_KNEE_PITCH
    joint_selection_(28, 28) = 0;  // R_ANKLE_PITCH
    joint_selection_(29, 29) = 0;  // R_ANKLE_ROLL
}

Eigen::VectorXd InverseDynamics::computeJointTorques(const State::SharedPtr state) {
    const long dofs_ = robot_->getNumDofs();
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(dofs_);
    std::map<std::string, Eigen::MatrixXd> J = getJacobians();
    std::map<std::string, Eigen::MatrixXd> Jdot = getJacobiansDot();
    std::map<std::string, Eigen::MatrixXd> ff = getFeedforwardTerms(state);
    std::map<std::string, Eigen::MatrixXd> pos_errors = getPositionErrors(state);
    std::map<std::string, Eigen::MatrixXd> vel_errors = getVelocityErrors(state);

    RCLCPP_INFO(rclcpp::get_logger("InverseDynamics"), "======================================");
    RCLCPP_INFO(rclcpp::get_logger("InverseDynamics"), "\nDesired left foot position\n[%.3f, %.3f, %.3f]",
                state->desired_left_foot.pose.position.x, state->desired_left_foot.pose.position.y,
                state->desired_left_foot.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("InverseDynamics"), "\nDesired right foot position\n[%.3f, %.3f, %.3f]",
                state->desired_right_foot.pose.position.x, state->desired_right_foot.pose.position.y,
                state->desired_right_foot.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("InverseDynamics"), "======================================\n\n\n");

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars_, n_vars_);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars_);

    std::vector<int> q_ddot_indices(dofs_);
    std::vector<int> tau_indices(dofs_);
    std::vector<int> f_c_indices(num_contact_dims_);

    std::iota(q_ddot_indices.begin(), q_ddot_indices.end(), 0);
    std::iota(tau_indices.begin(), tau_indices.end(), dofs_);
    std::iota(f_c_indices.begin(), f_c_indices.end(), 2 * dofs_);

    for (const auto& task : tasks_config_.tasks) {
        Eigen::MatrixXd H_task = tasks_config_.weights[task] * J[task].transpose() * J[task];
        Eigen::VectorXd task_ref = ff[task] + tasks_config_.pos_gains[task] * pos_errors[task]
                                   + tasks_config_.vel_gains[task] * vel_errors[task]
                                   - Jdot[task] * robot_->getVelocities();
        Eigen::VectorXd g_task = -tasks_config_.weights[task] * J[task].transpose() * task_ref;

        H.block(0, 0, dofs_, dofs_) += H_task;
        g.head(dofs_) += g_task;
    }

    // Regularization term for contact forces
    for (size_t i = 0; i < f_c_indices.size(); ++i) {
        H(f_c_indices[i], f_c_indices[i]) += 1e-6;
    }

    // Actuation Matrix
    Eigen::MatrixXd actuation_matrix = Eigen::MatrixXd::Zero(dofs_, dofs_);
    actuation_matrix.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
    actuation_matrix.block(6, 6, dofs_ - 6, dofs_ - 6) = Eigen::MatrixXd::Identity(dofs_ - 6, dofs_ - 6);

    // Contact Jacobian
    Eigen::MatrixXd J_c = Eigen::MatrixXd::Zero(num_contact_dims_, dofs_);
    if (state->lip.left_contact) {
        J_c.block(0, 0, 6, dofs_) = J["l_foot"];
    }
    if (state->lip.right_contact) {
        J_c.block(6, 0, 6, dofs_) = J["r_foot"];
    }

    // Dynamics constraints: M * q_ddot + C - J_c^T * f_c = tau
    Eigen::MatrixXd A_eq = Eigen::MatrixXd::Zero(n_eq_constraints_, n_vars_);
    A_eq.block(0, 0, dofs_, dofs_) = robot_->getMassMatrix();
    A_eq.block(0, dofs_, dofs_, dofs_) = -actuation_matrix;
    A_eq.block(0, 2 * dofs_, dofs_, num_contact_dims_) = -J_c.transpose();
    Eigen::VectorXd b_eq = -robot_->getCoriolisAndGravityForces();

    Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(n_ineq_constraints_, n_vars_);
    Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(n_ineq_constraints_);

    /**
     * Contact force constraints for a single 6D contact wrench f_c_single = [fx, fy, fz, tx, ty, tz]
     *
     * Rows 0-3: |fx| <= d * fz and |fy| <= d * fz (friction cone approximation)
     * Rows 4-7: |tx| <= mu * fz and |ty| <= mu * fz (tilting moment constraints)
     * Row 8: fz >= 0 (unilateral contact constraint)
     */
    // clang-format off
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 6);
    A(0, 0) = 1;   A(0, 5) = -d_;   // fx <= d * fz
    A(1, 0) = -1;  A(1, 5) = -d_;   // -fx <= d * fz
    A(2, 1) = 1;   A(2, 5) = -d_;   // fy <= d * fz  
    A(3, 1) = -1;  A(3, 5) = -d_;   // -fy <= d * fz
    A(4, 3) = 1;   A(4, 5) = -mu_;  // tx <= mu * fz
    A(5, 3) = -1;  A(5, 5) = -mu_;  // -tx <= mu * fz
    A(6, 4) = 1;   A(6, 5) = -mu_;  // ty <= mu * fz
    A(7, 4) = -1;  A(7, 5) = -mu_;  // -ty <= mu * fz
    A(8, 2) = -1;                  // -fz <= 0 (i.e., fz >= 0)
    // clang-format on

    // Create block diagonal matrix for both contacts
    // A_ineq has shape (n_ineq_constraints_, n_vars_)
    // We need to fill the contact force constraint part
    A_ineq.block(0, f_c_indices[0], 9, 6) = A;
    A_ineq.block(9, f_c_indices[6], 9, 6) = A;

    // Compute solution using the QP solver
    qp_->settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp_->update(H, g, A_eq, b_eq, A_ineq, std::nullopt, b_ineq);
    qp_->solve();
    solution_ = qp_->results.x;

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dofs_);
    int tau_index = 0;
    for (const auto& index : tau_indices) {
        tau(tau_index++) = solution_(index);
    }

    Eigen::VectorXd command = tau.segment(6, dofs_ - 6);

    return command;
}

std::map<std::string, Eigen::MatrixXd> InverseDynamics::getJacobians() const {
    std::map<std::string, Eigen::MatrixXd> jacobians;

    jacobians["l_foot"] = robot_->getJacobian(l_foot_, world_frame_);       // [6, n_dofs]
    jacobians["r_foot"] = robot_->getJacobian(r_foot_, world_frame_);       // [6, n_dofs]
    jacobians["torso"] = robot_->getAngularJacobian(torso_, world_frame_);  // [3, n_dofs]
    jacobians["base"] = robot_->getAngularJacobian(base_, world_frame_);    // [3, n_dofs]
    jacobians["com"] = robot_->getCOMLinearJacobian(world_frame_);          // [3, n_dofs]
    jacobians["joints"] = joint_selection_;                                 // [n_dofs, n_dofs]
    return jacobians;
}

std::map<std::string, Eigen::MatrixXd> InverseDynamics::getJacobiansDot() const {
    std::map<std::string, Eigen::MatrixXd> jacobians_dot;

    jacobians_dot["l_foot"] = robot_->getJacobianClassicDeriv(l_foot_, world_frame_);  // [6, n_dofs]
    jacobians_dot["r_foot"] = robot_->getJacobianClassicDeriv(r_foot_, world_frame_);  // [6, n_dofs]
    jacobians_dot["torso"] = robot_->getAngularJacobianDeriv(torso_, world_frame_);    // [3, n_dofs]
    jacobians_dot["base"] = robot_->getAngularJacobianDeriv(base_, world_frame_);      // [3, n_dofs]
    jacobians_dot["com"] = robot_->getCOMLinearJacobianDeriv(world_frame_);            // [3, n_dofs]
    jacobians_dot["joints"] = Eigen::MatrixXd::Zero(robot_->getNumDofs(), robot_->getNumDofs());
    return jacobians_dot;
}

std::map<std::string, Eigen::MatrixXd>
InverseDynamics::getFeedforwardTerms(const State::SharedPtr state) const {
    std::map<std::string, Eigen::MatrixXd> feedforward_terms;

    Eigen::Vector6d l_foot_acc = Eigen::Vector6d::Zero();
    l_foot_acc.head<3>() = ConversionUtils::toEigenVector(state->desired_left_foot.acceleration.angular);
    l_foot_acc.tail<3>() = ConversionUtils::toEigenVector(state->desired_left_foot.acceleration.linear);
    feedforward_terms["l_foot"] = l_foot_acc;
    Eigen::Vector6d r_foot_acc = Eigen::Vector6d::Zero();
    r_foot_acc.head<3>() = ConversionUtils::toEigenVector(state->desired_right_foot.acceleration.angular);
    r_foot_acc.tail<3>() = ConversionUtils::toEigenVector(state->desired_right_foot.acceleration.linear);
    feedforward_terms["r_foot"] = r_foot_acc;
    feedforward_terms["torso"] = ConversionUtils::toEigenVector(state->desired_torso.acceleration.angular);
    feedforward_terms["base"] = ConversionUtils::toEigenVector(state->desired_torso.acceleration.angular);
    feedforward_terms["com"] = ConversionUtils::toEigenVector(state->desired_lip.com_acc);
    feedforward_terms["joints"] = Eigen::MatrixXd::Zero(robot_->getNumDofs(), 1);

    return feedforward_terms;
}

std::map<std::string, Eigen::MatrixXd>
InverseDynamics::getPositionErrors(const State::SharedPtr state) const {
    std::map<std::string, Eigen::MatrixXd> position_errors;

    position_errors["l_foot"] = Math::poseDifference(state->left_foot.pose, state->desired_left_foot.pose);
    position_errors["r_foot"] = Math::poseDifference(state->right_foot.pose, state->desired_right_foot.pose);
    position_errors["torso"] = Math::rotationDifference(state->torso.pose, state->desired_torso.pose);
    position_errors["base"] = Math::rotationDifference(state->base.pose, state->desired_torso.pose);
    position_errors["com"] = ConversionUtils::toEigenVector(state->desired_lip.com_pos)
                             - ConversionUtils::toEigenVector(state->lip.com_pos);
    position_errors["joints"] = initial_q_ - robot_->getPositions();

    return position_errors;
}

std::map<std::string, Eigen::MatrixXd>
InverseDynamics::getVelocityErrors(const State::SharedPtr state) const {
    std::map<std::string, Eigen::MatrixXd> velocity_errors;

    Eigen::Vector6d l_foot_vel_error = Eigen::Vector6d::Zero();
    l_foot_vel_error.head<3>() = ConversionUtils::toEigenVector(state->desired_left_foot.velocity.angular)
                                 - ConversionUtils::toEigenVector(state->left_foot.velocity.angular);
    l_foot_vel_error.tail<3>() = ConversionUtils::toEigenVector(state->desired_left_foot.velocity.linear)
                                 - ConversionUtils::toEigenVector(state->left_foot.velocity.linear);
    velocity_errors["l_foot"] = l_foot_vel_error;
    Eigen::Vector6d r_foot_vel_error = Eigen::Vector6d::Zero();
    r_foot_vel_error.head<3>() = ConversionUtils::toEigenVector(state->desired_right_foot.velocity.angular)
                                 - ConversionUtils::toEigenVector(state->right_foot.velocity.angular);
    r_foot_vel_error.tail<3>() = ConversionUtils::toEigenVector(state->desired_right_foot.velocity.linear)
                                 - ConversionUtils::toEigenVector(state->right_foot.velocity.linear);
    velocity_errors["r_foot"] = r_foot_vel_error;
    velocity_errors["torso"] = ConversionUtils::toEigenVector(state->desired_torso.velocity.angular)
                               - ConversionUtils::toEigenVector(state->torso.velocity.angular);
    velocity_errors["base"] = ConversionUtils::toEigenVector(state->desired_torso.velocity.angular)
                              - ConversionUtils::toEigenVector(state->base.velocity.angular);
    velocity_errors["com"] = ConversionUtils::toEigenVector(state->desired_lip.com_vel)
                             - ConversionUtils::toEigenVector(state->lip.com_vel);
    velocity_errors["joints"] = initial_qdot_ - robot_->getVelocities();

    return velocity_errors;
}

}  // namespace ros
}  // namespace ismpc
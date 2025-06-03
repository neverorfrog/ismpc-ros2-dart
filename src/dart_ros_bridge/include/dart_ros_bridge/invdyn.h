#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/status.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ismpc_interfaces/msg/state.hpp"
#include "ismpc_ros_utils/math.h"
#include "ismpc_ros_utils/utils.h"

using ismpc_interfaces::msg::State;
using proxsuite::proxqp::InitialGuessStatus;
using proxsuite::proxqp::dense::QP;

namespace ismpc {
namespace ros {

class InverseDynamics {
   public:
    InverseDynamics() = default;

    InverseDynamics(const dart::dynamics::SkeletonPtr& robot, const dart::simulation::WorldPtr& world);

    /**
     * @brief Compute joint torques based on the current and desired state of the robot.
     */
    Eigen::VectorXd computeJointTorques(const ismpc_interfaces::msg::State::SharedPtr state);

   private:
    dart::dynamics::SkeletonPtr robot_;
    dart::simulation::WorldPtr world_;
    Eigen::VectorXd initial_q_;
    Eigen::VectorXd initial_qdot_;
    double d_ = 0.05;                  // TODO: make configurable
    double mu_ = 0.5;                  // TODO: make configurable
    int num_contacts_;                 // TODO: make configurable
    int num_contact_dims_;             // 6D contact forces (fx, fy, fz, mx, my, mz)
    int n_vars_;                       // Number of optimization variables
    int n_eq_constraints_;             // Number of equality constraints
    int n_ineq_constraints_;           // Number of inequality constraints
    Eigen::MatrixXd joint_selection_;  // Joint selection matrix
    Eigen::VectorXd solution_;         // Optimization solution vector
    std::unique_ptr<QP<double>> qp_;   // QP solver instance

    /**
     * @brief Configuration for the tasks used in the inverse dynamics computation.
     * TODO: Make configurable via ROS parameters or constructor arguments.
     */
    // clang-format off
    struct TasksConfig {
        std::vector<std::string> tasks = { "joints", "com", "torso", "base", "l_foot", "r_foot" };

        std::unordered_map<std::string, double> weights
            = {{"com", 1.0}, {"torso", 1.0},{"base", 1.0}, {"joints", 5e-2},  {"l_foot", 1.0}, {"r_foot", 1.0}};

        std::unordered_map<std::string, double> pos_gains
            = {{"com", 10.0}, {"torso", 5.0}, {"base", 5.0},{"joints", 10.0},  {"l_foot", 5.0}, {"r_foot", 5.0}};

        std::unordered_map<std::string, double> vel_gains
            = {{"com", 10.0}, {"torso", 10.0}, {"base", 10.0},{"joints", 1e-1},  {"l_foot", 10.0}, {"r_foot", 10.0}};
    };
    // clang-format on

    dart::dynamics::BodyNode* l_foot_;
    dart::dynamics::BodyNode* r_foot_;
    dart::dynamics::BodyNode* torso_;
    dart::dynamics::BodyNode* base_;
    dart::dynamics::Frame* world_frame_ = dart::dynamics::Frame::World();

    TasksConfig tasks_config_{};
    std::map<std::string, Eigen::MatrixXd> getJacobians() const;
    std::map<std::string, Eigen::MatrixXd> getJacobiansDot() const;
    std::map<std::string, Eigen::MatrixXd> getFeedforwardTerms(const State::SharedPtr state) const;
    std::map<std::string, Eigen::MatrixXd> getPositionErrors(const State::SharedPtr state) const;
    std::map<std::string, Eigen::MatrixXd> getVelocityErrors(const State::SharedPtr state) const;
};

}  // namespace ros
}  // namespace ismpc
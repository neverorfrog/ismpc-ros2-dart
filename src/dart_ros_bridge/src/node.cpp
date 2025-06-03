#include "dart_ros_bridge/node.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <imgui.h>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

using geometry_msgs::msg::TransformStamped;

namespace ismpc {
namespace ros {

DartBridgeNode::DartBridgeNode() : Node("dart_ros_bridge") {
    try {
        this->constructWorld();
        invdyn = InverseDynamics(robot, world);

        // Publishers
        lip_data_pub = this->create_publisher<ismpc_interfaces::msg::LipData>("lip_data", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_markers", 10);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribers
        state_sub = this->create_subscription<ismpc_interfaces::msg::State>(
            "state", 10, std::bind(&DartBridgeNode::stateCallback, this, std::placeholders::_1));

        // Internal simulation timer
        simTimer = this->create_wall_timer(std::chrono::milliseconds(12),
                                           std::bind(&DartBridgeNode::simulationCallback, this));
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", e.what());
        return;
    }
}

void DartBridgeNode::simulationCallback() {
    if (!world) {
        RCLCPP_ERROR(this->get_logger(), "World is not initialized.");
        return;
    }

    world->step();

    publishLipData();
    publishTransforms();
    publishWorldFrame();
    publishMarkers();
}

void DartBridgeNode::stateCallback(const ismpc_interfaces::msg::State::SharedPtr msg) {
    if (!robot) {
        RCLCPP_ERROR(this->get_logger(), "Robot is not initialized.");
        return;
    }

    mpc_step = msg->k;
    mpc_time = msg->tk;
    com_pos = ConversionUtils::toEigenVector(msg->lip.com_pos);
    lf_pos = ConversionUtils::toEigenVector(msg->desired_left_foot.pose.position);
    rf_pos = ConversionUtils::toEigenVector(msg->desired_right_foot.pose.position);

    if (mpc_time > 10.0) {
        RCLCPP_INFO(logger, "Simulation time exceeded 10 seconds, stopping simulation.");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(logger, "=====================================");
    RCLCPP_INFO(logger, "Received state update from MPC at time %.3f", mpc_time);
    RCLCPP_INFO(logger, "\nCOM Position\n(%f, %f, %f)", msg->lip.com_pos.x, msg->lip.com_pos.y,
                msg->lip.com_pos.z);
    RCLCPP_INFO(logger, "\nDesired COM Position\n(%f, %f, %f)", msg->desired_lip.com_pos.x,
                msg->desired_lip.com_pos.y, msg->desired_lip.com_pos.z);
    RCLCPP_INFO(logger, "\nLeft Foot Position\n(%f, %f, %f)", msg->left_foot.pose.position.x,
                msg->left_foot.pose.position.y, msg->left_foot.pose.position.z);
    RCLCPP_INFO(logger, "\nDesired Left Foot Position\n(%f, %f, %f)", msg->desired_left_foot.pose.position.x,
                msg->desired_left_foot.pose.position.y, msg->desired_left_foot.pose.position.z);
    RCLCPP_INFO(logger, "\nRight Foot Position\n(%f, %f, %f)", msg->right_foot.pose.position.x,
                msg->right_foot.pose.position.y, msg->right_foot.pose.position.z);
    RCLCPP_INFO(logger, "\nDesired Right Foot Position\n(%f, %f, %f)",
                msg->desired_right_foot.pose.position.x, msg->desired_right_foot.pose.position.y,
                msg->desired_right_foot.pose.position.z);
    RCLCPP_INFO(logger, "\nTorso Orientation\n(%f, %f, %f, %f)", msg->torso.pose.orientation.x,
                msg->torso.pose.orientation.y, msg->torso.pose.orientation.z, msg->torso.pose.orientation.w);
    RCLCPP_INFO(logger, "\nDesired Torso Orientation\n(%f, %f, %f, %f)",
                msg->desired_torso.pose.orientation.x, msg->desired_torso.pose.orientation.y,
                msg->desired_torso.pose.orientation.z, msg->desired_torso.pose.orientation.w);
    RCLCPP_INFO(logger, "======================================\n\n\n");

    Eigen::VectorXd joint_torques = invdyn.computeJointTorques(msg);
    for (size_t i = 0; i < robot->getNumDofs() - 6; i++) {
        robot->setCommand(i + 6, joint_torques(i));
    }
}

void DartBridgeNode::publishLipData() {
    if (!robot)
        return;

    auto lip_data = ismpc_interfaces::msg::LipData();
    lip_data.header.stamp = this->get_clock()->now();

    lip_data.com_pos = ConversionUtils::toRosVector(robot->getCOM());
    lip_data.com_vel = ConversionUtils::toRosVector(robot->getCOMLinearVelocity());
    lip_data.com_acc = ConversionUtils::toRosVector(robot->getCOMLinearAcceleration());
    computeZmp(lip_data);

    lip_data_pub->publish(lip_data);
}

void DartBridgeNode::publishTransforms() {
    if (!robot)
        return;

    Eigen::Isometry3d torso_transform = robot->getBodyNode("torso")->getWorldTransform();
    Eigen::Isometry3d base_transform = robot->getBodyNode("body")->getWorldTransform();
    Eigen::Isometry3d lsole_transform = robot->getBodyNode("l_sole")->getWorldTransform();
    Eigen::Isometry3d rsole_transform = robot->getBodyNode("r_sole")->getWorldTransform();

    TransformStamped torso_tf = ConversionUtils::toRosTransform(torso_transform, "torso");
    TransformStamped base_tf = ConversionUtils::toRosTransform(base_transform, "body");
    TransformStamped lsole_tf = ConversionUtils::toRosTransform(lsole_transform, "l_sole");
    TransformStamped rsole_tf = ConversionUtils::toRosTransform(rsole_transform, "r_sole");

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(torso_tf);
    transforms.push_back(base_tf);
    transforms.push_back(lsole_tf);
    transforms.push_back(rsole_tf);

    tf_broadcaster->sendTransform(transforms);
}

void DartBridgeNode::computeZmp(ismpc_interfaces::msg::LipData &lip_data) {
    const auto &left_foot = robot->getBodyNode("l_sole");
    const auto &right_foot = robot->getBodyNode("r_sole");

    bool left_contact = false;
    bool right_contact = false;
    Eigen::VectorXd grf_L = left_foot->getConstraintImpulse();
    Eigen::VectorXd grf_R = right_foot->getConstraintImpulse();
    Eigen::Vector3d left_cop, right_cop;
    Eigen::Vector3d new_zmp_pos = zmp_pos;

    if (abs(grf_L[5]) > 0.1) {
        left_cop << -grf_L(1) / grf_L(5), grf_L(0) / grf_L(5), 0.0;
        left_cop = left_foot->getWorldTransform().translation()
                   + left_foot->getWorldTransform().rotation() * left_cop;
        left_contact = true;
    }

    if (abs(grf_R[5]) > 0.1) {
        right_cop << -grf_R(1) / grf_R(5), grf_R(0) / grf_R(5), 0.0;
        right_cop = right_foot->getWorldTransform().translation()
                    + right_foot->getWorldTransform().rotation() * right_cop;
        right_contact = true;
    }

    if (left_contact && right_contact) {
        new_zmp_pos = Eigen::Vector3d(
            (left_cop(0) * grf_L[5] + right_cop(0) * grf_R[5]) / (grf_L[5] + grf_R[5]),
            (left_cop(1) * grf_L[5] + right_cop(1) * grf_R[5]) / (grf_L[5] + grf_R[5]), 0.0);
    } else if (left_contact) {
        new_zmp_pos = Eigen::Vector3d(left_cop(0), left_cop(1), 0.0);
    } else if (right_contact) {
        new_zmp_pos = Eigen::Vector3d(right_cop(0), right_cop(1), 0.0);
    }

    new_zmp_pos[0] = std::clamp(new_zmp_pos[0], zmp_pos[0] - 0.1, zmp_pos[0] + 0.1);
    new_zmp_pos[1] = std::clamp(new_zmp_pos[1], zmp_pos[1] - 0.1, zmp_pos[1] + 0.1);
    zmp_pos = new_zmp_pos;

    lip_data.zmp_pos = ConversionUtils::toRosVector(zmp_pos);
    lip_data.left_contact = left_contact;
    lip_data.right_contact = right_contact;
}

void DartBridgeNode::publishMarkers() {
    if (!robot)
        return;

    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < robot->getNumBodyNodes(); i++) {
        auto body = robot->getBodyNode(i);
        auto transform = body->getWorldTransform();

        // Create a simple sphere marker for each body
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "robot_bodies";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position from DART transform
        marker.pose.position.x = transform.translation()[0];
        marker.pose.position.y = transform.translation()[1];
        marker.pose.position.z = transform.translation()[2];

        // Orientation from DART transform
        Eigen::Quaterniond quat(transform.rotation());
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        // Size and color
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    // Add ZMP marker
    visualization_msgs::msg::Marker zmp_marker;
    zmp_marker.header.frame_id = "world";
    zmp_marker.header.stamp = this->get_clock()->now();
    zmp_marker.ns = "zmp";
    zmp_marker.id = robot->getNumBodyNodes();
    zmp_marker.type = visualization_msgs::msg::Marker::SPHERE;
    zmp_marker.action = visualization_msgs::msg::Marker::ADD;
    zmp_marker.pose.position.x = zmp_pos.x();
    zmp_marker.pose.position.y = zmp_pos.y();
    zmp_marker.pose.position.z = zmp_pos.z();
    zmp_marker.pose.orientation.w = 1.0;
    zmp_marker.scale.x = zmp_marker.scale.y = zmp_marker.scale.z = 0.1;
    zmp_marker.color.r = 1.0;
    zmp_marker.color.g = 0.0;
    zmp_marker.color.b = 0.0;
    zmp_marker.color.a = 1.0;  // Fully opaque
    marker_array.markers.push_back(zmp_marker);

    // Add COM marker
    visualization_msgs::msg::Marker com_marker;
    com_marker.header.frame_id = "world";
    com_marker.header.stamp = this->get_clock()->now();
    com_marker.ns = "com";
    com_marker.id = robot->getNumBodyNodes() + 1;
    com_marker.type = visualization_msgs::msg::Marker::SPHERE;
    com_marker.action = visualization_msgs::msg::Marker::ADD;
    com_marker.pose.position.x = robot->getCOM()[0];
    com_marker.pose.position.y = robot->getCOM()[1];
    com_marker.pose.position.z = robot->getCOM()[2];
    com_marker.pose.orientation.w = 1.0;
    com_marker.scale.x = com_marker.scale.y = com_marker.scale.z = 0.1;
    com_marker.color.r = 0.0;
    com_marker.color.g = 0.0;
    com_marker.color.b = 1.0;
    com_marker.color.a = 1.0;  // Fully opaque
    marker_array.markers.push_back(com_marker);

    // Add left foot marker
    visualization_msgs::msg::Marker lfoot_marker;
    lfoot_marker.header.frame_id = "world";
    lfoot_marker.header.stamp = this->get_clock()->now();
    lfoot_marker.ns = "left_foot";
    lfoot_marker.id = robot->getNumBodyNodes() + 2;
    lfoot_marker.type = visualization_msgs::msg::Marker::SPHERE;
    lfoot_marker.action = visualization_msgs::msg::Marker::ADD;
    lfoot_marker.pose.position.x = lf_pos.x();
    lfoot_marker.pose.position.y = lf_pos.y();
    lfoot_marker.pose.position.z = lf_pos.z();
    lfoot_marker.pose.orientation.w = 1.0;
    lfoot_marker.scale.x = lfoot_marker.scale.y = lfoot_marker.scale.z = 0.1;
    lfoot_marker.color.r = 0.0;
    lfoot_marker.color.g = 1.0;
    lfoot_marker.color.b = 1.0;
    lfoot_marker.color.a = 1.0;  // Fully opaque
    marker_array.markers.push_back(lfoot_marker);

    // Add right foot marker
    visualization_msgs::msg::Marker rfoot_marker;
    rfoot_marker.header.frame_id = "world";
    rfoot_marker.header.stamp = this->get_clock()->now();
    rfoot_marker.ns = "right_foot";
    rfoot_marker.id = robot->getNumBodyNodes() + 3;
    rfoot_marker.type = visualization_msgs::msg::Marker::SPHERE;
    rfoot_marker.action = visualization_msgs::msg::Marker::ADD;
    rfoot_marker.pose.position.x = rf_pos.x();
    rfoot_marker.pose.position.y = rf_pos.y();
    rfoot_marker.pose.position.z = rf_pos.z();
    rfoot_marker.pose.orientation.w = 1.0;
    rfoot_marker.scale.x = rfoot_marker.scale.y = rfoot_marker.scale.z = 0.1;
    rfoot_marker.color.r = 0.0;
    rfoot_marker.color.g = 1.0;
    rfoot_marker.color.b = 1.0;
    rfoot_marker.color.a = 1.0;
    marker_array.markers.push_back(rfoot_marker);

    marker_pub->publish(marker_array);
}

void DartBridgeNode::publishWorldFrame() {
    // Publish world frame transform
    geometry_msgs::msg::TransformStamped world_transform;
    world_transform.header.stamp = this->get_clock()->now();
    world_transform.header.frame_id = "world";
    world_transform.child_frame_id = "base_link";

    // Identity transform (no rotation or translation)
    world_transform.transform.translation.x = 0.0;
    world_transform.transform.translation.y = 0.0;
    world_transform.transform.translation.z = 0.0;
    world_transform.transform.rotation.x = 0.0;
    world_transform.transform.rotation.y = 0.0;
    world_transform.transform.rotation.z = 0.0;
    world_transform.transform.rotation.w = 1.0;

    tf_broadcaster->sendTransform(world_transform);
}

void DartBridgeNode::constructWorld() {
    this->declare_parameter("robot_description_package", "hrp4_description");
    this->declare_parameter("robot_urdf_filename", "hrp4.urdf");
    this->declare_parameter("ground_urdf_filename", "ground.urdf");

    std::string robot_package = this->get_parameter("robot_description_package").as_string();
    std::string robot_urdf_filename = this->get_parameter("robot_urdf_filename").as_string();
    std::string ground_urdf_filename = this->get_parameter("ground_urdf_filename").as_string();

    std::string robot_urdf_path;
    std::string ground_urdf_path;

    try {
        std::string package_path = ament_index_cpp::get_package_share_directory(robot_package);
        robot_urdf_path = package_path + "/urdf/" + robot_urdf_filename;
        ground_urdf_path = package_path + "/urdf/" + ground_urdf_filename;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find package '%s': %s", robot_package.c_str(), e.what());
        return;
    }
    world = dart::simulation::World::create("dart_world");
    world->setGravity(Eigen::Vector3d(0, 0, -9.81));
    world->setTimeStep(0.012);  // TODO: Make configurable

    dart::utils::DartLoader::Options options;
    dart::utils::DartLoader loader(options);
    options.mDefaultInertia
        = dart::dynamics::Inertia(1e-8, Eigen::Vector3d::Zero(), 1e-10 * Eigen::Matrix3d::Identity());
    robot = loader.parseSkeleton(robot_urdf_path);
    auto ground = loader.parseSkeleton(ground_urdf_path);

    for (auto *bn : robot->getBodyNodes()) {
        if (bn->getMass() == 0.0) {
            bn->setMass(1e-8);
            bn->setInertia(options.mDefaultInertia);
        }
    }

    // set joint actuator type
    for (size_t i = 0; i < robot->getNumJoints(); i++) {
        size_t dim = robot->getJoint(i)->getNumDofs();
        if (dim == 6) {
            robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
        }
        if (dim == 1) {
            robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::FORCE);
        }
    }

    for (const auto &[key, value] : INITIAL_CONFIG) {
        robot->setPosition(robot->getDof(key)->getIndexInSkeleton(), value * M_PI / 180.0);
    }

    const auto &lsole_pos = robot->getBodyNode("l_sole")->getTransform().translation();
    const auto &rsole_pos = robot->getBodyNode("r_sole")->getTransform().translation();
    robot->setPosition(5, -(lsole_pos.z() + rsole_pos.z()) / 2.0);

    world->addSkeleton(robot);
    world->addSkeleton(ground);
}

}  // namespace ros
}  // namespace ismpc
#include "dart_ros_bridge/node.h"
#include <imgui.h>

namespace ismpc {
namespace ros {

DartBridgeNode::DartBridgeNode() : Node("dart_ros_bridge") {
    try {
        this->constructWorld();
        sim_data_pub = this->create_publisher<ismpc_interfaces::msg::SimData>("sim_data", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_markers", 10);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        simTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&DartBridgeNode::simulationCallback, this));
        start = Time::now();
        RCLCPP_INFO(this->get_logger(), "DART world started");
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

    // world->step();

    // Publish robot state for RViz
    publishWorldFrame();
    publishSimData();
    publishMarkers();
    publishTransforms();
}

void DartBridgeNode::publishSimData() {
    if (!robot)
        return;

    auto sim_data = ismpc_interfaces::msg::SimData();
    sim_data.header.stamp = this->get_clock()->now();
    sim_data.time = this->world->getTime();
    Eigen::VectorXd positions = robot->getPositions();
    Eigen::VectorXd velocities = robot->getVelocities();
    Eigen::VectorXd accelerations = robot->getAccelerations();
    sim_data.qpos = std::vector<double>(positions.size());
    sim_data.qvel = std::vector<double>(positions.size());
    sim_data.qacc = std::vector<double>(positions.size());
    for (int i = 0; i < positions.size(); ++i) {
        sim_data.qpos[i] = positions[i];
        sim_data.qvel[i] = velocities[i];
        sim_data.qacc[i] = accelerations[i];
    }

    sim_data.com_pos = ConversionUtils::toRosVector(robot->getCOM());
    sim_data.com_vel = ConversionUtils::toRosVector(robot->getCOMLinearVelocity());
    sim_data.com_acc = ConversionUtils::toRosVector(robot->getCOMLinearAcceleration());
    // sim_data.zmp_pos = computeZmp();

    sim_data_pub->publish(sim_data);
}

void DartBridgeNode::publishTransforms() {
    if (!robot)
        return;

    Eigen::Isometry3d torso_transform = robot->getBodyNode("torso")->getWorldTransform();
    Eigen::Isometry3d lsole_transform = robot->getBodyNode("l_sole")->getWorldTransform();
    Eigen::Isometry3d rsole_transform = robot->getBodyNode("r_sole")->getWorldTransform();

    // geometry_msgs::msg::TransformStamped base_tf = ConversionUtils::toRosTransform(base_transform, "base");
    geometry_msgs::msg::TransformStamped torso_tf = ConversionUtils::toRosTransform(torso_transform, "torso");
    geometry_msgs::msg::TransformStamped lsole_tf = ConversionUtils::toRosTransform(lsole_transform, "l_sole");
    geometry_msgs::msg::TransformStamped rsole_tf = ConversionUtils::toRosTransform(rsole_transform, "r_sole");

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(torso_tf);
    transforms.push_back(lsole_tf);
    transforms.push_back(rsole_tf);

    tf_broadcaster->sendTransform(transforms);
}

geometry_msgs::msg::Vector3 DartBridgeNode::computeZmp() {
    return ConversionUtils::toRosVector(Eigen::Vector3d::Zero());
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
    this->declare_parameter("robot_urdf_filename", "hrp4.urdf");  // Add this line!
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
    world->setTimeStep(0.01);

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
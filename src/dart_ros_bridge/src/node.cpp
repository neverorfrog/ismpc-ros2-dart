#include "dart_ros_bridge/node.h"
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <osgViewer/config/SingleWindow>

namespace ismpc {
namespace ros {

DartBridgeNode::DartBridgeNode() : Node("dart_ros_bridge") {

  try {
    this->constructWorld();
    this->constructViewer();
    // simTimer = this->create_wall_timer(
    //     std::chrono::milliseconds(10),
    //     std::bind(&DartBridgeNode::simulationCallback, this));
    // visTimer = this->create_wall_timer(
    //     std::chrono::milliseconds(20),
    //     std::bind(&DartBridgeNode::visualizationCallback, this));
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

  auto now = Time::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
  if (elapsed.count() < 10) {
    return; // Skip if less than 10ms has passed
  }
  start = now;

  world->step();
}

void DartBridgeNode::visualizationCallback() {
  if (!viewer) {
    RCLCPP_ERROR(this->get_logger(), "Viewer is not initialized.");
    return;
  }

  viewer->frame();
}

void DartBridgeNode::constructWorld() {

  this->declare_parameter("robot_description_package", "hrp4_description");
  this->declare_parameter("robot_urdf_filename", "hrp4.urdf"); // Add this line!
  this->declare_parameter("ground_urdf_filename", "ground.urdf");

  std::string robot_package =
      this->get_parameter("robot_description_package").as_string();
  std::string robot_urdf_filename =
      this->get_parameter("robot_urdf_filename").as_string();
  std::string ground_urdf_filename =
      this->get_parameter("ground_urdf_filename").as_string();

  std::string robot_urdf_path;
  std::string ground_urdf_path;

  try {
    std::string package_path =
        ament_index_cpp::get_package_share_directory(robot_package);
    robot_urdf_path = package_path + "/urdf/" + robot_urdf_filename;
    ground_urdf_path = package_path + "/urdf/" + ground_urdf_filename;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find package '%s': %s",
                 robot_package.c_str(), e.what());
    return;
  }
  world = dart::simulation::World::create("dart_world");
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(0.01);

  dart::utils::DartLoader::Options options;
  dart::utils::DartLoader loader(options);
  options.mDefaultInertia = dart::dynamics::Inertia(
      1e-8, Eigen::Vector3d::Zero(), 1e-10 * Eigen::Matrix3d::Identity());
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

  world->addSkeleton(robot);
  world->addSkeleton(ground);
}

void DartBridgeNode::constructViewer() {

  if (!world) {
    RCLCPP_ERROR(this->get_logger(), "World is not initialized.");
    return;
  }
  node = std::make_unique<dart::gui::osg::WorldNode>(world);
  if (!node) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create WorldNode.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Creating Viewer...");
  viewer = std::make_unique<dart::gui::osg::Viewer>();
  RCLCPP_INFO(this->get_logger(), "Viewer created successfully.");

  RCLCPP_INFO(this->get_logger(), "Setting up window...");
  viewer->apply(new osgViewer::SingleWindow(0, 0, 800, 600));
  viewer->realize();
  RCLCPP_INFO(this->get_logger(), "Window setup complete.");

  RCLCPP_INFO(this->get_logger(), "Adding WorldNode to viewer...");
  viewer->addWorldNode(node.get(), true);
  RCLCPP_INFO(this->get_logger(), "WorldNode added successfully.");
}

} // namespace ros
} // namespace ismpc
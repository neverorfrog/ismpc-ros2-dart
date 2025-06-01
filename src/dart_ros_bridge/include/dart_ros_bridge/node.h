#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <dart/dart.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/WorldNode.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgViewer/config/SingleWindow>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

typedef std::chrono::high_resolution_clock Time;

namespace ismpc {
namespace ros {

/**
 * @brief ROS2 node that bridges DART physics simulation with ROS2 ecosystem
 *
 * Provides integration between DART (Dynamic Animation and Robotics Toolkit)
 * physics simulation and ROS2, enabling robot simulation and control.
 */

class DartBridgeNode : public rclcpp::Node {
public:
  DartBridgeNode();

private:
  rclcpp::TimerBase::SharedPtr simTimer;
  rclcpp::TimerBase::SharedPtr visTimer;
  std::chrono::_V2::system_clock::time_point start;

  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr robot;
  std::unique_ptr<dart::gui::osg::WorldNode> node;
  std::unique_ptr<dart::gui::osg::Viewer> viewer;
  /**
   * @brief Constructs and initializes a DART simulation world
   */
  void constructWorld();

  /**
   * @brief Constructs and initializes the DART viewer
   *
   * Sets up the viewer to visualize the DART simulation world.
   */
  void constructViewer();

  /**
   * @brief Updates the DART simulation world
   *
   * This method is called periodically to update the DART simulation state.
   * It is bound to a timer that triggers at a specified interval.
   */
  void simulationCallback();

  /**
   * @brief Updates the DART viewer
   *
   */
  void visualizationCallback();
};

} // namespace ros
} // namespace ismpc
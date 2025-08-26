#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"  // Added include for tf2_ros::Buffer

namespace nav2_straightline_planner
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // Plugin configuration method
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Plugin cleanup
  void cleanup() override;

  // Plugin activation
  void activate() override;

  // Plugin deactivation
  void deactivate() override;

  void printCostmap();
  void printBinaryMap();

  // This method creates a path for a given start and goal pose.
  // Note: The signature exactly matches the base class.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer for transforms
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Lifecycle node pointer
  nav2_util::LifecycleNode::SharedPtr node_;

  // Pointer to the global costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // Global frame ID and plugin name
  std::string global_frame_, name_;

  // Resolution for interpolation when generating the path
  double interpolation_resolution_;

  // binary map
  int  binary_map[10000][10000];
  int width=0;
  int height=0;
  int threshold=100;
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

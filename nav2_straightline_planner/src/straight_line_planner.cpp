// UPP

#include <cmath>
#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <stack>
#include <utility>
#include <algorithm>
#include <iostream>

#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

// Include OccupancyGrid message for visualization
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_straightline_planner
{
  // Global parameters (do not modify these with resolution)
  int threshold = 200;

  // Heuristic parameters
  float radius = 8.0f;
  float beta = 2.0f;
  float alpha = 0.5f;


  struct PathNode {
      int x, y;
      float value;
      PathNode(int x, int y, float value) : x(x), y(y), value(value) {}
      PathNode() : x(0), y(0), value(0) {}
  };

  int manhattan_heuristics(int x1, int y1, int x2, int y2) {
      return std::abs(x2 - x1) + std::abs(y2 - y1);
  }

  int chebyshev_heuristic(int x1, int y1, int x2, int y2) {
      return std::max(std::abs(x2 - x1), std::abs(y2 - y1));
  }

  float safety_heuristics(int x, int y, const std::vector<std::vector<int>> &binary_map) {
      float safety_cost = 0.0f;
      int rows = binary_map.size();
      int cols = binary_map[0].size();
      for (int i = -static_cast<int>(radius); i <= static_cast<int>(radius); i++) {
          for (int j = -static_cast<int>(radius); j <= static_cast<int>(radius); j++) {
              int nx = x + i;
              int ny = y + j;
              if (nx >= 0 && nx < cols && ny >= 0 && ny < rows && binary_map[ny][nx] == 1) {
                  
                  safety_cost += 1/(0.0001 + std::max(std::abs(i), std::abs(j)));
              }
          }
      }

      
      return safety_cost;
  }

  float combined_heuristics(int x1, int y1, int x2, int y2, const std::vector<std::vector<int>> &binary_map) {
      return (alpha * manhattan_heuristics(x1, y1, x2, y2) +
              (1 - alpha) * chebyshev_heuristic(x1, y1, x2, y2) +
              beta * safety_heuristics(x1, y1, binary_map));
  }


  bool withinGrid(int x, int y, int width, int height) {
      return (x >= 0 && x < width && y >= 0 && y < height);
  }

 
  std::vector<PathNode> findPathAStar(int start_x, int start_y, int goal_x, int goal_y,
                                        const std::vector<std::vector<int>> &binary_map) {

      int height = binary_map.size();      
      if (height == 0) return {};
      int width = binary_map[0].size();    


      std::vector<std::vector<std::pair<int, int>>> parent(
          height, std::vector<std::pair<int, int>>(width, {-1, -1}));
      
      std::vector<std::vector<bool>> visited(
          height, std::vector<bool>(width, false));

     
      std::priority_queue<
          std::pair<float, std::pair<int, int>>,
          std::vector<std::pair<float, std::pair<int, int>>>,
          std::greater<std::pair<float, std::pair<int, int>>>
      > openlist;

     
      if (!withinGrid(start_x, start_y, width, height) || !withinGrid(goal_x, goal_y, width, height)) {
          std::cout << "Not in grid" << std::endl;
          return {};
      }

      float start_heur = combined_heuristics(start_x, start_y, goal_x, goal_y, binary_map);
      openlist.push({start_heur, {start_x, start_y}});

      int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
      int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

      bool found = false;
      while (!openlist.empty()) {
          auto currentPair = openlist.top();
          openlist.pop();

          int cx = currentPair.second.first;  // column
          int cy = currentPair.second.second;   // row

          std::cout << "value " << currentPair.first << std::endl;

          if (visited[cy][cx])
              continue;
          visited[cy][cx] = true;

          if (cx == goal_x && cy == goal_y) {
              found = true;
              break;
          }
         
          for (int i = 0; i < 8; i++) {
              int nx = cx + dx[i];
              int ny = cy + dy[i];

              if (withinGrid(nx, ny, width, height) && !visited[ny][nx] && binary_map[ny][nx] == 0) {
                  float heur = combined_heuristics(nx, ny, goal_x, goal_y, binary_map);
                  float move_cost = ((std::abs(dx[i]) + std::abs(dy[i])) == 2) ? 1.414f : 1.0f;
                  float newVal = move_cost + heur +
                                   std::max(std::abs(start_x - nx), std::abs(start_y - ny));
                  openlist.push({newVal, {nx, ny}});
                  if (parent[ny][nx].first == -1)
                      parent[ny][nx] = {cy, cx};
              }
          }
      }

      std::vector<PathNode> path;
      if (found) {
          int cx = goal_x;
          int cy = goal_y;
          std::stack<PathNode> reversePath;
          while (!(cx == start_x && cy == start_y)) {
              reversePath.push(PathNode(cx, cy, 0));
              auto par = parent[cy][cx];  // parent stored as (row, col)
              cy = par.first;
              cx = par.second;
          }
          reversePath.push(PathNode(start_x, start_y, 0));
          while (!reversePath.empty()) {
              path.push_back(reversePath.top());
              reversePath.pop();
          }
      }
      return path;
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr binary_map_pub_;

  void StraightLine::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
      node_ = parent.lock();
      name_ = name;
      tf_ = tf;
      costmap_ = costmap_ros->getCostmap();
      global_frame_ = costmap_ros->getGlobalFrameID();

      nav2_util::declare_parameter_if_not_declared(
          node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
      node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

      // Initialize the binary map publisher
      binary_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("binary_map", 1);
  }

  void StraightLine::cleanup() {
      RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s", name_.c_str());
  }

  void StraightLine::activate() {
      RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
  }

  void StraightLine::deactivate() {
      RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
  }

  void StraightLine::printCostmap() {
      unsigned int size_x = costmap_->getSizeInCellsX();
      unsigned int size_y = costmap_->getSizeInCellsY();
      RCLCPP_INFO(node_->get_logger(), "Costmap dimensions: %u x %u", size_x, size_y);
      for (unsigned int i = 0; i < size_x; i++) {
          for (unsigned int j = 0; j < size_y; j++) {
              unsigned char cost = costmap_->getCost(i, j);
              RCLCPP_INFO(node_->get_logger(), "Cost at (%u, %u): %u", i, j, cost);
          }
      }
  }

  nav_msgs::msg::Path StraightLine::createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal)
  {
      nav_msgs::msg::Path global_path;
     
      width = costmap_->getSizeInCellsX();
      height = costmap_->getSizeInCellsY();
      float map_resolution = costmap_->getResolution();


      std::vector<std::vector<int>> binary_map(height, std::vector<int>(width, 0));
      for (unsigned int row = 0; row < height; row++) {
          for (unsigned int col = 0; col < width; col++) {
              unsigned char cost = costmap_->getCost(col, row);
              binary_map[row][col] = (cost >= threshold) ? 1 : 0;
          }
      }

 
      nav_msgs::msg::OccupancyGrid occ_grid;
      occ_grid.header.stamp = node_->now();
      occ_grid.header.frame_id = global_frame_;
      occ_grid.info.resolution = map_resolution;
      occ_grid.info.width = width;
      occ_grid.info.height = height;
      occ_grid.info.origin.position.x = costmap_->getOriginX();
      occ_grid.info.origin.position.y = costmap_->getOriginY();
      occ_grid.info.origin.position.z = 0.0;
      occ_grid.info.origin.orientation.w = 1.0;
      occ_grid.data.resize(width * height);
      for (unsigned int row = 0; row < height; row++) {
          for (unsigned int col = 0; col < width; col++) {
              int idx = row * width + col;
              occ_grid.data[idx] = binary_map[row][col];
          }
      }
      binary_map_pub_->publish(occ_grid);

   
      unsigned int start_col, start_row, goal_col, goal_row;
      if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_col, start_row)) {
          RCLCPP_ERROR(node_->get_logger(), "Start pose is out of costmap bounds!");
          return global_path;
      }
      if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_col, goal_row)) {
          RCLCPP_ERROR(node_->get_logger(), "Goal pose is out of costmap bounds!");
          return global_path;
      }



    
      std::vector<PathNode> path_nodes = findPathAStar(
          static_cast<int>(start_col), static_cast<int>(start_row),
          static_cast<int>(goal_col),  static_cast<int>(goal_row),
          binary_map);

      if (path_nodes.empty()) {
          RCLCPP_ERROR(node_->get_logger(), "No path found using A*");
          return global_path;
      }

      global_path.header.stamp = node_->now();
      global_path.header.frame_id = global_frame_;
      for (const auto &node : path_nodes) {
          geometry_msgs::msg::PoseStamped pose;
          double wx, wy;
          costmap_->mapToWorld(node.x, node.y, wx, wy);
          pose.pose.position.x = wx;
          pose.pose.position.y = wy;
          pose.pose.orientation.w = 1.0;
          pose.header.stamp = node_->now();
          pose.header.frame_id = global_frame_;
          global_path.poses.push_back(pose);
      }
      return global_path;
  }
}  // namespace nav2_straightline_planner

PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)

#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
  void planPath();

  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid current_map_;
  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::PointStamped current_goal_;

  bool has_map_ = false;
  bool has_odom_ = false;
  bool has_goal_ = false;
};

#endif

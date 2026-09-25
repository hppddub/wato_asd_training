#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  robot::CostmapCore costmap_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  bool received_first_scan_ = false;
};

#endif

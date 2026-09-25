#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void controlLoop();
  bool turning_in_place_{false};

  robot::ControlCore control_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::Odometry current_odom_;
  bool has_path_ = false;
  bool has_odom_ = false;
  bool announced_goal_ = false;

  std::chrono::steady_clock::time_point last_path_time_;
  std::chrono::steady_clock::time_point last_odom_time_;
};

#endif

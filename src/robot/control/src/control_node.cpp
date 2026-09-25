#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"),
      control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", rclcpp::SensorDataQoS(),
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(
    const nav_msgs::msg::Path::SharedPtr path) {
  if (path->header.frame_id != "sim_world") {
    RCLCPP_WARN(this->get_logger(), "Path is not in sim_world");
    has_path_ = false;
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  if (!path->poses.empty()) {
    const auto &new_goal = path->poses.back().pose.position;

    if (current_path_.poses.empty()) {
      announced_goal_ = false;
    } else {
      const auto &old_goal = current_path_.poses.back().pose.position;
      if (std::hypot(new_goal.x - old_goal.x,
                     new_goal.y - old_goal.y) > 0.1) {
        announced_goal_ = false;
      }
    }
  }

  current_path_ = *path;
  has_path_ = !path->poses.empty();
  last_path_time_ = std::chrono::steady_clock::now();

  if (!has_path_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  RCLCPP_INFO(this->get_logger(),
              "Received /path with %zu waypoints",
              path->poses.size());
}

void ControlNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = *odom;
  has_odom_ = true;
  last_odom_time_ = std::chrono::steady_clock::now();
}

void ControlNode::controlLoop() {
  if (!has_path_ || !has_odom_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_path_time_ > std::chrono::seconds(3) ||
      now - last_odom_time_ > std::chrono::seconds(1)) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  const auto &robot = current_odom_.pose.pose;
  const auto &goal = current_path_.poses.back().pose.position;
  const double distance_to_goal =
      std::hypot(goal.x - robot.position.x,
                 goal.y - robot.position.y);

  if (distance_to_goal < 0.35) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

    if (!announced_goal_) {
      announced_goal_ = true;
      RCLCPP_INFO(this->get_logger(), "Reached goal; stopping");
    }
    return;
  }

  // Find the nearest waypoint on the path.
  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto &point = current_path_.poses[i].pose.position;
    const double distance =
        std::hypot(point.x - robot.position.x,
                   point.y - robot.position.y);

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }

  // Aim for the first waypoint at least 0.25 m ahead.
  size_t target_index = current_path_.poses.size() - 1;
  for (size_t i = nearest; i < current_path_.poses.size(); ++i) {
    const auto &point = current_path_.poses[i].pose.position;
    if (std::hypot(point.x - robot.position.x,
                   point.y - robot.position.y) >= 0.25) {
      target_index = i;
      break;
    }
  }

  const auto &target =
      current_path_.poses[target_index].pose.position;
  const double dx = target.x - robot.position.x;
  const double dy = target.y - robot.position.y;

  const auto &q = robot.orientation;
  const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_error = std::atan2(
      std::sin(desired_yaw - yaw),
      std::cos(desired_yaw - yaw));

  geometry_msgs::msg::Twist command;

  if (std::abs(yaw_error) > 0.65) {
  turning_in_place_ = true;
} else if (std::abs(yaw_error) < 0.35) {
  turning_in_place_ = false;
}

if (turning_in_place_) {
    // Face the route before driving forward.
    command.angular.z = std::clamp(yaw_error, -0.6, 0.6);
  } else {
    command.linear.x = std::min(0.18, 0.35 * distance_to_goal);

    // Pure Pursuit: steer toward the lookahead point.
    const double target_y_in_robot_frame =
        -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double squared_distance = dx * dx + dy * dy;
    const double curvature =
        2.0 * target_y_in_robot_frame /
        std::max(squared_distance, 0.01);

    command.angular.z = std::clamp(
        command.linear.x * curvature, -0.6, 0.6);
  }

  cmd_vel_pub_->publish(command);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

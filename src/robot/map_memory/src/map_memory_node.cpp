#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/costmap", 10,
          std::bind(&MapMemoryNode::costmapCallback, this,
                    std::placeholders::_1));

  odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom/filtered", rclcpp::SensorDataQoS(),
          std::bind(&MapMemoryNode::odomCallback, this,
                    std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());

  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 500;
  global_map_.info.height = 500;
  global_map_.info.origin.position.x = -25.0;
  global_map_.info.origin.position.y = -25.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(500 * 500, 0);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = *costmap;
  has_costmap_ = true;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  latest_odom_ = *odom;
  has_odom_ = true;
}

void MapMemoryNode::updateMap() {
  if (!has_costmap_ || !has_odom_ || !costmap_updated_) {
    return;
  }

  const auto &pose = latest_odom_.pose.pose;
  const auto &q = pose.orientation;

  // Convert the laser's quaternion orientation into a 2D heading.
  const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  int updated_cells = 0;
  const int local_width = static_cast<int>(latest_costmap_.info.width);
  const int local_height = static_cast<int>(latest_costmap_.info.height);
  const int world_width = static_cast<int>(global_map_.info.width);
  const int world_height = static_cast<int>(global_map_.info.height);

  for (int row = 0; row < local_height; ++row) {
    for (int col = 0; col < local_width; ++col) {
      const int local_index = row * local_width + col;
      const int cost = latest_costmap_.data[local_index];

      // Zero is empty in our current local costmap.
      if (cost <= 0) {
        continue;
      }

      // Centre of this square, measured from the laser.
      const double local_x =
          latest_costmap_.info.origin.position.x +
          (col + 0.5) * latest_costmap_.info.resolution;
      const double local_y =
          latest_costmap_.info.origin.position.y +
          (row + 0.5) * latest_costmap_.info.resolution;

      // Rotate and move the square into sim_world.
      const double world_x =
          pose.position.x + cos_yaw * local_x - sin_yaw * local_y;
      const double world_y =
          pose.position.y + sin_yaw * local_x + cos_yaw * local_y;

      const int world_col = static_cast<int>(std::floor(
          (world_x - global_map_.info.origin.position.x) /
          global_map_.info.resolution));
      const int world_row = static_cast<int>(std::floor(
          (world_y - global_map_.info.origin.position.y) /
          global_map_.info.resolution));

      if (world_col < 0 || world_col >= world_width ||
          world_row < 0 || world_row >= world_height) {
        continue;
      }

      const int world_index = world_row * world_width + world_col;
      if (cost > global_map_.data[world_index]) {
        global_map_.data[world_index] = cost;
        ++updated_cells;
      }
    }
  }

  global_map_.header.stamp = latest_costmap_.header.stamp;
  map_pub_->publish(global_map_);
  costmap_updated_ = false;

  if (!published_first_map_) {
    published_first_map_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Published /map in sim_world; updated %d cells",
                updated_cells);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

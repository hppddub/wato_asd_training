#include <functional>
#include <cmath>
#include <memory>
#include <algorithm>
#include <vector>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_(robot::CostmapCore(this->get_logger())) {
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10);
}

void CostmapNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  constexpr int width = 200;
  constexpr int height = 200;
  constexpr double resolution = 0.1;  // Metres per square
  constexpr double origin_x = -10.0;
  constexpr double origin_y = -10.0;

  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan->header;
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(width * height, 0);

  int occupied_cells = 0;
  std::vector<int> obstacle_indices;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double distance = scan->ranges[i];

    // Ignore missing, invalid, or out-of-range measurements.
    if (!std::isfinite(distance) ||
        distance < scan->range_min ||
        distance >= scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;
    const double x = distance * std::cos(angle);
    const double y = distance * std::sin(angle);
    const int cell_x =
        static_cast<int>(std::floor((x - origin_x) / resolution));
    const int cell_y =
        static_cast<int>(std::floor((y - origin_y) / resolution));

    if (cell_x < 0 || cell_x >= width ||
        cell_y < 0 || cell_y >= height) {
      continue;
    }

    const int index = cell_y * width + cell_x;
    if (grid.data[index] != 100) {
      grid.data[index] = 100;
      obstacle_indices.push_back(index);
      ++occupied_cells;
    }
  }

constexpr double hard_radius = 1.8;
constexpr double inflation_radius = 2.0;
const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius / resolution));

for (const int obstacle_index : obstacle_indices) {
  const int obstacle_x = obstacle_index % width;
  const int obstacle_y = obstacle_index / width;

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int cell_x = obstacle_x + dx;
      const int cell_y = obstacle_y + dy;

      if (cell_x < 0 || cell_x >= width ||
          cell_y < 0 || cell_y >= height) {
        continue;
      }

      const double distance =
          std::hypot(dx * resolution, dy * resolution);
      if (distance > inflation_radius) {
        continue;
      }

      const int cost =
    distance <= hard_radius
        ? 100
        : std::max(
              1,
              static_cast<int>(
                  99.0 * (inflation_radius - distance) /
                  (inflation_radius - hard_radius)));
      const int index = cell_y * width + cell_x;
      grid.data[index] = std::max<int>(grid.data[index], cost);
    }
  }
}


  costmap_pub_->publish(grid);

  if (!received_first_scan_) {
    received_first_scan_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Published /costmap: %d obstacle cells from %zu lidar readings",
                occupied_cells, scan->ranges.size());
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", rclcpp::SensorDataQoS(),
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  const bool first_map = !has_map_;
  current_map_ = *map;
  has_map_ = true;

  if (first_map) {
    RCLCPP_INFO(this->get_logger(),
                "Received /map: %u by %u cells in %s",
                map->info.width, map->info.height,
                map->header.frame_id.c_str());
  }
  if (has_goal_ && has_odom_) {
  planPath();
}


}

void PlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  const bool first_odom = !has_odom_;
  current_odom_ = *odom;
  has_odom_ = true;

  if (first_odom) {
    RCLCPP_INFO(this->get_logger(),
                "Received robot position (%.2f, %.2f) in %s",
                odom->pose.pose.position.x,
                odom->pose.pose.position.y,
                odom->header.frame_id.c_str());
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  if (goal->header.frame_id != "sim_world") {
    RCLCPP_WARN(this->get_logger(),
                "Goal must use sim_world; received %s",
                goal->header.frame_id.c_str());
    return;
  }

  current_goal_ = *goal;
  has_goal_ = true;
  RCLCPP_INFO(this->get_logger(),
              "Received goal (%.2f, %.2f) in sim_world",
              goal->point.x, goal->point.y);
  if (has_map_ && has_odom_) {
  planPath();
}

}

void PlannerNode::planPath() {
const auto stopRobot = [this]() {
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "sim_world";
  empty_path.header.stamp = current_map_.header.stamp;
  path_pub_->publish(empty_path);
};

  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  const double resolution = current_map_.info.resolution;

  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
      current_map_.data.size() !=
          static_cast<size_t>(width * height)) {
    RCLCPP_WARN(this->get_logger(), "Map is invalid");

  return;
  }

  // Convert a position in metres to an index in the map's data array.
  const auto toIndex = [&](double x, double y) -> int {
    const int col = static_cast<int>(std::floor(
        (x - current_map_.info.origin.position.x) / resolution));
    const int row = static_cast<int>(std::floor(
        (y - current_map_.info.origin.position.y) / resolution));

    if (col < 0 || col >= width || row < 0 || row >= height) {
      return -1;
    }
    return row * width + col;
  };

  const int start = toIndex(
      current_odom_.pose.pose.position.x,
      current_odom_.pose.pose.position.y);
  const int goal = toIndex(
      current_goal_.point.x, current_goal_.point.y);

  if (start == -1 || goal == -1) {
    RCLCPP_WARN(this->get_logger(),
                "Robot or goal is outside the map");

return;
  }

  // For now, cells with cost 30 or higher are blocked.
  if (current_map_.data[goal] > 100) {
    RCLCPP_WARN(this->get_logger(),
                "Goal is inside or too close to an obstacle");
return;
  }

  const auto heuristic = [&](int index) -> double {
    const int col = index % width;
    const int row = index / width;
    const int goal_col = goal % width;
    const int goal_row = goal / width;
    return std::abs(col - goal_col) + std::abs(row - goal_row);
  };

  const int total = width * height;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distance(total, infinity);
  std::vector<int> parent(total, -1);
  std::vector<bool> visited(total, false);

  // Each queue entry is {estimated total cost, cell index}.
  using QueueEntry = std::pair<double, int>;
  std::priority_queue<
      QueueEntry, std::vector<QueueEntry>,
      std::greater<QueueEntry>> open;

  distance[start] = 0.0;
  open.push({heuristic(start), start});

  const int step_x[4] = {1, -1, 0, 0};
  const int step_y[4] = {0, 0, 1, -1};

  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();

    if (visited[current]) {
      continue;
    }
    visited[current] = true;

    if (current == goal) {
      break;
    }

    const int current_col = current % width;
    const int current_row = current / width;

    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = current_col + step_x[direction];
      const int next_row = current_row + step_y[direction];

      if (next_col < 0 || next_col >= width ||
          next_row < 0 || next_row >= height) {
        continue;
      }

      const int next = next_row * width + next_col;
      const int cell_cost = current_map_.data[next];

      if (visited[next] || cell_cost > 100) {
        continue;
      }

      // Walking near an obstacle costs slightly more than open space.
      const double next_distance =
          distance[current] + 1.0 +
          10.0 * std::max(0, cell_cost) / 100.0;

      if (next_distance < distance[next]) {
        distance[next] = next_distance;
        parent[next] = current;
        open.push({
            next_distance + heuristic(next),
            next
        });
      }
    }
  }

  if (!visited[goal]) {
    RCLCPP_WARN(this->get_logger(),
            "No route to goal found (start cost %d, goal cost %d)",
            static_cast<int>(current_map_.data[start]),
            static_cast<int>(current_map_.data[goal]));
stopRobot();
return;
  }

  // Follow parent links backwards, then reverse into start-to-goal order.
  std::vector<int> cells;
  for (int at = goal; at != -1; at = parent[at]) {
    cells.push_back(at);
    if (at == start) {
      break;
    }
  }
  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";
  path.header.stamp = current_map_.header.stamp;

  for (const int cell : cells) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = path.header;
    waypoint.pose.position.x =
        current_map_.info.origin.position.x +
        (cell % width + 0.5) * resolution;
    waypoint.pose.position.y =
        current_map_.info.origin.position.y +
        (cell / width + 0.5) * resolution;
    waypoint.pose.orientation.w = 1.0;
    path.poses.push_back(waypoint);
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(),
              "Published /path with %zu waypoints",
              path.poses.size());
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

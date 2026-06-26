#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct TimedPoint
{
  double x {};
  double y {};
  double z {};
  rclcpp::Time stamp;
};

struct Point3
{
  double x {};
  double y {};
  double z {};
};

Point3 transformPoint(const Point3 & point, const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  q.normalize();
  const tf2::Vector3 p(point.x, point.y, point.z);
  const tf2::Vector3 t(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  const tf2::Vector3 out = tf2::quatRotate(q, p) + t;
  return {out.x(), out.y(), out.z()};
}

}  // namespace

class LocalHazardGrid : public rclcpp::Node
{
public:
  LocalHazardGrid()
  : Node("local_hazard_grid"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/rko_lio/frame_xyzi");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/local_hazard_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_m_ = declare_parameter<double>("width_m", 10.0);
    height_m_ = declare_parameter<double>("height_m", 10.0);
    history_duration_sec_ = declare_parameter<double>("history_duration_sec", 2.0);
    max_history_points_ = declare_parameter<int>("max_history_points", 250000);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.25);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.20);
    terrain_min_height_ = declare_parameter<double>("terrain_min_height", -0.75);
    terrain_max_height_ = declare_parameter<double>("terrain_max_height", 1.20);
    terrain_slope_hazard_deg_ = declare_parameter<double>("terrain_slope_hazard_deg", 22.0);
    terrain_step_hazard_m_ = declare_parameter<double>("terrain_step_hazard_m", 0.25);
    terrain_min_points_per_cell_ = declare_parameter<int>("terrain_min_points_per_cell", 5);
    terrain_neighbor_radius_cells_ = declare_parameter<int>("terrain_neighbor_radius_cells", 2);
    min_points_per_obstacle_cell_ = declare_parameter<int>("min_points_per_obstacle_cell", 1);
    gradient_radius_m_ = declare_parameter<double>("gradient_radius_m", 0.20);
    gradient_min_cost_ = declare_parameter<int>("gradient_min_cost", 8);
    occupied_value_ = declare_parameter<int>("occupied_value", 100);
    free_value_ = declare_parameter<int>("free_value", 0);
    clear_robot_radius_m_ = declare_parameter<double>("clear_robot_radius_m", 1.0);
    max_input_range_m_ = declare_parameter<double>("max_input_range_m", 6.0);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be > 0");
    }
    width_cells_ = std::max(1, static_cast<int>(std::ceil(width_m_ / resolution_)));
    height_cells_ = std::max(1, static_cast<int>(std::ceil(height_m_ / resolution_)));
    history_duration_sec_ = std::max(0.1, history_duration_sec_);
    max_history_points_ = std::max(1000, max_history_points_);
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    terrain_slope_hazard_deg_ = std::clamp(terrain_slope_hazard_deg_, 0.0, 89.0);
    terrain_step_hazard_m_ = std::max(0.0, terrain_step_hazard_m_);
    terrain_min_points_per_cell_ = std::max(1, terrain_min_points_per_cell_);
    terrain_neighbor_radius_cells_ = std::max(1, terrain_neighbor_radius_cells_);
    min_points_per_obstacle_cell_ = std::max(1, min_points_per_obstacle_cell_);
    gradient_radius_m_ = std::max(0.0, gradient_radius_m_);
    gradient_min_cost_ = std::clamp(gradient_min_cost_, 0, 99);
    occupied_value_ = std::clamp(occupied_value_, 1, 100);
    free_value_ = std::clamp(free_value_, 0, 100);

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_topic_, map_qos);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalHazardGrid::receiveCloud, this, std::placeholders::_1));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&LocalHazardGrid::publishGrid, this));

    RCLCPP_INFO(
      get_logger(), "Local hazard grid: %s -> %s frame=%s %.1fx%.1fm %.2fm/cell history=%.1fs",
      input_cloud_topic_.c_str(), output_map_topic_.c_str(), target_frame_.c_str(),
      width_m_, height_m_, resolution_, history_duration_sec_);
  }

private:
  void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->width == 0 || cloud->height == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        target_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for transform %s -> %s: %s",
        target_frame_.c_str(), cloud->header.frame_id.c_str(), ex.what());
      return;
    }

    const rclcpp::Time stamp = cloud->header.stamp.sec == 0 && cloud->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(cloud->header.stamp);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Point3 point{*iter_x, *iter_y, *iter_z};
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        if (max_input_range_m_ > 0.0 && std::hypot(point.x, point.y) > max_input_range_m_) {
          continue;
        }
        point = transformPoint(point, transform);
        points_.push_back({point.x, point.y, point.z, stamp});
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud is missing x/y/z fields: %s", ex.what());
      return;
    }
    pruneOldPoints(now());
  }

  void publishGrid()
  {
    const auto robot = lookupRobot();
    if (!robot.has_value()) {
      return;
    }
    pruneOldPoints(now());

    const double origin_x = robot->first - 0.5 * width_cells_ * resolution_;
    const double origin_y = robot->second - 0.5 * height_cells_ * resolution_;
    const size_t grid_size = static_cast<size_t>(width_cells_ * height_cells_);
    std::vector<int> obstacle_counts(grid_size, 0);
    std::vector<int> terrain_counts(grid_size, 0);
    std::vector<double> terrain_sum_z(grid_size, 0.0);
    std::vector<double> terrain_min_z(grid_size, std::numeric_limits<double>::infinity());
    std::vector<double> terrain_max_z(grid_size, -std::numeric_limits<double>::infinity());
    std::vector<std::pair<int, int>> hazard_cells;

    for (const auto & point : points_) {
      const int x = static_cast<int>(std::floor((point.x - origin_x) / resolution_));
      const int y = static_cast<int>(std::floor((point.y - origin_y) / resolution_));
      if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
        continue;
      }
      const size_t idx = static_cast<size_t>(y * width_cells_ + x);
      if (point.z >= terrain_min_height_ && point.z <= terrain_max_height_) {
        terrain_counts[idx] += 1;
        terrain_sum_z[idx] += point.z;
        terrain_min_z[idx] = std::min(terrain_min_z[idx], point.z);
        terrain_max_z[idx] = std::max(terrain_max_z[idx], point.z);
      }
      if (point.z >= min_obstacle_height_ && point.z <= max_obstacle_height_) {
        obstacle_counts[idx] += 1;
      }
    }

    nav_msgs::msg::OccupancyGrid map;
    stampMap(map);
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<uint32_t>(width_cells_);
    map.info.height = static_cast<uint32_t>(height_cells_);
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(grid_size, static_cast<int8_t>(free_value_));

    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (obstacle_counts[idx] >= min_points_per_obstacle_cell_) {
          markHazard(map, x, y, hazard_cells);
        }
      }
    }
    markTerrainHazards(map, terrain_counts, terrain_sum_z, terrain_min_z, terrain_max_z, hazard_cells);
    applyGradient(map, hazard_cells);
    clearRobot(map, origin_x, origin_y, robot->first, robot->second);
    map_pub_->publish(map);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published /local_hazard_map from %zu recent points; hazards=%zu",
      points_.size(), hazard_cells.size());
  }

  std::optional<std::pair<double, double>> lookupRobot()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
      return std::pair<double, double>{
        tf.transform.translation.x,
        tf.transform.translation.y};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for robot transform %s -> %s: %s",
        target_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void pruneOldPoints(const rclcpp::Time & stamp)
  {
    const auto cutoff = stamp - rclcpp::Duration::from_seconds(history_duration_sec_);
    while (!points_.empty() && points_.front().stamp < cutoff) {
      points_.pop_front();
    }
    while (static_cast<int>(points_.size()) > max_history_points_) {
      points_.pop_front();
    }
  }

  void markHazard(
    nav_msgs::msg::OccupancyGrid & map,
    const int x,
    const int y,
    std::vector<std::pair<int, int>> & hazard_cells) const
  {
    if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
      return;
    }
    const size_t idx = static_cast<size_t>(y * width_cells_ + x);
    if (map.data[idx] < occupied_value_) {
      map.data[idx] = static_cast<int8_t>(occupied_value_);
      hazard_cells.emplace_back(x, y);
    }
  }

  void markTerrainHazards(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int> & terrain_counts,
    const std::vector<double> & terrain_sum_z,
    const std::vector<double> & terrain_min_z,
    const std::vector<double> & terrain_max_z,
    std::vector<std::pair<int, int>> & hazard_cells) const
  {
    std::vector<double> mean_z(
      static_cast<size_t>(width_cells_ * height_cells_),
      std::numeric_limits<double>::quiet_NaN());
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (terrain_counts[idx] >= terrain_min_points_per_cell_) {
          mean_z[idx] = terrain_sum_z[idx] / static_cast<double>(terrain_counts[idx]);
        }
      }
    }

    constexpr double pi = 3.14159265358979323846;
    const double slope_threshold = std::tan(terrain_slope_hazard_deg_ * pi / 180.0);
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (!std::isfinite(mean_z[idx])) {
          continue;
        }
        bool hazard = terrain_max_z[idx] - terrain_min_z[idx] >= terrain_step_hazard_m_;
        for (int dy = -terrain_neighbor_radius_cells_; !hazard && dy <= terrain_neighbor_radius_cells_; ++dy) {
          for (int dx = -terrain_neighbor_radius_cells_; dx <= terrain_neighbor_radius_cells_; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width_cells_ || ny >= height_cells_) {
              continue;
            }
            const size_t neighbor_idx = static_cast<size_t>(ny * width_cells_ + nx);
            if (!std::isfinite(mean_z[neighbor_idx])) {
              continue;
            }
            const double horizontal_distance =
              resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
            const double slope = std::abs(mean_z[idx] - mean_z[neighbor_idx]) / horizontal_distance;
            if (slope >= slope_threshold) {
              hazard = true;
              break;
            }
          }
        }
        if (hazard) {
          markHazard(map, x, y, hazard_cells);
        }
      }
    }
  }

  void applyGradient(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::pair<int, int>> & hazard_cells) const
  {
    const int radius_cells = static_cast<int>(std::ceil(gradient_radius_m_ / resolution_));
    if (radius_cells <= 0) {
      return;
    }
    for (const auto & hazard : hazard_cells) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance > radius_cells) {
            continue;
          }
          const int x = hazard.first + dx;
          const int y = hazard.second + dy;
          if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
            continue;
          }
          const size_t idx = static_cast<size_t>(y * width_cells_ + x);
          if (map.data[idx] >= occupied_value_) {
            continue;
          }
          const double falloff = 1.0 - distance / static_cast<double>(radius_cells);
          const int cost = gradient_min_cost_ +
            static_cast<int>(std::round((occupied_value_ - gradient_min_cost_) * falloff));
          map.data[idx] = static_cast<int8_t>(
            std::max(static_cast<int>(map.data[idx]), std::clamp(cost, gradient_min_cost_, occupied_value_)));
        }
      }
    }
  }

  void clearRobot(
    nav_msgs::msg::OccupancyGrid & map,
    const double origin_x,
    const double origin_y,
    const double robot_x,
    const double robot_y) const
  {
    const int radius_cells = static_cast<int>(std::ceil(clear_robot_radius_m_ / resolution_));
    const int center_x = static_cast<int>(std::floor((robot_x - origin_x) / resolution_));
    const int center_y = static_cast<int>(std::floor((robot_y - origin_y) / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
          continue;
        }
        map.data[static_cast<size_t>(y * width_cells_ + x)] =
          static_cast<int8_t>(free_value_);
      }
    }
  }

  void stampMap(nav_msgs::msg::OccupancyGrid & map)
  {
    const int64_t now_ns = now().nanoseconds();
    map.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    map.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    map.header.frame_id = target_frame_;
    map.info.map_load_time = map.header.stamp;
  }

  std::string input_cloud_topic_;
  std::string output_map_topic_;
  std::string target_frame_;
  std::string robot_frame_;
  double resolution_ {};
  double width_m_ {};
  double height_m_ {};
  double history_duration_sec_ {};
  int max_history_points_ {};
  double publish_rate_hz_ {};
  double min_obstacle_height_ {};
  double max_obstacle_height_ {};
  double terrain_min_height_ {};
  double terrain_max_height_ {};
  double terrain_slope_hazard_deg_ {};
  double terrain_step_hazard_m_ {};
  int terrain_min_points_per_cell_ {};
  int terrain_neighbor_radius_cells_ {};
  int min_points_per_obstacle_cell_ {};
  double gradient_radius_m_ {};
  int gradient_min_cost_ {};
  int occupied_value_ {};
  int free_value_ {};
  double clear_robot_radius_m_ {};
  double max_input_range_m_ {};
  int width_cells_ {};
  int height_cells_ {};
  std::deque<TimedPoint> points_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalHazardGrid>());
  rclcpp::shutdown();
  return 0;
}

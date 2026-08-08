#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "dstar_lite.hpp"
#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kEpsilon = 1.0e-9;

bool costsEqual(double a, double b)
{
    if (a == b) {
        return true;
    }

    if (!std::isfinite(a) || !std::isfinite(b)) {
        return false;
    }

    return std::abs(a - b) <= kEpsilon;
}

}  // namespace

class TurtlebotDStarLiteNode : public rclcpp::Node {
public:
    TurtlebotDStarLiteNode()
    : Node("turtlebot_dstar_lite_node")
    {
        declareParameters();

        const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, sensor_qos,
            std::bind(
                &TurtlebotDStarLiteNode::odomCallback,
                this,
                std::placeholders::_1));
        scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, sensor_qos,
            std::bind(
                &TurtlebotDStarLiteNode::scanCallback,
                this,
                std::placeholders::_1));
        goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, 1,
            std::bind(
                &TurtlebotDStarLiteNode::goalCallback,
                this,
                std::placeholders::_1));

        belief_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            belief_topic_, 1);
        path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
        waypoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            waypoint_topic_, 1);

        initializeMap();
    }

private:
    struct RobotPose {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr belief_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;

    std::unique_ptr<Grid> belief_grid_;
    std::unique_ptr<DStarLite> planner_;

    double world_width_{10.0};
    double world_height_{10.0};
    double resolution_{0.05};
    double origin_x_{-5.0};
    double origin_y_{-5.0};
    double goal_x_{2.0};
    double goal_y_{0.0};
    double inflation_radius_{0.20};
    double scan_offset_x_{0.0};
    double scan_offset_y_{0.0};
    double scan_yaw_offset_{0.0};
    double waypoint_lookahead_{0.30};
    int map_width_{0};
    int map_height_{0};

    std::string map_frame_{"odom"};
    std::string odom_topic_{"/odom"};
    std::string scan_topic_{"/scan"};
    std::string goal_topic_{"/goal_pose"};
    std::string belief_topic_{"/belief_map"};
    std::string path_topic_{"/path"};
    std::string waypoint_topic_{"/waypoint"};

    bool have_odom_{false};
    RobotPose robot_pose_;
    Coord start_cell_{};
    Coord goal_cell_{};

    void declareParameters()
    {
        world_width_ = declare_parameter<double>("world_width", world_width_);
        world_height_ = declare_parameter<double>("world_height", world_height_);
        resolution_ = declare_parameter<double>("resolution", resolution_);
        origin_x_ = declare_parameter<double>("origin_x", origin_x_);
        origin_y_ = declare_parameter<double>("origin_y", origin_y_);
        goal_x_ = declare_parameter<double>("goal_x", goal_x_);
        goal_y_ = declare_parameter<double>("goal_y", goal_y_);
        inflation_radius_ = declare_parameter<double>(
            "inflation_radius", inflation_radius_);
        scan_offset_x_ = declare_parameter<double>(
            "scan_offset_x", scan_offset_x_);
        scan_offset_y_ = declare_parameter<double>(
            "scan_offset_y", scan_offset_y_);
        scan_yaw_offset_ = declare_parameter<double>(
            "scan_yaw_offset", scan_yaw_offset_);
        waypoint_lookahead_ = declare_parameter<double>(
            "waypoint_lookahead", waypoint_lookahead_);
        map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
        odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
        scan_topic_ = declare_parameter<std::string>("scan_topic", scan_topic_);
        goal_topic_ = declare_parameter<std::string>("goal_topic", goal_topic_);
        belief_topic_ = declare_parameter<std::string>(
            "belief_topic", belief_topic_);
        path_topic_ = declare_parameter<std::string>("path_topic", path_topic_);
        waypoint_topic_ = declare_parameter<std::string>(
            "waypoint_topic", waypoint_topic_);

        if (world_width_ <= 0.0 || world_height_ <= 0.0 || resolution_ <= 0.0) {
            throw std::runtime_error("world dimensions and resolution must be positive");
        }
    }

    void initializeMap()
    {
        map_width_ = static_cast<int>(std::ceil(world_width_ / resolution_));
        map_height_ = static_cast<int>(std::ceil(world_height_ / resolution_));
        belief_grid_ = std::make_unique<Grid>(map_width_, map_height_, resolution_);

        if (!worldToGrid(goal_x_, goal_y_, goal_cell_)) {
            throw std::runtime_error("Goal is outside the configured map");
        }
    }

    bool worldToGrid(double world_x, double world_y, Coord& cell) const
    {
        const int grid_x = static_cast<int>(std::floor(
            (world_x - origin_x_) / resolution_));
        const int grid_y = static_cast<int>(std::floor(
            (world_y - origin_y_) / resolution_));

        if (grid_x < 0 || grid_x >= map_width_ ||
            grid_y < 0 || grid_y >= map_height_) {
            return false;
        }

        cell = Coord{grid_x, grid_y};
        return true;
    }

    geometry_msgs::msg::Point gridToWorld(const Coord& cell) const
    {
        geometry_msgs::msg::Point point;
        point.x = origin_x_ + (static_cast<double>(cell.x) + 0.5) * resolution_;
        point.y = origin_y_ + (static_cast<double>(cell.y) + 0.5) * resolution_;
        point.z = 0.0;
        return point;
    }

    void createPlanner(const Coord& start)
    {
        start_cell_ = start;
        planner_ = std::make_unique<DStarLite>(*belief_grid_, start_cell_, goal_cell_);
        planner_->computeShortestPath();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring odometry in frame '%s'; expected map_frame '%s'",
                msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
        }

        const auto& position = msg->pose.pose.position;
        const auto& orientation = msg->pose.pose.orientation;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
            !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring invalid odometry pose");
            return;
        }

        tf2::Quaternion quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w);
        if (quaternion.length2() < kEpsilon) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring zero odometry quaternion");
            return;
        }
        quaternion.normalize();

        robot_pose_.x = position.x;
        robot_pose_.y = position.y;
        robot_pose_.yaw = tf2::getYaw(quaternion);
        have_odom_ = true;

        Coord new_start;
        if (!worldToGrid(robot_pose_.x, robot_pose_.y, new_start)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Robot is outside the configured map: (%.2f, %.2f)",
                robot_pose_.x, robot_pose_.y);
            return;
        }

        if (!planner_) {
            createPlanner(new_start);
            publishPath();
            return;
        }

        if (!(new_start == start_cell_)) {
            start_cell_ = new_start;
            planner_->moveStart(start_cell_);
            planner_->computeShortestPath();
            publishPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
            RCLCPP_WARN(
                get_logger(), "Ignoring goal in frame '%s'; expected map_frame '%s'",
                msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
        }

        Coord new_goal;
        if (!worldToGrid(msg->pose.position.x, msg->pose.position.y, new_goal)) {
            RCLCPP_WARN(get_logger(), "Ignoring goal outside the configured map");
            return;
        }

        goal_cell_ = new_goal;
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;

        if (have_odom_) {
            Coord current_start;
            if (worldToGrid(robot_pose_.x, robot_pose_.y, current_start)) {
                createPlanner(current_start);
                publishPath();
            }
        }
    }

    bool applyObservation(const Coord& cell, int observed_state)
    {
        const int old_state = belief_grid_->state(cell);
        if (old_state == observed_state) {
            return false;
        }

        const double old_cost = belief_grid_->traversalCost(cell);
        planner_->updateCellState(cell, observed_state);
        const double new_cost = belief_grid_->traversalCost(cell);
        return !costsEqual(old_cost, new_cost);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!have_odom_ || !planner_) {
            return;
        }

        if (msg->range_max <= msg->range_min || msg->angle_increment == 0.0F) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring invalid laser scan metadata");
            return;
        }

        const double cos_yaw = std::cos(robot_pose_.yaw);
        const double sin_yaw = std::sin(robot_pose_.yaw);
        const double sensor_x = robot_pose_.x +
            cos_yaw * scan_offset_x_ - sin_yaw * scan_offset_y_;
        const double sensor_y = robot_pose_.y +
            sin_yaw * scan_offset_x_ + cos_yaw * scan_offset_y_;
        const double sensor_yaw = robot_pose_.yaw + scan_yaw_offset_;
        const double ray_step = 0.5 * resolution_;

        std::vector<Coord> free_cells;
        std::vector<Coord> obstacle_cells;
        free_cells.reserve(msg->ranges.size() * 20U);
        obstacle_cells.reserve(msg->ranges.size());

        for (std::size_t index = 0; index < msg->ranges.size(); ++index) {
            const double measured_range = static_cast<double>(msg->ranges[index]);
            bool obstacle_hit = false;
            double ray_length = 0.0;

            if (std::isfinite(measured_range)) {
                if (measured_range < msg->range_min) {
                    continue;
                }
                ray_length = std::min(measured_range, static_cast<double>(msg->range_max));
                obstacle_hit = measured_range < msg->range_max;
            } else if (std::isinf(measured_range) && measured_range > 0.0) {
                ray_length = msg->range_max;
            } else {
                continue;
            }

            const double scan_angle = static_cast<double>(msg->angle_min) +
                static_cast<double>(index) * static_cast<double>(msg->angle_increment);
            const double world_angle = sensor_yaw + scan_angle;
            const double dx = ray_length * std::cos(world_angle);
            const double dy = ray_length * std::sin(world_angle);
            const int steps = std::max(
                1, static_cast<int>(std::ceil(std::hypot(dx, dy) / ray_step)));

            for (int step = 0; step < steps; ++step) {
                const double alpha = static_cast<double>(step) / static_cast<double>(steps);
                Coord free_cell;
                if (worldToGrid(sensor_x + alpha * dx, sensor_y + alpha * dy, free_cell)) {
                    free_cells.push_back(free_cell);
                }
            }

            Coord endpoint;
            if (!worldToGrid(sensor_x + dx, sensor_y + dy, endpoint)) {
                continue;
            }

            if (obstacle_hit) {
                obstacle_cells.push_back(endpoint);
            } else {
                free_cells.push_back(endpoint);
            }
        }

        std::vector<bool> occupied_mask(
            static_cast<std::size_t>(map_width_) * static_cast<std::size_t>(map_height_),
            false);
        const double inflation_radius_cells = inflation_radius_ / resolution_;
        const int inflation_extent = static_cast<int>(std::ceil(inflation_radius_cells));
        const double inflation_radius_squared =
            inflation_radius_cells * inflation_radius_cells;

        for (const Coord& obstacle : obstacle_cells) {
            for (int dy = -inflation_extent; dy <= inflation_extent; ++dy) {
                for (int dx = -inflation_extent; dx <= inflation_extent; ++dx) {
                    if (static_cast<double>(dx * dx + dy * dy) > inflation_radius_squared) {
                        continue;
                    }

                    const Coord inflated{obstacle.x + dx, obstacle.y + dy};
                    if (!belief_grid_->inBounds(inflated) || inflated == start_cell_) {
                        continue;
                    }

                    const std::size_t linear_index = static_cast<std::size_t>(inflated.x) +
                        static_cast<std::size_t>(inflated.y) *
                        static_cast<std::size_t>(map_width_);
                    occupied_mask[linear_index] = true;
                }
            }
        }

        bool cost_changed = false;
        for (const Coord& cell : free_cells) {
            const std::size_t linear_index = static_cast<std::size_t>(cell.x) +
                static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(map_width_);
            if (!occupied_mask[linear_index]) {
                cost_changed = applyObservation(cell, 0) || cost_changed;
            }
        }

        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const std::size_t linear_index = static_cast<std::size_t>(x) +
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(map_width_);
                if (occupied_mask[linear_index]) {
                    cost_changed = applyObservation(Coord{x, y}, 1) || cost_changed;
                }
            }
        }

        if (cost_changed) {
            planner_->computeShortestPath();
        }

        publishBeliefMap();
        publishPath();
    }

    geometry_msgs::msg::Quaternion orientationTo(
        const Coord& from, const Coord& to) const
    {
        const auto from_point = gridToWorld(from);
        const auto to_point = gridToWorld(to);
        const double yaw = std::atan2(
            to_point.y - from_point.y, to_point.x - from_point.x);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        return tf2::toMsg(orientation);
    }

    void publishBeliefMap()
    {
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = now();
        map.header.frame_id = map_frame_;
        map.info.map_load_time = map.header.stamp;
        map.info.resolution = static_cast<float>(resolution_);
        map.info.width = static_cast<std::uint32_t>(map_width_);
        map.info.height = static_cast<std::uint32_t>(map_height_);
        map.info.origin.position.x = origin_x_;
        map.info.origin.position.y = origin_y_;
        map.info.origin.orientation.w = 1.0;
        map.data.resize(static_cast<std::size_t>(map_width_) *
            static_cast<std::size_t>(map_height_));

        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const Coord cell{x, y};
                const std::size_t linear_index = static_cast<std::size_t>(x) +
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(map_width_);
                const int state = belief_grid_->state(cell);
                map.data[linear_index] = state < 0 ? -1 : (state == 0 ? 0 : 100);
            }
        }

        belief_publisher_->publish(map);
    }

    void publishPath()
    {
        nav_msgs::msg::Path path_message;
        path_message.header.stamp = now();
        path_message.header.frame_id = map_frame_;

        if (!planner_) {
            path_publisher_->publish(path_message);
            return;
        }

        const std::vector<Coord> path = planner_->extractPath();
        path_message.poses.reserve(path.size());
        for (std::size_t index = 0; index < path.size(); ++index) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_message.header;
            pose.pose.position = gridToWorld(path[index]);
            pose.pose.orientation = index + 1U < path.size()
                ? orientationTo(path[index], path[index + 1U])
                : geometry_msgs::msg::Quaternion{};
            if (index + 1U == path.size()) {
                pose.pose.orientation.w = 1.0;
            }
            path_message.poses.push_back(pose);
        }

        path_publisher_->publish(path_message);
        publishWaypoint(path, path_message.header);
    }

    void publishWaypoint(
        const std::vector<Coord>& path,
        const std_msgs::msg::Header& header)
    {
        if (path.size() < 2U) {
            return;
        }

        std::size_t waypoint_index = 1U;
        const auto start_point = gridToWorld(path.front());
        for (std::size_t index = 1U; index < path.size(); ++index) {
            const auto candidate = gridToWorld(path[index]);
            if (std::hypot(
                    candidate.x - start_point.x,
                    candidate.y - start_point.y) > waypoint_lookahead_) {
                break;
            }
            waypoint_index = index;
        }

        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header = header;
        waypoint.pose.position = gridToWorld(path[waypoint_index]);
        waypoint.pose.orientation = waypoint_index + 1U < path.size()
            ? orientationTo(path[waypoint_index], path[waypoint_index + 1U])
            : orientationTo(path[waypoint_index - 1U], path[waypoint_index]);
        waypoint_publisher_->publish(waypoint);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotDStarLiteNode>());
    rclcpp::shutdown();
    return 0;
}

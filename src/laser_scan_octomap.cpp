#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// Maps a standard ROS LaserScan into both an OctoMap and a persistent 2D
// occupancy projection.  Unlike obstacle_distance_octomap_node, this node
// deliberately has no PX4 messages or frame conversion: the scan frame is
// transformed to map_frame through TF exactly as a normal ROS lidar would be.
class LaserScanOctomapNode : public rclcpp::Node
{
public:
    LaserScanOctomapNode()
    : Node("laser_scan_octomap_node")
    {
        const double map_resolution = declare_parameter<double>("resolution", 0.10);
        map_frame_ = declare_parameter<std::string>("map_frame", "odom");
        scan_topic_ = declare_parameter<std::string>("scan_topic", "/mock_lidar/scan");
        slice_topic_ = declare_parameter<std::string>("slice_topic", "/voxel_slice");
        use_latest_tf_ = declare_parameter<bool>("use_latest_tf", false);
        world_width_ = declare_parameter<double>("world_width", 12.0);
        world_height_ = declare_parameter<double>("world_height", 10.0);
        origin_x_ = declare_parameter<double>("origin_x", -6.0);
        origin_y_ = declare_parameter<double>("origin_y", -5.0);

        if (map_resolution <= 0.0 || world_width_ <= 0.0 || world_height_ <= 0.0) {
            throw std::runtime_error("resolution, world_width, and world_height must be positive");
        }

        octree_ = std::make_unique<octomap::OcTree>(map_resolution);
        slice_width_ = static_cast<int>(std::ceil(world_width_ / map_resolution));
        slice_height_ = static_cast<int>(std::ceil(world_height_ / map_resolution));
        projected_map_.assign(static_cast<std::size_t>(slice_width_ * slice_height_), -1);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, sensor_qos,
            std::bind(&LaserScanOctomapNode::scanCallback, this, std::placeholders::_1));

        octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", 1);
        slice_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(slice_topic_, 1);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mapping_pose", 1);
        scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/mapping_scan", 1);

        RCLCPP_INFO(
            get_logger(), "Mapping %s into %s; publishing %s and /octomap_binary",
            scan_topic_.c_str(), map_frame_.c_str(), slice_topic_.c_str());
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring LaserScan without a frame_id");
            return;
        }
        if (msg->ranges.empty() || msg->range_max <= msg->range_min || msg->range_min < 0.0F) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring invalid LaserScan on %s", scan_topic_.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = use_latest_tf_
                ? tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id, tf2::TimePointZero,
                tf2::durationFromSec(0.10))
                : tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id, msg->header.stamp,
                tf2::durationFromSec(0.10));
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for TF %s -> %s at the scan timestamp: %s",
                map_frame_.c_str(), msg->header.frame_id.c_str(), error.what());
            return;
        }

        Eigen::Quaterniond orientation{
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z};
        if (orientation.norm() < 1e-6) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring scan with an invalid TF rotation");
            return;
        }
        orientation.normalize();
        const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
        const Eigen::Vector3d sensor_origin_enu{
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z};
        const octomap::point3d sensor_origin(
            static_cast<float>(sensor_origin_enu.x()),
            static_cast<float>(sensor_origin_enu.y()),
            static_cast<float>(sensor_origin_enu.z()));

        octomap::Pointcloud obstacle_points;
        std::vector<octomap::KeyRay> clear_rays;
        std::vector<Eigen::Vector2d> clear_endpoints_xy;
        std::vector<Eigen::Vector2d> obstacle_endpoints_xy;
        std::vector<Eigen::Vector3d> obstacle_endpoints;
        obstacle_points.reserve(msg->ranges.size());

        for (std::size_t index = 0; index < msg->ranges.size(); ++index) {
            const float reported_range = msg->ranges[index];
            const bool is_clear_ray = std::isinf(reported_range) && reported_range > 0.0F;
            const bool is_hit = std::isfinite(reported_range) &&
                reported_range >= msg->range_min && reported_range <= msg->range_max;
            if (!is_clear_ray && !is_hit) {
                // NaN, too-close returns, and values outside LaserScan's
                // advertised range are unknown, not obstacles or free space.
                continue;
            }

            const double range_m = is_clear_ray
                ? static_cast<double>(msg->range_max)
                : static_cast<double>(reported_range);
            const double angle = static_cast<double>(msg->angle_min) +
                static_cast<double>(index) * static_cast<double>(msg->angle_increment);
            // LaserScan is ROS FLU: +X forward, +Y left, and positive angles
            // counter-clockwise. TF performs the only frame conversion.
            const Eigen::Vector3d endpoint_sensor{
                range_m * std::cos(angle), range_m * std::sin(angle), 0.0};
            const Eigen::Vector3d endpoint = sensor_origin_enu + rotation * endpoint_sensor;
            const octomap::point3d octomap_endpoint(
                static_cast<float>(endpoint.x()),
                static_cast<float>(endpoint.y()),
                static_cast<float>(endpoint.z()));

            if (is_clear_ray) {
                octomap::KeyRay ray;
                if (octree_->computeRayKeys(sensor_origin, octomap_endpoint, ray)) {
                    clear_rays.push_back(std::move(ray));
                }
                clear_endpoints_xy.emplace_back(endpoint.x(), endpoint.y());
            } else {
                obstacle_points.push_back(octomap_endpoint);
                obstacle_endpoints_xy.emplace_back(endpoint.x(), endpoint.y());
                obstacle_endpoints.push_back(endpoint);
            }
        }

        // Clear rays go in first, then this scan's hit endpoints take priority.
        for (const octomap::KeyRay& ray : clear_rays) {
            for (const octomap::OcTreeKey& key : ray) {
                octree_->updateNode(key, false);
            }
        }
        if (obstacle_points.size() > 0U) {
            octree_->insertPointCloud(obstacle_points, sensor_origin, -1.0, true, true);
        }
        octree_->updateInnerOccupancy();

        const Eigen::Vector2d sensor_origin_xy = sensor_origin_enu.head<2>();
        for (const Eigen::Vector2d& endpoint_xy : clear_endpoints_xy) {
            markProjectedRayFree(sensor_origin_xy, endpoint_xy);
        }
        for (const Eigen::Vector2d& endpoint_xy : obstacle_endpoints_xy) {
            markProjectedRayFree(sensor_origin_xy, endpoint_xy);
            markProjectedObstacleWithBuffer(endpoint_xy);
        }

        const rclcpp::Time stamp(msg->header.stamp);
        publishMappingPose(transform, stamp);
        publishMap(stamp);
        publishProjectedMap(stamp);
        publishTransformedScan(obstacle_endpoints, stamp);
    }

    bool worldToProjectedCell(double world_x, double world_y, int& cell_x, int& cell_y) const
    {
        cell_x = static_cast<int>(std::floor((world_x - origin_x_) / octree_->getResolution()));
        cell_y = static_cast<int>(std::floor((world_y - origin_y_) / octree_->getResolution()));
        return cell_x >= 0 && cell_x < slice_width_ && cell_y >= 0 && cell_y < slice_height_;
    }

    void markProjectedRayFree(const Eigen::Vector2d& origin_xy, const Eigen::Vector2d& endpoint_xy)
    {
        const Eigen::Vector2d ray = endpoint_xy - origin_xy;
        const int samples = std::max(
            1, static_cast<int>(std::ceil(ray.norm() / (0.5 * octree_->getResolution()))));
        for (int sample = 0; sample < samples; ++sample) {
            const Eigen::Vector2d point = origin_xy +
                (static_cast<double>(sample) / samples) * ray;
            int cell_x = 0;
            int cell_y = 0;
            if (worldToProjectedCell(point.x(), point.y(), cell_x, cell_y)) {
                projected_map_[static_cast<std::size_t>(cell_x + cell_y * slice_width_)] = 0;
            }
        }
    }

    void markProjectedObstacleWithBuffer(const Eigen::Vector2d& endpoint_xy)
    {
        int center_x = 0;
        int center_y = 0;
        if (!worldToProjectedCell(endpoint_xy.x(), endpoint_xy.y(), center_x, center_y)) {
            return;
        }

        // Inflate each observed endpoint by a fixed 3x3 occupancy kernel.
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int cell_x = center_x + dx;
                const int cell_y = center_y + dy;
                if (cell_x >= 0 && cell_x < slice_width_ &&
                    cell_y >= 0 && cell_y < slice_height_)
                {
                    projected_map_[static_cast<std::size_t>(cell_x + cell_y * slice_width_)] = 100;
                }
            }
        }
    }

    void publishMappingPose(
        const geometry_msgs::msg::TransformStamped& transform, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        pose_publisher_->publish(pose);
    }

    void publishMap(const rclcpp::Time& stamp)
    {
        octomap_msgs::msg::Octomap map;
        if (!octomap_msgs::binaryMapToMsg(*octree_, map)) {
            RCLCPP_ERROR(get_logger(), "Failed to serialize the OctoMap");
            return;
        }
        map.header.stamp = stamp;
        map.header.frame_id = map_frame_;
        octomap_publisher_->publish(map);
    }

    void publishProjectedMap(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::OccupancyGrid slice;
        slice.header.stamp = stamp;
        slice.header.frame_id = map_frame_;
        slice.info.map_load_time = stamp;
        slice.info.resolution = static_cast<float>(octree_->getResolution());
        slice.info.width = static_cast<std::uint32_t>(slice_width_);
        slice.info.height = static_cast<std::uint32_t>(slice_height_);
        slice.info.origin.position.x = origin_x_;
        slice.info.origin.position.y = origin_y_;
        slice.info.origin.position.z = 0.0;
        slice.info.origin.orientation.w = 1.0;
        slice.data = projected_map_;
        slice_publisher_->publish(slice);
    }

    void publishTransformedScan(
        const std::vector<Eigen::Vector3d>& points, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = map_frame_;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
        for (const Eigen::Vector3d& point : points) {
            *x_iterator = static_cast<float>(point.x());
            *y_iterator = static_cast<float>(point.y());
            *z_iterator = static_cast<float>(point.z());
            ++x_iterator;
            ++y_iterator;
            ++z_iterator;
        }
        scan_publisher_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<octomap::OcTree> octree_;
    std::string map_frame_;
    std::string scan_topic_;
    std::string slice_topic_;
    bool use_latest_tf_ = false;
    double world_width_ = 0.0;
    double world_height_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    int slice_width_ = 0;
    int slice_height_ = 0;
    std::vector<std::int8_t> projected_map_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanOctomapNode>());
    rclcpp::shutdown();
    return 0;
}

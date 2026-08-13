#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

class ObstacleDistanceOctomapNode : public rclcpp::Node
{
public:
    ObstacleDistanceOctomapNode()
    : Node("obstacle_distance_octomap_node")
    {
        const double map_resolution = declare_parameter<double>("resolution", 0.10);
        map_frame_ = declare_parameter<std::string>("map_frame", "odom");
        slice_topic_ = declare_parameter<std::string>("slice_topic", "/voxel_slice");
        world_width_ = declare_parameter<double>("world_width", 20.5);
        world_height_ = declare_parameter<double>("world_height", 17.0);
        origin_x_ = declare_parameter<double>("origin_x", -6.5);
        origin_y_ = declare_parameter<double>("origin_y", -3.0);
        sensor_offset_flu_.x() = declare_parameter<double>("sensor_offset_x", 0.12);
        sensor_offset_flu_.y() = declare_parameter<double>("sensor_offset_y", 0.0);
        sensor_offset_flu_.z() = declare_parameter<double>("sensor_offset_z", 0.315);

        octree_ = std::make_unique<octomap::OcTree>(map_resolution);
        slice_width_ = static_cast<int>(std::ceil(world_width_ / map_resolution));
        slice_height_ = static_cast<int>(std::ceil(world_height_ / map_resolution));
        projected_map_.assign(static_cast<std::size_t>(slice_width_ * slice_height_), -1);

        R_ned_to_enu_ << 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0;
        R_flu_to_frd_ << 1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;

        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        odometry_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", sensor_qos,
            std::bind(&ObstacleDistanceOctomapNode::odometryCallback, this, std::placeholders::_1));
        obstacle_subscription_ = create_subscription<px4_msgs::msg::ObstacleDistance>(
            "/fmu/out/obstacle_distance", sensor_qos,
            std::bind(&ObstacleDistanceOctomapNode::obstacleCallback, this, std::placeholders::_1));

        octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", 1);
        slice_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(slice_topic_, 1);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mapping_pose", 1);
        scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/mapping_scan", 1);
    }

private:
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        const Eigen::Vector3d position_ned{
            static_cast<double>(msg->position[0]),
            static_cast<double>(msg->position[1]),
            static_cast<double>(msg->position[2])};
        Eigen::Quaterniond q_ned_frd{
            static_cast<double>(msg->q[0]),
            static_cast<double>(msg->q[1]),
            static_cast<double>(msg->q[2]),
            static_cast<double>(msg->q[3])};

        if (q_ned_frd.norm() < 1e-6) {
            return;
        }

        q_ned_frd.normalize();
        position_enu_ = R_ned_to_enu_ * position_ned;
        R_enu_flu_ = R_ned_to_enu_ * q_ned_frd.toRotationMatrix() * R_flu_to_frd_;
        have_odometry_ = true;

        // Publish the exact pose used to register every scan. This gives RViz
        // a world-frame yaw reference alongside the fixed occupancy grid.
        const Eigen::Quaterniond q_enu_flu(R_enu_flu_);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = map_frame_;
        pose.pose.position.x = position_enu_.x();
        pose.pose.position.y = position_enu_.y();
        pose.pose.position.z = position_enu_.z();
        pose.pose.orientation.w = q_enu_flu.w();
        pose.pose.orientation.x = q_enu_flu.x();
        pose.pose.orientation.y = q_enu_flu.y();
        pose.pose.orientation.z = q_enu_flu.z();
        pose_publisher_->publish(pose);
    }

    void obstacleCallback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg)
    {
        if (!have_odometry_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for vehicle odometry before inserting LiDAR points");
            return;
        }
        if (msg->frame != px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring ObstacleDistance frame %u; expected BODY_FRD (%u)",
                msg->frame, px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD);
            return;
        }

        constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
        constexpr std::uint16_t kUnknownDistance = std::numeric_limits<std::uint16_t>::max();
        const std::uint32_t clear_distance = static_cast<std::uint32_t>(msg->max_distance) + 1U;

        const Eigen::Vector3d sensor_origin_enu =
            position_enu_ + R_enu_flu_ * sensor_offset_flu_;
        const octomap::point3d sensor_origin(
            static_cast<float>(sensor_origin_enu.x()),
            static_cast<float>(sensor_origin_enu.y()),
            static_cast<float>(sensor_origin_enu.z()));

        octomap::Pointcloud obstacle_points;
        std::vector<octomap::KeyRay> clear_rays;
        std::vector<Eigen::Vector2d> clear_endpoints_xy;
        std::vector<Eigen::Vector2d> obstacle_endpoints_xy;
        std::vector<Eigen::Vector3d> obstacle_endpoints_enu;

        for (std::size_t index = 0; index < msg->distances.size(); ++index) {
            const std::uint16_t distance_cm = msg->distances[index];
            if (distance_cm == kUnknownDistance) {
                continue;
            }

            const bool is_clear_ray = static_cast<std::uint32_t>(distance_cm) == clear_distance;
            if (!is_clear_ray &&
                (distance_cm < msg->min_distance || distance_cm > msg->max_distance)) {
                continue;
            }

            const double range_m = is_clear_ray
                ? static_cast<double>(msg->max_distance) / 100.0
                : static_cast<double>(distance_cm) / 100.0;
            const double angle_frd =
                (static_cast<double>(msg->angle_offset) +
                 static_cast<double>(index) * static_cast<double>(msg->increment)) * kDegreesToRadians;

            // ObstacleDistance angles are clockwise in BODY_FRD. Convert the
            // sample to FLU before transforming it to the ENU OctoMap frame.
            const Eigen::Vector3d endpoint_flu{
                range_m * std::cos(angle_frd), -range_m * std::sin(angle_frd), 0.0};
            const Eigen::Vector3d endpoint_enu = sensor_origin_enu + R_enu_flu_ * endpoint_flu;
            const octomap::point3d endpoint(
                static_cast<float>(endpoint_enu.x()),
                static_cast<float>(endpoint_enu.y()),
                static_cast<float>(endpoint_enu.z()));

            if (is_clear_ray) {
                octomap::KeyRay ray;
                if (octree_->computeRayKeys(sensor_origin, endpoint, ray)) {
                    clear_rays.push_back(std::move(ray));
                }
                clear_endpoints_xy.emplace_back(endpoint_enu.x(), endpoint_enu.y());
            } else {
                obstacle_points.push_back(endpoint);
                obstacle_endpoints_xy.emplace_back(endpoint_enu.x(), endpoint_enu.y());
                obstacle_endpoints_enu.push_back(endpoint_enu);
            }
        }

        // Apply clear rays first so valid obstacle hits in this scan take precedence.
        for (const octomap::KeyRay& ray : clear_rays) {
            for (const octomap::OcTreeKey& key : ray) {
                octree_->updateNode(key, false);
            }
        }

        if (obstacle_points.size() > 0U) {
            octree_->insertPointCloud(obstacle_points, sensor_origin, -1.0, true, true);
        }
        octree_->updateInnerOccupancy();

        // Project all observations into XY for the planner. Free rays are
        // applied first; obstacle hits then receive a 3x3-cell dilation.
        const Eigen::Vector2d sensor_origin_xy = sensor_origin_enu.head<2>();
        for (const Eigen::Vector2d& endpoint_xy : clear_endpoints_xy) {
            markProjectedRayFree(sensor_origin_xy, endpoint_xy);
        }
        for (const Eigen::Vector2d& endpoint_xy : obstacle_endpoints_xy) {
            markProjectedRayFree(sensor_origin_xy, endpoint_xy);
            markProjectedObstacleWithBuffer(endpoint_xy);
        }

        octomap_msgs::msg::Octomap map;
        if (!octomap_msgs::binaryMapToMsg(*octree_, map)) {
            RCLCPP_ERROR(get_logger(), "Failed to serialize the OctoMap");
            return;
        }
        map.header.stamp = now();
        map.header.frame_id = map_frame_;
        octomap_publisher_->publish(map);
        publishProjectedMap();
        publishTransformedScan(obstacle_endpoints_enu);
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
            if (!worldToProjectedCell(point.x(), point.y(), cell_x, cell_y)) {
                continue;
            }
            projected_map_[static_cast<std::size_t>(cell_x + cell_y * slice_width_)] = 0;
        }
    }

    void markProjectedObstacleWithBuffer(const Eigen::Vector2d& endpoint_xy)
    {
        int center_x = 0;
        int center_y = 0;
        if (!worldToProjectedCell(endpoint_xy.x(), endpoint_xy.y(), center_x, center_y)) {
            return;
        }

        // A 3x3 convolution kernel around each observed obstacle endpoint.
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int cell_x = center_x + dx;
                const int cell_y = center_y + dy;
                if (cell_x < 0 || cell_x >= slice_width_ ||
                    cell_y < 0 || cell_y >= slice_height_) {
                    continue;
                }
                projected_map_[static_cast<std::size_t>(cell_x + cell_y * slice_width_)] = 100;
            }
        }
    }

    void publishProjectedMap()
    {
        nav_msgs::msg::OccupancyGrid slice;
        slice.header.stamp = now();
        slice.header.frame_id = map_frame_;
        slice.info.map_load_time = slice.header.stamp;
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

    void publishTransformedScan(const std::vector<Eigen::Vector3d>& points)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = now();
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

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;

    std::unique_ptr<octomap::OcTree> octree_;
    Eigen::Matrix3d R_ned_to_enu_;
    Eigen::Matrix3d R_flu_to_frd_;
    Eigen::Matrix3d R_enu_flu_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d position_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d sensor_offset_flu_ = Eigen::Vector3d::Zero();
    std::string map_frame_;
    std::string slice_topic_;
    double world_width_ = 0.0;
    double world_height_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    int slice_width_ = 0;
    int slice_height_ = 0;
    std::vector<std::int8_t> projected_map_;
    bool have_odometry_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDistanceOctomapNode>());
    rclcpp::shutdown();
    return 0;
}

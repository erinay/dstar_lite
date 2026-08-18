#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
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
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Inserts a standard ROS PointCloud2 into a persistent 3D OctoMap. The cloud
// is transformed at its measurement stamp, so it works with the raw Gazebo
// 3D LiDAR bridge and a normal TF tree alike.
class PointcloudOctomapNode : public rclcpp::Node
{
public:
    PointcloudOctomapNode()
    : Node("pointcloud_octomap_node")
    {
        const double resolution = declare_parameter<double>("resolution", 0.10);
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/sim_lidar/points");
        slice_topic_ = declare_parameter<std::string>("slice_topic", "/voxel_slice");
        mapping_cloud_topic_ = declare_parameter<std::string>(
            "mapping_cloud_topic", "/mapping_scan");
        use_latest_tf_ = declare_parameter<bool>("use_latest_tf", false);
        world_width_ = declare_parameter<double>("world_width", 20.5);
        world_height_ = declare_parameter<double>("world_height", 17.0);
        origin_x_ = declare_parameter<double>("origin_x", -6.5);
        origin_y_ = declare_parameter<double>("origin_y", -3.0);
        min_range_ = declare_parameter<double>("min_range", 0.0);
        max_range_ = declare_parameter<double>("max_range", 30.0);
        min_projection_z_ = declare_parameter<double>("min_projection_z", 0.32);
        max_projection_z_ = declare_parameter<double>("max_projection_z", 2.0);
        point_stride_ = declare_parameter<int>("point_stride", 4);

        if (resolution <= 0.0 || world_width_ <= 0.0 || world_height_ <= 0.0 ||
            min_range_ < 0.0 || max_range_ <= min_range_ || point_stride_ <= 0 ||
            max_projection_z_ <= min_projection_z_)
        {
            throw std::runtime_error(
                "resolution, map dimensions, ranges, projection heights, and point_stride are invalid");
        }

        octree_ = std::make_unique<octomap::OcTree>(resolution);
        slice_width_ = static_cast<int>(std::ceil(world_width_ / resolution));
        slice_height_ = static_cast<int>(std::ceil(world_height_ / resolution));
        projected_map_.assign(static_cast<std::size_t>(slice_width_ * slice_height_), -1);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, sensor_qos,
            std::bind(&PointcloudOctomapNode::cloudCallback, this, std::placeholders::_1));

        octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", 1);
        slice_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(slice_topic_, 1);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mapping_pose", 1);
        mapping_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            mapping_cloud_topic_, 1);

        RCLCPP_INFO(
            get_logger(), "Mapping PointCloud2 %s into %s; publishing %s and /octomap_binary",
            cloud_topic_.c_str(), map_frame_.c_str(), slice_topic_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring PointCloud2 without a frame_id");
            return;
        }
        if (!hasFloat32Xyz(*msg)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring PointCloud2 on %s: expected FLOAT32 x, y, and z fields",
                cloud_topic_.c_str());
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
                "Waiting for TF %s -> %s at the cloud timestamp: %s",
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
                get_logger(), *get_clock(), 2000, "Ignoring cloud with an invalid TF rotation");
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
        std::vector<Eigen::Vector3d> transformed_points;
        std::vector<Eigen::Vector2d> projected_endpoints_xy;
        const std::size_t point_count =
            static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        obstacle_points.reserve(point_count / static_cast<std::size_t>(point_stride_) + 1U);
        transformed_points.reserve(point_count / static_cast<std::size_t>(point_stride_) + 1U);

        sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*msg, "z");
        for (std::size_t index = 0; index < point_count;
            ++index, ++x_iterator, ++y_iterator, ++z_iterator)
        {
            if (index % static_cast<std::size_t>(point_stride_) != 0U) {
                continue;
            }


            const Eigen::Vector3d point_sensor{*x_iterator, *y_iterator, *z_iterator};
            const double range = point_sensor.norm();
            if (!point_sensor.allFinite() || range < min_range_ || range > max_range_) {
                continue;
            }

            const Eigen::Vector3d point_enu = sensor_origin_enu + rotation * point_sensor;
            obstacle_points.push_back(octomap::point3d(
                static_cast<float>(point_enu.x()),
                static_cast<float>(point_enu.y()),
                static_cast<float>(point_enu.z())));
            transformed_points.push_back(point_enu);

            // The planner's 2D slice ignores floor and ceiling observations,
            // while the full 3D OctoMap retains them.
            if (point_enu.z() >= min_projection_z_ && point_enu.z() <= max_projection_z_) {
                projected_endpoints_xy.emplace_back(point_enu.x(), point_enu.y());
            }
        }

        if (obstacle_points.size() == 0U) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No finite in-range points received on %s", cloud_topic_.c_str());
            return;
        }

        // insertPointCloud clears the rays from the sensor origin, then marks
        // each finite endpoint occupied. Missing / NaN returns are skipped and
        // therefore remain unknown instead of becoming artificial obstacles.

        octree_->insertPointCloud(obstacle_points, sensor_origin, -1.0, true, true);
        octree_->updateInnerOccupancy();

        const Eigen::Vector2d sensor_origin_xy = sensor_origin_enu.head<2>();
        for (const Eigen::Vector2d& endpoint_xy : projected_endpoints_xy) {
            markProjectedRayFree(sensor_origin_xy, endpoint_xy);
        }
        for (const Eigen::Vector2d& endpoint_xy : projected_endpoints_xy) {
            markProjectedObstacleWithBuffer(endpoint_xy);
        }

        const rclcpp::Time stamp(msg->header.stamp);
        publishMappingPose(transform, stamp);
        publishMap(stamp);
        publishProjectedMap(stamp);
        publishTransformedCloud(transformed_points, stamp);
    }

    static bool hasFloat32Xyz(const sensor_msgs::msg::PointCloud2& cloud)
    {
        const auto is_float32_field = [&cloud](const std::string& name) {
            return std::any_of(
                cloud.fields.begin(), cloud.fields.end(), [&name](const auto& field) {
                    return field.name == name &&
                        field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1U;
                });
        };
        return is_float32_field("x") && is_float32_field("y") && is_float32_field("z");
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

    void publishTransformedCloud(
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
        mapping_cloud_publisher_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_cloud_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<octomap::OcTree> octree_;

    std::string map_frame_;
    std::string cloud_topic_;
    std::string slice_topic_;
    std::string mapping_cloud_topic_;
    bool use_latest_tf_ = false;
    double world_width_ = 0.0;
    double world_height_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double min_range_ = 0.0;
    double max_range_ = 0.0;
    double min_projection_z_ = 0.0;
    double max_projection_z_ = 0.0;
    int point_stride_ = 1;
    int slice_width_ = 0;
    int slice_height_ = 0;
    std::vector<std::int8_t> projected_map_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudOctomapNode>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bonxai_map/probabilistic_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Inserts the raw 3D LiDAR cloud into Bonxai's probabilistic sparse voxel
// grid, then publishes only occupied voxel centers in a drone-centered local
// box.  Poisson consumes that PointCloud2 directly as its occupancy map.
class PointcloudBonxaiNode : public rclcpp::Node
{
public:
    PointcloudBonxaiNode()
    : Node("pointcloud_bonxai_node")
    {
        resolution_ = declare_parameter<double>("resolution", 0.10);
        map_frame_ = declare_parameter<std::string>("map_frame", "Drone1/gps_origin");
        cloud_topic_ = declare_parameter<std::string>(
            "cloud_topic", "/Drone1/lidar/point_cloud");
        occupancy_topic_ = declare_parameter<std::string>(
            "occupancy_topic", "/Drone1/sdf_map/occupancy");
        local_box_width_ = declare_parameter<double>("local_box_width", 3.5);
        local_box_height_ = declare_parameter<double>("local_box_height", 3.5);
        use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);
        min_range_ = declare_parameter<double>("min_range", 0.28);
        max_range_ = declare_parameter<double>("max_range", 40.0);
        point_stride_ = declare_parameter<int>("point_stride", 4);
        self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);
        self_filter_min_x_ = declare_parameter<double>("self_filter_min_x", -0.35);
        self_filter_max_x_ = declare_parameter<double>("self_filter_max_x", 0.42);
        self_filter_min_y_ = declare_parameter<double>("self_filter_min_y", -0.45);
        self_filter_max_y_ = declare_parameter<double>("self_filter_max_y", 0.45);
        self_filter_min_z_ = declare_parameter<double>("self_filter_min_z", -0.18);
        self_filter_max_z_ = declare_parameter<double>("self_filter_max_z", 0.08);
        const double sensor_roll = declare_parameter<double>("sensor_roll", 0.0);
        const double sensor_pitch = declare_parameter<double>("sensor_pitch", 0.0);
        const double sensor_yaw = declare_parameter<double>("sensor_yaw", 0.0);
        sensor_to_body_rotation_ =
            Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sensor_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sensor_roll, Eigen::Vector3d::UnitX());

        if (resolution_ <= 0.0 || local_box_width_ <= 0.0 || local_box_height_ <= 0.0 ||
            min_range_ < 0.0 || max_range_ <= min_range_ || point_stride_ <= 0 ||
            self_filter_max_x_ <= self_filter_min_x_ ||
            self_filter_max_y_ <= self_filter_min_y_ ||
            self_filter_max_z_ <= self_filter_min_z_)
        {
            throw std::runtime_error("Bonxai mapping parameters are invalid");
        }

        bonxai_map_ = std::make_unique<Bonxai::ProbabilisticMap>(resolution_);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
            std::bind(&PointcloudBonxaiNode::cloudCallback, this, std::placeholders::_1));
        occupancy_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(occupancy_topic_, 1);

        RCLCPP_INFO(
            get_logger(), "Mapping %s into Bonxai; publishing a %.2f x %.2f m local map on %s",
            cloud_topic_.c_str(), local_box_width_, local_box_height_, occupancy_topic_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id.empty() || !hasFloat32Xyz(*msg)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring PointCloud2 on %s: expected frame_id and FLOAT32 x/y/z fields",
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
                "Waiting for TF %s -> %s: %s",
                map_frame_.c_str(), msg->header.frame_id.c_str(), error.what());
            return;
        }

        Eigen::Quaterniond orientation{
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z};
        if (orientation.norm() < 1e-6) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring cloud with invalid TF rotation");
            return;
        }
        orientation.normalize();
        const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
        const Eigen::Vector3d sensor_origin{
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z};

        std::vector<Eigen::Vector3d> obstacle_points;
        const std::size_t point_count =
            static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        obstacle_points.reserve(point_count / static_cast<std::size_t>(point_stride_) + 1U);
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
            if (!point_sensor.allFinite() || range < min_range_ || range > max_range_ ||
                isInsideSelfFilter(sensor_to_body_rotation_ * point_sensor))
            {
                continue;
            }
            obstacle_points.push_back(sensor_origin + rotation * point_sensor);
        }

        if (obstacle_points.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "No finite in-range points received on %s",
                cloud_topic_.c_str());
            return;
        }

        bonxai_map_->insertPointCloud(obstacle_points, sensor_origin, max_range_);
        publishLocalOccupiedCloud(sensor_origin, rclcpp::Time(msg->header.stamp));
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

    bool isInsideSelfFilter(const Eigen::Vector3d& point_body) const
    {
        return self_filter_enabled_ &&
            point_body.x() >= self_filter_min_x_ && point_body.x() <= self_filter_max_x_ &&
            point_body.y() >= self_filter_min_y_ && point_body.y() <= self_filter_max_y_ &&
            point_body.z() >= self_filter_min_z_ && point_body.z() <= self_filter_max_z_;
    }

    void publishLocalOccupiedCloud(const Eigen::Vector3d& drone_center, const rclcpp::Time& stamp)
    {
        std::vector<Eigen::Vector3d> occupied_voxels;
        bonxai_map_->getOccupiedVoxels(occupied_voxels);

        const double half_width = 0.5 * local_box_width_;
        const double half_height = 0.5 * local_box_height_;
        const double voxel_center_offset = 0.5 * resolution_;
        std::vector<Eigen::Vector3d> local_occupied_voxels;
        local_occupied_voxels.reserve(occupied_voxels.size());
        for (const Eigen::Vector3d& voxel_origin : occupied_voxels) {
            const Eigen::Vector3d voxel_center =
                voxel_origin + Eigen::Vector3d::Constant(voxel_center_offset);
            if (std::abs(voxel_center.x() - drone_center.x()) <= half_width &&
                std::abs(voxel_center.y() - drone_center.y()) <= half_height)
            {
                local_occupied_voxels.push_back(voxel_center);
            }
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = map_frame_;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(local_occupied_voxels.size());
        sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
        for (const Eigen::Vector3d& point : local_occupied_voxels) {
            *x_iterator = static_cast<float>(point.x());
            *y_iterator = static_cast<float>(point.y());
            *z_iterator = static_cast<float>(point.z());
            ++x_iterator;
            ++y_iterator;
            ++z_iterator;
        }
        occupancy_publisher_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<Bonxai::ProbabilisticMap> bonxai_map_;

    double resolution_ = 0.0;
    std::string map_frame_;
    std::string cloud_topic_;
    std::string occupancy_topic_;
    double local_box_width_ = 0.0;
    double local_box_height_ = 0.0;
    bool use_latest_tf_ = true;
    double min_range_ = 0.0;
    double max_range_ = 0.0;
    int point_stride_ = 1;
    bool self_filter_enabled_ = true;
    double self_filter_min_x_ = 0.0;
    double self_filter_max_x_ = 0.0;
    double self_filter_min_y_ = 0.0;
    double self_filter_max_y_ = 0.0;
    double self_filter_min_z_ = 0.0;
    double self_filter_max_z_ = 0.0;
    Eigen::Quaterniond sensor_to_body_rotation_ = Eigen::Quaterniond::Identity();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudBonxaiNode>());
    rclcpp::shutdown();
    return 0;
}

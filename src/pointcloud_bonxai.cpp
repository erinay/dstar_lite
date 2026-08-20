#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bonxai_map/probabilistic_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Inserts the raw 3D LiDAR cloud into Bonxai's probabilistic sparse voxel
// grid. Each scan updates a moving local observation window, while Bonxai
// retains those observations in its global map. The complete accumulated map
// is projected into a 2D OccupancyGrid for D* Lite.
class PointcloudBonxaiNode : public rclcpp::Node
{
public:
    PointcloudBonxaiNode()
    : Node("pointcloud_bonxai_node")
    {
        resolution_ = declare_parameter<double>("resolution", 0.10);
        map_frame_ = declare_parameter<std::string>("map_frame", "odom");
        cloud_topic_ = declare_parameter<std::string>(
            "cloud_topic", "/Drone1/lidar/point_cloud");
        occupancy_topic_ = declare_parameter<std::string>(
            "occupancy_topic", "/Drone1/sdf_map/occupancy");
        slice_topic_ = declare_parameter<std::string>("slice_topic", "/voxel_slice");
        mapping_scan_topic_ = declare_parameter<std::string>("mapping_scan_topic", "/mapping_scan");
        observation_width_ = declare_parameter<double>("observation_width", 5.0);
        observation_height_ = declare_parameter<double>("observation_height", 5.0);
        world_width_ = declare_parameter<double>("world_width", 20.5);
        world_height_ = declare_parameter<double>("world_height", 17.0);
        origin_x_ = declare_parameter<double>("origin_x", -6.5);
        origin_y_ = declare_parameter<double>("origin_y", -3.0);
        min_projection_z_ = declare_parameter<double>("min_projection_z", 0.10);
        max_projection_z_ = declare_parameter<double>("max_projection_z", 2.0);
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

        if (resolution_ <= 0.0 || observation_width_ <= 0.0 || observation_height_ <= 0.0 ||
            world_width_ <= 0.0 || world_height_ <= 0.0 ||
            max_projection_z_ <= min_projection_z_ ||
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
        slice_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(slice_topic_, 1);
        mapping_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(mapping_scan_topic_, 1);

        RCLCPP_INFO(
            get_logger(),
            "Mapping %s into Bonxai with a %.2f x %.2f m observation window; "
            "publishing the accumulated map on %s and %s",
            cloud_topic_.c_str(), observation_width_, observation_height_, occupancy_topic_.c_str(),
            slice_topic_.c_str());
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

        const std::size_t point_count =
            static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        std::size_t observation_count = 0U;
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
            const Eigen::Vector3d point_map = sensor_origin + rotation * point_sensor;
            const Eigen::Vector3d ray = point_map - sensor_origin;
            const double half_width = 0.5 * observation_width_;
            const double half_height = 0.5 * observation_height_;
            double clipped_fraction = 1.0;
            if (std::abs(ray.x()) > half_width) {
                clipped_fraction = std::min(clipped_fraction, half_width / std::abs(ray.x()));
            }
            if (std::abs(ray.y()) > half_height) {
                clipped_fraction = std::min(clipped_fraction, half_height / std::abs(ray.y()));
            }

            if (clipped_fraction < 1.0) {
                // A return outside the observation square establishes free
                // space only up to the square boundary; it is not an obstacle.
                bonxai_map_->addMissPoint(sensor_origin + clipped_fraction * ray);
            } else {
                bonxai_map_->addHitPoint(point_map);
            }
            ++observation_count;
        }

        if (observation_count == 0U) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "No finite in-range points received on %s",
                cloud_topic_.c_str());
            return;
        }

        bonxai_map_->updateFreeCells(sensor_origin);
        publishBonxaiOutputs(rclcpp::Time(msg->header.stamp));
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

    bool projectVoxelToSlice(
        const Eigen::Vector3d& voxel_origin, nav_msgs::msg::OccupancyGrid& slice,
        const std::int8_t occupancy) const
    {
        const Eigen::Vector3d voxel_center =
            voxel_origin + Eigen::Vector3d::Constant(0.5 * resolution_);
        if (voxel_center.z() < min_projection_z_ || voxel_center.z() > max_projection_z_)
        {
            return false;
        }

        const int cell_x = static_cast<int>(std::floor(
            (voxel_center.x() - origin_x_) / resolution_));
        const int cell_y = static_cast<int>(std::floor(
            (voxel_center.y() - origin_y_) / resolution_));
        if (cell_x < 0 || cell_y < 0 ||
            cell_x >= static_cast<int>(slice.info.width) ||
            cell_y >= static_cast<int>(slice.info.height))
        {
            return false;
        }

        const std::size_t index = static_cast<std::size_t>(cell_x) +
            static_cast<std::size_t>(cell_y) * slice.info.width;
        // Occupied wins when the 3D projection contains both a free ray and a
        // solid voxel in the same 2D cell.
        if (occupancy == 100 || slice.data[index] != 100) {
            slice.data[index] = occupancy;
        }
        return true;
    }

    void publishPointCloud(
        const std::vector<Eigen::Vector3d>& points, const rclcpp::Time& stamp,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
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
        publisher->publish(cloud);
    }

    void publishBonxaiOutputs(const rclcpp::Time& stamp)
    {
        std::vector<Eigen::Vector3d> occupied_voxels;
        std::vector<Bonxai::CoordT> free_voxel_coords;
        bonxai_map_->getOccupiedVoxels(occupied_voxels);
        bonxai_map_->getFreeVoxels(free_voxel_coords);

        std::vector<Eigen::Vector3d> occupied_voxel_centers;
        occupied_voxel_centers.reserve(occupied_voxels.size());
        for (const Eigen::Vector3d& voxel_origin : occupied_voxels) {
            const Eigen::Vector3d voxel_center =
                voxel_origin + Eigen::Vector3d::Constant(0.5 * resolution_);
            occupied_voxel_centers.push_back(voxel_center);
        }
        publishPointCloud(occupied_voxel_centers, stamp, occupancy_publisher_);
        publishPointCloud(occupied_voxel_centers, stamp, mapping_scan_publisher_);

        nav_msgs::msg::OccupancyGrid slice;
        slice.header.stamp = stamp;
        slice.header.frame_id = map_frame_;
        slice.info.map_load_time = stamp;
        slice.info.resolution = static_cast<float>(resolution_);
        slice.info.width = static_cast<std::uint32_t>(std::ceil(world_width_ / resolution_));
        slice.info.height = static_cast<std::uint32_t>(std::ceil(world_height_ / resolution_));
        slice.info.origin.position.x = origin_x_;
        slice.info.origin.position.y = origin_y_;
        slice.info.origin.orientation.w = 1.0;
        slice.data.assign(
            static_cast<std::size_t>(slice.info.width) * slice.info.height, static_cast<std::int8_t>(-1));

        for (const Bonxai::CoordT& voxel_coord : free_voxel_coords) {
            const Eigen::Vector3d voxel_origin{
                static_cast<double>(voxel_coord.x) * resolution_,
                static_cast<double>(voxel_coord.y) * resolution_,
                static_cast<double>(voxel_coord.z) * resolution_};
            projectVoxelToSlice(voxel_origin, slice, 0);
        }
        for (const Eigen::Vector3d& voxel_origin : occupied_voxels) {
            projectVoxelToSlice(voxel_origin, slice, 100);
        }
        slice_publisher_->publish(slice);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_scan_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<Bonxai::ProbabilisticMap> bonxai_map_;

    double resolution_ = 0.0;
    std::string map_frame_;
    std::string cloud_topic_;
    std::string occupancy_topic_;
    std::string slice_topic_;
    std::string mapping_scan_topic_;
    double observation_width_ = 0.0;
    double observation_height_ = 0.0;
    double world_width_ = 0.0;
    double world_height_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double min_projection_z_ = 0.0;
    double max_projection_z_ = 0.0;
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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>

// Publishes the pose of the Gazebo lidar in standard ROS ENU / FLU frames.
// It is deliberately separate from the mapper: the raw Gazebo LaserScan is
// left untouched, and the mapping path uses this TF just like real hardware.
class Px4LidarTfBroadcaster : public rclcpp::Node
{
public:
    Px4LidarTfBroadcaster()
    : Node("px4_lidar_tf_broadcaster_node")
    {
        map_frame_ = declare_parameter<std::string>("map_frame", "odom");
        sensor_frame_ = declare_parameter<std::string>("sensor_frame", "mock_lidar_link");
        odometry_topic_ = declare_parameter<std::string>(
            "odometry_topic", "/fmu/out/vehicle_odometry");
        sensor_offset_flu_.x() = declare_parameter<double>("sensor_offset_x", 0.0);
        sensor_offset_flu_.y() = declare_parameter<double>("sensor_offset_y", 0.0);
        // The centred lidar's actual ray origin is 0.315 m above base_link:
        // 0.260 m mount height plus the component's 0.055 m sensor pose.
        sensor_offset_flu_.z() = declare_parameter<double>("sensor_offset_z", 0.315);

        R_ned_to_enu_ << 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0;
        R_flu_to_frd_ << 1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim_lidar/pose", 10);
        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        odometry_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            odometry_topic_, sensor_qos,
            std::bind(&Px4LidarTfBroadcaster::odometryCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(), "Publishing %s -> %s from %s",
            map_frame_.c_str(), sensor_frame_.c_str(), odometry_topic_.c_str());
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

        if (!position_ned.allFinite() || q_ned_frd.norm() < 1e-6) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Waiting for finite PX4 vehicle odometry");
            return;
        }

        q_ned_frd.normalize();
        const Eigen::Vector3d vehicle_position_enu = R_ned_to_enu_ * position_ned;
        const Eigen::Matrix3d R_enu_flu =
            R_ned_to_enu_ * q_ned_frd.toRotationMatrix() * R_flu_to_frd_;
        const Eigen::Vector3d sensor_position_enu =
            vehicle_position_enu + R_enu_flu * sensor_offset_flu_;
        const Eigen::Quaterniond sensor_orientation(R_enu_flu);
        const rclcpp::Time stamp = now();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = sensor_frame_;
        transform.transform.translation.x = sensor_position_enu.x();
        transform.transform.translation.y = sensor_position_enu.y();
        transform.transform.translation.z = sensor_position_enu.z();
        transform.transform.rotation.w = sensor_orientation.w();
        transform.transform.rotation.x = sensor_orientation.x();
        transform.transform.rotation.y = sensor_orientation.y();
        transform.transform.rotation.z = sensor_orientation.z();
        tf_broadcaster_->sendTransform(transform);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = transform.header;
        pose.pose.position.x = sensor_position_enu.x();
        pose.pose.position.y = sensor_position_enu.y();
        pose.pose.position.z = sensor_position_enu.z();
        pose.pose.orientation = transform.transform.rotation;
        pose_publisher_->publish(pose);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Matrix3d R_ned_to_enu_;
    Eigen::Matrix3d R_flu_to_frd_;
    Eigen::Vector3d sensor_offset_flu_ = Eigen::Vector3d::Zero();
    std::string map_frame_;
    std::string sensor_frame_;
    std::string odometry_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4LidarTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

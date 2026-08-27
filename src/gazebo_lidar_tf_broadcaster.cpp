#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <Eigen/Geometry>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Registers the raw Gazebo lidar using Gazebo's exact ENU / FLU model pose.
// This is for simulator mapping validation; PX4 estimator resets cannot shift
// or rotate this transform because PX4 is not in this data path.
class GazeboLidarTfBroadcaster : public rclcpp::Node
{
public:
    GazeboLidarTfBroadcaster()
    : Node("gazebo_lidar_tf_broadcaster_node")
    {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        body_frame_ = declare_parameter<std::string>("body_frame", "gazebo_body");
        sensor_frame_ = declare_parameter<std::string>("sensor_frame", "mock_lidar_link");
        model_name_ = declare_parameter<std::string>("model_name", "x500_mock_lidar_2d_0");
        pose_topic_ = declare_parameter<std::string>(
            "pose_topic", "/world/maze_2d/dynamic_pose/info");
        sensor_offset_flu_.x() = declare_parameter<double>("sensor_offset_x", 0.0);
        sensor_offset_flu_.y() = declare_parameter<double>("sensor_offset_y", 0.0);
        sensor_offset_flu_.z() = declare_parameter<double>("sensor_offset_z", 0.315);
        const double sensor_roll = declare_parameter<double>("sensor_roll", 0.0);
        const double sensor_pitch = declare_parameter<double>("sensor_pitch", 0.0);
        const double sensor_yaw = declare_parameter<double>("sensor_yaw", 0.0);
        sensor_mount_orientation_ =
            Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sensor_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sensor_roll, Eigen::Vector3d::UnitX());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim_lidar/pose", 10);
        if (!gz_node_.Subscribe(pose_topic_, &GazeboLidarTfBroadcaster::poseCallback, this)) {
            throw std::runtime_error("Failed to subscribe to Gazebo pose topic: " + pose_topic_);
        }

        RCLCPP_INFO(
            get_logger(), "Publishing %s -> %s -> %s from %s for %s (sensor RPY %.3f %.3f %.3f rad)",
            map_frame_.c_str(), body_frame_.c_str(), sensor_frame_.c_str(), pose_topic_.c_str(), model_name_.c_str(),
            sensor_roll, sensor_pitch, sensor_yaw);
    }

private:
    void poseCallback(const gz::msgs::Pose_V& msg)
    {
        for (int index = 0; index < msg.pose_size(); ++index) {
            const gz::msgs::Pose& model_pose = msg.pose(index);
            if (model_pose.name() != model_name_) {
                continue;
            }

            Eigen::Quaterniond orientation{
                model_pose.orientation().w(), model_pose.orientation().x(),
                model_pose.orientation().y(), model_pose.orientation().z()};
            if (orientation.norm() < 1e-6) {
                return;
            }
            orientation.normalize();
            const Eigen::Vector3d model_position{
                model_pose.position().x(), model_pose.position().y(), model_pose.position().z()};
            const Eigen::Vector3d sensor_position =
                model_position + orientation.toRotationMatrix() * sensor_offset_flu_;
            const Eigen::Quaterniond sensor_orientation =
                orientation * sensor_mount_orientation_;
            geometry_msgs::msg::TransformStamped body_transform;
            // The raw LaserScan is stamped by Gazebo simulation time.  TF
            // must use this same timestamp; stamping it with ROS wall time
            // forces the mapper to use a pose from a different instant.
            if (msg.header().has_stamp()) {
                body_transform.header.stamp.sec = msg.header().stamp().sec();
                body_transform.header.stamp.nanosec = msg.header().stamp().nsec();
            } else {
                body_transform.header.stamp = now();
            }
            body_transform.header.frame_id = map_frame_;
            body_transform.child_frame_id = body_frame_;
            body_transform.transform.translation.x = model_position.x();
            body_transform.transform.translation.y = model_position.y();
            body_transform.transform.translation.z = model_position.z();
            body_transform.transform.rotation.w = orientation.w();
            body_transform.transform.rotation.x = orientation.x();
            body_transform.transform.rotation.y = orientation.y();
            body_transform.transform.rotation.z = orientation.z();

            geometry_msgs::msg::TransformStamped sensor_transform;
            sensor_transform.header.stamp = body_transform.header.stamp;
            sensor_transform.header.frame_id = body_frame_;
            sensor_transform.child_frame_id = sensor_frame_;
            sensor_transform.transform.translation.x = sensor_offset_flu_.x();
            sensor_transform.transform.translation.y = sensor_offset_flu_.y();
            sensor_transform.transform.translation.z = sensor_offset_flu_.z();
            sensor_transform.transform.rotation.w = sensor_mount_orientation_.w();
            sensor_transform.transform.rotation.x = sensor_mount_orientation_.x();
            sensor_transform.transform.rotation.y = sensor_mount_orientation_.y();
            sensor_transform.transform.rotation.z = sensor_mount_orientation_.z();
            tf_broadcaster_->sendTransform(
                std::vector<geometry_msgs::msg::TransformStamped>{body_transform, sensor_transform});

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = body_transform.header.stamp;
            pose.header.frame_id = map_frame_;
            pose.pose.position.x = sensor_position.x();
            pose.pose.position.y = sensor_position.y();
            pose.pose.position.z = sensor_position.z();
            pose.pose.orientation.w = sensor_orientation.w();
            pose.pose.orientation.x = sensor_orientation.x();
            pose.pose.orientation.y = sensor_orientation.y();
            pose.pose.orientation.z = sensor_orientation.z();
            pose_publisher_->publish(pose);
            return;
        }
    }

    gz::transport::Node gz_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Vector3d sensor_offset_flu_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond sensor_mount_orientation_ = Eigen::Quaterniond::Identity();
    std::string map_frame_;
    std::string body_frame_;
    std::string sensor_frame_;
    std::string model_name_;
    std::string pose_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboLidarTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

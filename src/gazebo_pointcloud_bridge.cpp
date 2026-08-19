#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

// A small one-way bridge for the exact Gazebo Harmonic libraries installed by
// PX4.  It preserves the packed point data and field layout, changing only
// the ROS topic/frame names required by the Poisson safety stack.
class GazeboPointcloudBridge : public rclcpp::Node
{
public:
    GazeboPointcloudBridge()
    : Node("gazebo_pointcloud_bridge_node")
    {
        gz_topic_ = declare_parameter<std::string>(
            "gz_topic",
            "/world/maze_2d/model/x500_lidar_3d_0/link/link/"
            "sensor/lidar_3d/scan/points");
        ros_topic_ = declare_parameter<std::string>(
            "ros_topic", "/Drone1/lidar/point_cloud");
        frame_id_ = declare_parameter<std::string>(
            "frame_id", "Drone1/livox_frame/lidar");

        if (gz_topic_.empty() || ros_topic_.empty() || frame_id_.empty()) {
            throw std::runtime_error("gz_topic, ros_topic, and frame_id must not be empty");
        }

        // Spark Fast-LIO requests reliable lidar data.  A reliable publisher
        // is also compatible with the existing best-effort map consumers.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2));
        qos.reliable().durability_volatile();
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, qos);
        if (!gz_node_.Subscribe(gz_topic_, &GazeboPointcloudBridge::pointcloudCallback, this)) {
            throw std::runtime_error("Failed to subscribe to Gazebo point cloud: " + gz_topic_);
        }

        RCLCPP_INFO(
            get_logger(), "Bridging Gazebo point cloud %s to %s (frame %s)",
            gz_topic_.c_str(), ros_topic_.c_str(), frame_id_.c_str());
    }

private:
    static std::uint8_t rosDatatype(const gz::msgs::PointCloudPacked::Field & field)
    {
        using GzField = gz::msgs::PointCloudPacked::Field;
        switch (field.datatype()) {
        case GzField::INT8:
            return sensor_msgs::msg::PointField::INT8;
        case GzField::UINT8:
            return sensor_msgs::msg::PointField::UINT8;
        case GzField::INT16:
            return sensor_msgs::msg::PointField::INT16;
        case GzField::UINT16:
            return sensor_msgs::msg::PointField::UINT16;
        case GzField::INT32:
            return sensor_msgs::msg::PointField::INT32;
        case GzField::UINT32:
            return sensor_msgs::msg::PointField::UINT32;
        case GzField::FLOAT32:
            return sensor_msgs::msg::PointField::FLOAT32;
        case GzField::FLOAT64:
            return sensor_msgs::msg::PointField::FLOAT64;
        default:
            throw std::runtime_error("Unsupported Gazebo PointCloudPacked field datatype");
        }
    }

    void pointcloudCallback(const gz::msgs::PointCloudPacked & msg)
    {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        if (msg.has_header() && msg.header().has_stamp()) {
            ros_cloud.header.stamp.sec = msg.header().stamp().sec();
            ros_cloud.header.stamp.nanosec = msg.header().stamp().nsec();
        } else {
            ros_cloud.header.stamp = now();
        }
        ros_cloud.header.frame_id = frame_id_;
        ros_cloud.height = msg.height();
        ros_cloud.width = msg.width();
        ros_cloud.is_bigendian = msg.is_bigendian();
        ros_cloud.point_step = msg.point_step();
        ros_cloud.row_step = msg.row_step();
        ros_cloud.is_dense = msg.is_dense();
        ros_cloud.fields.reserve(static_cast<std::size_t>(msg.field_size()));
        for (int index = 0; index < msg.field_size(); ++index) {
            const auto & gz_field = msg.field(index);
            sensor_msgs::msg::PointField ros_field;
            ros_field.name = gz_field.name();
            ros_field.offset = gz_field.offset();
            ros_field.datatype = rosDatatype(gz_field);
            ros_field.count = gz_field.count();
            ros_cloud.fields.push_back(std::move(ros_field));
        }
        ros_cloud.data.assign(msg.data().begin(), msg.data().end());
        publisher_->publish(std::move(ros_cloud));
    }

    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string gz_topic_;
    std::string ros_topic_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboPointcloudBridge>());
    rclcpp::shutdown();
    return 0;
}

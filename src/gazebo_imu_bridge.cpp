#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>

#include <memory>
#include <stdexcept>
#include <string>

// One-way Gazebo Harmonic IMU -> ROS sensor_msgs/Imu bridge.  Reading the
// Gazebo sensor directly keeps the IMU and the simulated point cloud in the
// same simulation clock and FLU body-frame convention.
class GazeboImuBridge : public rclcpp::Node
{
public:
    GazeboImuBridge()
    : Node("gazebo_imu_bridge_node")
    {
        gz_topic_ = declare_parameter<std::string>(
            "gz_topic",
            "/world/maze_2d/model/x500_lidar_3d_0/link/base_link/"
            "sensor/imu_sensor/imu");
        ros_topic_ = declare_parameter<std::string>("ros_topic", "/Drone1/imu");
        frame_id_ = declare_parameter<std::string>("frame_id", "Drone1/base_link");

        if (gz_topic_.empty() || ros_topic_.empty() || frame_id_.empty()) {
            throw std::runtime_error("gz_topic, ros_topic, and frame_id must not be empty");
        }

        publisher_ = create_publisher<sensor_msgs::msg::Imu>(
            ros_topic_, rclcpp::SensorDataQoS().keep_last(50));
        if (!gz_node_.Subscribe(gz_topic_, &GazeboImuBridge::imuCallback, this)) {
            throw std::runtime_error("Failed to subscribe to Gazebo IMU: " + gz_topic_);
        }

        RCLCPP_INFO(
            get_logger(), "Bridging Gazebo IMU %s to %s (frame %s)",
            gz_topic_.c_str(), ros_topic_.c_str(), frame_id_.c_str());
    }

private:
    void imuCallback(const gz::msgs::IMU & msg)
    {
        sensor_msgs::msg::Imu ros_imu;
        if (msg.has_header() && msg.header().has_stamp()) {
            ros_imu.header.stamp.sec = msg.header().stamp().sec();
            ros_imu.header.stamp.nanosec = msg.header().stamp().nsec();
        } else {
            ros_imu.header.stamp = now();
        }
        ros_imu.header.frame_id = frame_id_;

        // Gazebo's IMU reports vectors in the Gazebo FLU base_link frame.
        ros_imu.angular_velocity.x = msg.angular_velocity().x();
        ros_imu.angular_velocity.y = msg.angular_velocity().y();
        ros_imu.angular_velocity.z = msg.angular_velocity().z();
        ros_imu.linear_acceleration.x = msg.linear_acceleration().x();
        ros_imu.linear_acceleration.y = msg.linear_acceleration().y();
        ros_imu.linear_acceleration.z = msg.linear_acceleration().z();

        // This is a rate/acceleration sensor.  Do not imply that it provides
        // a fused absolute orientation estimate.
        ros_imu.orientation_covariance[0] = -1.0;
        publisher_->publish(std::move(ros_imu));
    }

    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::string gz_topic_;
    std::string ros_topic_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboImuBridge>());
    rclcpp::shutdown();
    return 0;
}

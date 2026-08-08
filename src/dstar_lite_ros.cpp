#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dstar_lite.hpp"
#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class DStarLiteNode: public rclcpp::Node{
    public: DStarLiteNode(): Node("dsar_lite_node"){
        rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
			sensor_qos.best_effort();
			sensor_qos.durability_volatile();

        declare_parameters();
        have_odom_=false;

        R_NED2ENU << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;

        R_FLU2FRD << 1, 0, 0,
                        0, -1, 0,
                        0, 0, -1;

        // Subscribers
        odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", sensor_qos, std::bind(&DStarLiteNode::odom_callback, this, std::placeholders::_1));
        obstacle_subscriber_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
            "/fmu/out/obstacle_distance", sensor_qos, std::bind(&DStarLiteNode::obstacle_callback, this, std::placeholders::_1));

        // Publishers
        belief_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/belief_map", 1);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 1);
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/waypoint", 1);

        initialize();
    }

    private:
    
    // Publishers & Subscripers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr belief_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;

    // States
    double world_width_{0.0};
    double world_height_{0.0};
    double map_resolution_{0.1};
    double origin_x_{0.0};
    double origin_y_{0.0};
    double initial_start_x_{0.0};
    double initial_start_y_{0.0};
    double goal_x_{0.0};
    double goal_y_{0.0};
    int map_width_{0};
    int map_height_{0};
    double inflation_radius_{0.35};
    bool have_odom_;
    std::string planning_frame_{"odom"};

    Coord start_cell_;
    Coord goal_cell_;

    std::unique_ptr<Grid> belief_grid_;
    std::unique_ptr<DStarLite> planner_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    Eigen::Vector3d position_enu_{0.0, 0.0, 0.0};
    Eigen::Quaterniond q_enu_flu_{1.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d R_NED2ENU, R_FLU2FRD;

    int occupied_threshold = 60;

    // Functions
    void declare_parameters(){
        world_width_ =declare_parameter<double>("world_width",20.0);
        world_height_ = declare_parameter<double>("world_height",15.0);
        map_resolution_ = declare_parameter<double>("resolution",0.10);
        origin_x_ =declare_parameter<double>("origin_x",0.0);
        origin_y_ =declare_parameter<double>("origin_y",0.0);
        initial_start_x_ =declare_parameter<double>("start_x",1.25);
        initial_start_y_ =declare_parameter<double>("start_y",1.25);
        goal_x_ =declare_parameter<double>("goal_x",18.75);
        goal_y_ =declare_parameter<double>("goal_y",13.75);
    }

    void initialize(){
        map_width_ = (int) (std::ceil(world_width_/map_resolution_));
        map_height_ = (int) (std::ceil(world_height_/map_resolution_));
        belief_grid_ = std::make_unique<Grid>(map_width_, map_height_, map_resolution_);
        if (!world2grid(initial_start_x_, initial_start_y_, start_cell_)){
            throw std::runtime_error("Initial start is outside belief grid");
        }

        if (!world2grid(goal_x_, goal_y_, goal_cell_)){
            throw std::runtime_error("Goal is outside belief grid");
        }

        planner_ = std::make_unique<DStarLite>(*belief_grid_, start_cell_, goal_cell_);
        planner_->computeShortestPath();
    }

    bool world2grid(double world_x, double world_y, Coord& cell) const
    {
        const int grid_x = static_cast<int>(std::floor((world_x - origin_x_) /map_resolution_));
        const int grid_y = static_cast<int>(std::floor((world_y - origin_y_) /map_resolution_));

        if (grid_x < 0 ||grid_x >= map_width_ ||grid_y < 0 ||grid_y >= map_height_){
            return false;
        }

        cell.x = grid_x;
        cell.y = grid_y;

        return true;
    }

    geometry_msgs::msg::Point grid2world(const Coord& cell) const {
        geometry_msgs::msg::Point point;
        point.x = origin_x_+((double)(cell.x) + 0.5)*map_resolution_;
        point.y = origin_y_+((double)(cell.y) + 0.5)*map_resolution_;
        point.z = 0.0;
        return point;
    }

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
        Eigen::Vector3d pos_enu{
            (double) msg->position[0],
            (double) msg->position[1],
            (double) msg->position[2]
        };
        Eigen::Quaterniond q_measured{
            (double) msg->q[0],
            (double) msg->q[1],
            (double) msg->q[2],
            (double) msg->q[3]
        };

        position_enu_ = R_NED2ENU * pos_enu;
        q_measured.normalize();

        const Eigen::Matrix3d R_ENU_FLU = R_NED2ENU * q_measured.toRotationMatrix() * R_FLU2FRD;
        q_enu_flu_ = Eigen::Quaterniond(R_ENU_FLU);

        q_enu_flu_.normalize();

        robot_pose_.position.x = pos_enu.x();
        robot_pose_.position.y = pos_enu.y();
        robot_pose_.position.z = pos_enu.z();

        robot_pose_.orientation.w = q_enu_flu_.w();
        robot_pose_.orientation.x = q_enu_flu_.x();
        robot_pose_.orientation.y = q_enu_flu_.y();
        robot_pose_.orientation.z = q_enu_flu_.z();

        have_odom_=true;
        
        Coord new_start;

        if (!world2grid(position_enu_.x(), position_enu_.y(), new_start))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Drone outside belief grid: ENU=(%.2f, %.2f)", position_enu_.x(), position_enu_.y());
            return;
        }
        world2grid(position_enu_.x(), position_enu_.y(), new_start);
        const bool changed_cell = new_start.x != start_cell_.x || new_start.y != start_cell_.y;
        
        if(!changed_cell){return;}
        start_cell_=new_start;
        planner_->moveStart(start_cell_);
        planner_->computeShortestPath();

        publish_path();
    }   

    bool apply_observation(const Coord& cell, int observed_state){
        const int old_state = belief_grid_->state(cell);
        if (old_state == observed_state) {
            return false;
        }

        const double old_cost =
            belief_grid_->traversalCost(cell);
        planner_->updateCellState(cell,observed_state);
        const double new_cost = belief_grid_->traversalCost(cell);
        return !costs_equal(old_cost,new_cost);
        }

    void obstacle_callback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg){
        constexpr double pi = 3.14159265358979323846;
        constexpr double degrees_to_radians = pi / 180.0;
        constexpr std::uint16_t unknown_distance = std::numeric_limits<std::uint16_t>::max();
        const double min_range = (double)(msg->min_distance) / 100.0;
        const double max_range = (double)(msg->max_distance) / 100.0;

        //LiDAR located at drone origin
        const double sensor_x = position_enu_.x();
        const double sensor_y = position_enu_.y();
        std::vector<Coord> occupied_cells;
        const std::uint32_t clear_value = static_cast<std::uint32_t>(msg->max_distance) + 1U;
        const int inflation_extent = (int)(std::ceil(inflation_radius_ / map_resolution_));

        for (std::size_t i = 0; i <= mmsg->distances.size(); i++){
            const std::uint16_t distance_cm = msg->distances[i];

            if (distance_cm == unknown_distance) {
                continue; //unknown distance
            }
            if (static_cast<std::uint32_t>(distance_cm) == clear_value) {
                continue; //no obstacle detected
            }
            if (distance_cm < msg->min_distance || distance_cm > msg->max_distance) {
                continue; //invalid obstacle measuremtns
            }

            // First, sensor processing
            const double range = (double)((distance_cm)/100.0); //evrything in cm
            const double angle_flu = -(((double)(msg->angle_offset))+(double)(i)*(double)(msg->increment)*degrees_to_radians); //negative of neu
            const Eigen::Vector3d endpoint_flu{range*std::cos(angle_flu), range*std::sin(angle_flu),0.0};
            const Eigen::Vector3d endpoint_enu = position_enu+q_enu_flu*endpoint_flu;

            Coord obstacle_cell;
            if(!word2grid(endpoint_enu.x(), endpoint_enu.y(), obstacle_cell)){
                continue;
            }

            for (int dy=-inflation_extent; dy<=inflation_extent; dy++){
                for(int dx=-inflation_extent; dx<=inflation_extent;dx++){
                    const double distance_squared = (double) (dx*dx+dy*dx);
                    if((distance_squared)>(inflation_extent*inflation_extent)){
                        continue;
                    }
                    const Coord inflated_cell{obstacle_cell.x+dx, obstacle_cell.dy};
                    // if cells are outside of grid, or on drone, ignore
                    if (inflated_cell.x < 0 || inflated_cell.x >= map_width_ || inflated_cell.y < 0 || inflated_cell.y >= map_height_) {
                        continue;
                    } else if(inflated_cell.x == start_cell_.x && inflated_cell.y == start_cell_.y) {
                        continue;
                    }
                    //skip cells if we know is occupied
                    if(belief_grid_->state(inflated_cell)==1){
                        continue;
                    }
                    // else, update belief grid and D lite vertex information
                }
            }

        }

    }

    void publish_belief_map(){
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = now();
        map_msg.header.frame_id = planning_frame_;
        map_msg.info.map_load_time = map_msg.header.stamp;
        map_msg.info.resolution = (float)(map_resolution_);
        map_msg.info.height = static_cast<std::uint32_t>(map_height_);
        map_msg.info.width = static_cast<std::uint32_t>(map_width_);
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.x = 0.0;
        map_msg.info.origin.orientation.y = 0.0;
        map_msg.info.origin.orientation.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data.resize(static_cast<std::size_t>(map_width_) *static_cast<std::size_t>(map_height_));


        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const Coord cell{x, y};
                const std::size_t index = x+y*map_width_;
                const int state = belief_grid_->state(cell);

                if(state==-1){
                    map_msg.data[index]=-1;
                } else if (state==0){
                    map_msg.data[index] = 0;
                } else{
                    map_msg.data[index]=100;
                }
            }
        }
        belief_publisher_->publish(map_msg);
    }

    geometry_msgs::msg::Quaternion path_orientation(const Coord& from, const Coord& to) const {
        const geometry_msgs::msg::Point from_point = grid2world(from);
        const geometry_msgs::msg::Point to_point = grid2world(to);
        const double yaw = std::atan2( to_point.y - from_point.y, to_point.x - from_point.x);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0,0.0,yaw); 

        return tf2::toMsg( quaternion);
    }

    void publish_waypoint(const std::vector<Coord>& path) {
        std::size_t waypoint_index = 0;

        //If path[0] is the current start cell, command path[1].
        if (path[0].x == start_cell_.x && path[0].y == start_cell_.y) {
            if (path.size() == 1) {
                RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"Goal reached");
                return;
            }
            waypoint_index = 1;
        }

        const Coord& waypoint_cell = path[waypoint_index];
        geometry_msgs::msg::PoseStamped waypoint_msg;

        waypoint_msg.header.stamp = now();
        waypoint_msg.header.frame_id = planning_frame_;
        waypoint_msg.pose.position = grid2world(waypoint_cell);

        if (waypoint_index + 1 < path.size()) {
            waypoint_msg.pose.orientation = path_orientation(waypoint_cell, path[waypoint_index + 1]);
        } else {
            waypoint_msg.pose.orientation.w = 1.0;
        }

        waypoint_publisher_->publish(waypoint_msg);
    }
    void publish_path(){
        if (!planner_) {
            return;
        }

        const std::vector<Coord> path = planner_->extractPath();
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = planning_frame_;

        if (path.empty()) {
            path_publisher_->publish(path_msg);
            return;
        }

        path_msg.poses.reserve(path.size());
        for (std::size_t i = 0; i < path.size();i++)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = path_msg.header;
            pose_msg.pose.position = grid2world(path[i]);

            if (i + 1 < path.size()) {
                pose_msg.pose.orientation = path_orientation(path[i], path[i + 1]);
            } else {
                pose_msg.pose.orientation.w = 1.0;
            }
            path_msg.poses.push_back(pose_msg);
        }

        path_publisher_->publish(path_msg);
        publish_waypoint(path);
    }

    static bool costs_equal( double a, double b) {
        constexpr double epsilon = 1e-9;

        if (a == b) {
            return true;
        }
        if (!std::isfinite(a) || !std::isfinite(b)){
            return false;
        }
        return std::abs(a - b) <= epsilon;
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    
    try{
        rclcpp::spin(std::make_shared<DStarLiteNode>());
    }
    catch(const char* msg){
        rclcpp::shutdown();
        std::cout << msg << std::endl;
    }
}

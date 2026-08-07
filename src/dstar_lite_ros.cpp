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

        std::size_t first_used = msg->distances.size();
        std::size_t last_used = 0;

        for (std::size_t i = 0; i < msg->distances.size(); i++) {
            if (msg->distances[i] != unknown_distance) {
                first_used = std::min(first_used, i);
                last_used = std::max(last_used, i);
            }
        }

        if (first_used == msg->distances.size()) {
            return;
        }

        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = now();
        scan->header.frame_id = "base_link";
        scan->range_min = static_cast<float>(msg->min_distance) / 100.0F;
        scan->range_max = static_cast<float>(msg->max_distance) / 100.0F;
        scan->angle_increment = static_cast<float>(static_cast<double>(msg->increment) * degrees_to_radians);

        const double first_body_angle = static_cast<double>(msg->angle_offset) + static_cast<double>(last_used) * static_cast<double>(msg->increment);
        const double last_body_angle = static_cast<double>(msg->angle_offset) + static_cast<double>(first_used) * static_cast<double>(msg->increment);

        scan->angle_min = static_cast<float>(-first_body_angle * degrees_to_radians);
        scan->angle_max = static_cast<float>(-last_body_angle * degrees_to_radians);
        scan->time_increment = 0.0F;
        scan->scan_time = 0.0F;
        scan->ranges.reserve(last_used - first_used + 1);
        for (std::size_t offset = 0; offset <= last_used - first_used; ++offset)
        {
            const std::size_t px4_index = last_used - offset;
            const std::uint16_t distance_cm = msg->distances[px4_index];

            if (distance_cm == unknown_distance) {
                scan->ranges.push_back(
                    std::numeric_limits<float>::quiet_NaN());
                continue;
            }

            const std::uint32_t clear_value = static_cast<std::uint32_t>(msg->max_distance) + 1U;

            if (static_cast<std::uint32_t>(distance_cm) == clear_value) {
                scan->ranges.push_back(std::numeric_limits<float>::infinity());
                continue;
            }

            if (distance_cm < msg->min_distance || distance_cm > msg->max_distance) {
                scan->ranges.push_back(std::numeric_limits<float>::quiet_NaN());
                continue;
            }

            scan->ranges.push_back(static_cast<float>(distance_cm) / 100.0F);
        }

        scan_callback(scan);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg){
        if (!have_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for PX4 vehicle odometry");
            return;
        }

        const double sensor_x = position_enu_.x();
        const double sensor_y = position_enu_.y();

        std::vector<Coord> free_cells;
        std::vector<Coord> occupied_cells;

        const double ray_step = 0.5 * map_resolution_;

        for (std::size_t i = 0; i < msg->ranges.size(); i++) {
            const double raw_range = static_cast<double>(msg->ranges[i]);

            bool obstacle_hit = false;
            double ray_length = 0.0;

            if (std::isfinite(raw_range)) {
                // Discard measurements that are too close.
                if (raw_range < msg->range_min) {
                    continue;
                }
                if (raw_range < msg->range_max) {
                    obstacle_hit = true;
                    ray_length = raw_range;
                } else {
                    // No obstacle detected.
                    ray_length = msg->range_max;
                }
            } else if (std::isinf(raw_range) && raw_range > 0.0) {
                // Positive infinity means no return.
                ray_length = msg->range_max;
            } else {
                // Disregard NaN and negative infinity.
                continue;
            }

            const double angle =
                static_cast<double>(msg->angle_min) +
                static_cast<double>(i) *
                static_cast<double>(msg->angle_increment);

            const Eigen::Vector3d endpoint_body{ray_length * std::cos(angle), ray_length * std::sin(angle), 0.0};

            // Rotate LiDAR from FLU to ENU and translate by drone position.
            const Eigen::Vector3d endpoint_enu = position_enu_ + q_enu_flu_ * endpoint_body;

            const double endpoint_x = endpoint_enu.x();
            const double endpoint_y = endpoint_enu.y();

            const double dx = endpoint_x - sensor_x;
            const double dy = endpoint_y - sensor_y;

            const double world_distance = std::hypot(dx, dy);
            const int number_of_steps = std::max(1, static_cast<int>(std::ceil(world_distance / ray_step)));

            // Mark all cells before the endpoint free.
            for (int step = 0; step < number_of_steps; ++step) {
                const double alpha = static_cast<double>(step) / static_cast<double>(number_of_steps);
                const double sample_x = sensor_x + alpha * dx;
                const double sample_y = sensor_y + alpha * dy;

                Coord free_cell;

                if (world2grid(sample_x, sample_y, free_cell)) {
                    free_cells.push_back(free_cell);
                }
            }

            Coord endpoint_cell;

            if (!world2grid(endpoint_x, endpoint_y, endpoint_cell)) {
                continue;
            }

            if (obstacle_hit) {
                occupied_cells.push_back(endpoint_cell);
            } else {
                free_cells.push_back(endpoint_cell);
            }
        }

        bool planning_cost_changed = false;
        int changed_states = 0;

        // Apply free observations first.
        for (const Coord& cell : free_cells) {
            const int current_state = belief_grid_->state(cell);

            // Do not erase persistent occupied cells in the static maze.
            if (current_state == 0 || current_state == 1) {
                continue;
            }
            planning_cost_changed = apply_observation(cell, 0) || planning_cost_changed;
            ++changed_states;
        }

        const double inflation_radius_cells = inflation_radius_ / map_resolution_;

        const int inflation_extent = static_cast<int>(std::ceil(inflation_radius_cells));

        const double inflation_radius_squared = inflation_radius_cells * inflation_radius_cells;

        // Apply occupied observations with circular inflation.
        for (const Coord& obstacle_cell : occupied_cells) {
            for (int dy = -inflation_extent; dy <= inflation_extent; dy++) {
                for (int dx = -inflation_extent; dx <= inflation_extent; dx++) {
                    const double distance_squared =
                        static_cast<double>(dx * dx + dy * dy);

                    if (distance_squared > inflation_radius_squared) {
                        continue;
                    }

                    const Coord inflated_cell{obstacle_cell.x + dx, obstacle_cell.y + dy};

                    if (inflated_cell.x < 0 || inflated_cell.x >= map_width_ || inflated_cell.y < 0 || inflated_cell.y >= map_height_){
                        continue;
                    }

                    // Never mark the current drone cell occupied.
                    if (inflated_cell.x == start_cell_.x && inflated_cell.y == start_cell_.y){
                        continue;
                    }

                    if (belief_grid_->state(inflated_cell) == 1) {
                        continue;
                    }

                    planning_cost_changed =
                        apply_observation(inflated_cell, 1) ||
                        planning_cost_changed;

                    ++changed_states;
                }
            }
        }

        // Repair D* Lite once after processing the full scan.
        if (planning_cost_changed) {
            planner_->computeShortestPath();
        }

        publish_belief_map();
        publish_path();

        RCLCPP_DEBUG(
            get_logger(),
            "Processed scan: %d belief-state changes, %zu obstacle endpoints",
            changed_states,
            occupied_cells.size()
        );
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

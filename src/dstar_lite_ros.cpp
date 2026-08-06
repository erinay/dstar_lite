#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class DStarLiteNode: public rclcpp::Node{
    public: DStarLiteNode(): Node("dsar_lite_node"){
        rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
			sensor_qos.best_effort();
			sensor_qos.durability_volatile();


        // Subscribers
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", sensor_qos, std::bind(&DStarLiteNode::odom_callback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos, std::bind(&DStarLiteNode::scan_callback, this, std::placeholders::_1));

        // Publishers
        belief_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/belief_map", 1);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 1);
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/waypoint", 1);

        initialize();
    }

    private:
    
    // Publishers & Subscripers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
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
    std::string planning_frame_{"odom"};

    Coord start_cell_;
    Coord goal_cell_;

    std::unique_ptr<Grid> belief_grid_;
    std::unique_ptr<DStarLite> planner_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    //Not really confident about tf stuff so adding as recommended
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
        world2grid(initial_start_x_, initial_start_y_, start_cell_);
        world2grid(goal_x_, goal_y_, goal_cell_);
        planner_ = std::make_unique<DStarLite>(*belief_grid_, start_cell_, goal_cell_);
        planner_->computeShortestPath();
    }
    void world2grid(double world_x, double world_y, Coord& cell) {
        cell.x = (int) (std::ceil((world_x-origin_x_)/map_resolution_));
        cell.y = (int) (std::ceil((world_y-origin_y_)/map_resolution_));
    }

    geometry_msgs::msg::Point grid2world(const Coord& cell) const {
        geometry_msgs::msg::Point point;
        point.x = origin_x_+((double)(cell.x) + 0.5)*map_resolution_;
        point.y = origin_y_+((double)(cell.y) + 0.5)*map_resolution_;
        point.z = 0.0;
        return point;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        Coord new_start;
        world2grid(msg->pose.pose.position.x, msg->pose.pose.position.y, new_start);
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

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg){
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(planning_frame_,msg->header.frame_id,rclcpp::Time(msg->header.stamp),rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException& exception) {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Cannot transform scan from '%s' to '%s': %s",msg->header.frame_id.c_str(),planning_frame_.c_str(),exception.what());
            return;
        }

        /*
        * Transform the LiDAR origin into the planning frame.
        */
        geometry_msgs::msg::Point sensor_origin_scan;
        sensor_origin_scan.x = 0.0;
        sensor_origin_scan.y = 0.0;
        sensor_origin_scan.z = 0.0;

        geometry_msgs::msg::Point sensor_origin_world;

        tf2::doTransform(sensor_origin_scan,sensor_origin_world,transform);
        /*
        * Free observations are applied first. * Occupied endpoints are applied afterward so that an occupied
        * endpoint wins if multiple rays touch the same discrete cell.
        */
        std::vector<Coord> free_cells;
        std::vector<Coord> occupied_cells;

        const double ray_step = 0.5 * map_resolution_;

        for (std::size_t i = 0;i < msg->ranges.size(); ++i) {
            const double raw_range = static_cast<double>(msg->ranges[i]);

            bool obstacle_hit = false;
            double ray_length = 0.0;

            if (std::isfinite(raw_range)) {
                //Discard measurements that are too close.
                if (raw_range < msg->range_min) {continue;}
                if (raw_range < msg->range_max) {
                    obstacle_hit = true;
                    ray_length = raw_range;
                } else {
                    //else, no obstacle detected
                    ray_length = msg->range_max;
                }
            } else if (std::isinf(raw_range) &&raw_range > 0.0){
                //inf = no return
                ray_length = msg->range_max;
            } else {
                // disregard NaN/-inf
                continue;
            }

            const double angle =static_cast<double>(msg->angle_min) + static_cast<double>(i) * static_cast<double>(msg->angle_increment);

            geometry_msgs::msg::Point endpoint_scan;
            endpoint_scan.x = ray_length * std::cos(angle);
            endpoint_scan.y = ray_length * std::sin(angle);
            endpoint_scan.z = 0.0;
            geometry_msgs::msg::Point endpoint_world;

            tf2::doTransform(endpoint_scan,endpoint_world,transform);

            const double dx = endpoint_world.x - sensor_origin_world.x;
            const double dy = endpoint_world.y - sensor_origin_world.y;
            const double world_distance = std::hypot(dx, dy);
            const int number_of_steps = std::max( 1, static_cast<int>( std::ceil(world_distance/ray_step)));

            // Mark all cells before endpoint fre
            for (int step = 0; step < number_of_steps; ++step) {
                const double alpha = static_cast<double>(step) / static_cast<double>(number_of_steps);
                const double sample_x = sensor_origin_world.x +alpha * dx;
                const double sample_y = sensor_origin_world.y + alpha * dy;
                Coord free_cell;
                world2grid( sample_x, sample_y, free_cell);
                free_cells.push_back(free_cell);
            }

            Coord endpoint_cell;
            world2grid(endpoint_world.x, endpoint_world.y, endpoint_cell);

            if (obstacle_hit){ 
                occupied_cells.push_back( endpoint_cell);
            } else {
                //no obstacle
                free_cells.push_back(endpoint_cell);
            }
        }

        bool planning_cost_changed = false;
        int changed_states = 0;

        //Apply free observations first.
        for (const Coord& cell : free_cells) {
            if (belief_grid_->state(cell) == 0) {
                continue;
            }
            planning_cost_changed = apply_observation(cell, 0) || planning_cost_changed;
            ++changed_states;
        }

        
        //Apply occupied endpoints second.
        for (const Coord& cell : occupied_cells) {
            if (belief_grid_->state(cell) == 1) {
                continue;
            }
            planning_cost_changed = apply_observation(cell, 1) || planning_cost_changed;
            ++changed_states;
        }

        //Repair D* Lite once after processing the full scan.
        if (planning_cost_changed) {
            planner_->computeShortestPath();
        }

        publish_belief_map();
        publish_path();

        RCLCPP_DEBUG(get_logger(),"Processed scan: %d belief-state changes",changed_states);
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
        for (std::size_t i = 0; i < path.size();++i)
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

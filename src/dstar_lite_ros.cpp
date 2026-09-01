#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dstar_lite.hpp"
#include "grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
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

        // Use PX4 odometry by default.  During Gazebo raw-lidar validation,
        // robot_pose_topic supplies the exact simulator pose in planning_frame
        // so D* and the voxel slice share one world frame.
        if (robot_pose_topic_.empty()) {
            odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                odometry_topic_, sensor_qos,
                std::bind(&DStarLiteNode::odom_callback, this, std::placeholders::_1));
        } else {
            robot_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                robot_pose_topic_, 10,
                std::bind(&DStarLiteNode::robot_pose_callback, this, std::placeholders::_1));
        }
        voxel_slice_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/voxel_slice", 1, std::bind(&DStarLiteNode::voxel_slice_callback, this, std::placeholders::_1));

        // Publishers
        belief_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/belief_map", 1);
        // Keep the planner path distinct from Spark's mapping trajectory,
        // which also publishes nav_msgs/Path on /path in this stack.
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/dstar_path", 1);
        // The goal is fixed at startup (goal_x_/goal_y_ params) and never moves, so a subscriber
        // spinning up after this constructor still needs it -- transient_local durability makes
        // the single publish below available to a late-joining subscriber (e.g. control_node).
        rclcpp::QoS goal_qos(1);
        goal_qos.transient_local();
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/goal", goal_qos);

        initialize();
    }

    private:
    
    // Publishers & Subscripers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr voxel_slice_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr belief_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr goal_publisher_;

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
    // A finite, distance-based penalty near a wall. Unlike hard obstacle
    // inflation, this keeps narrow passages traversable while preferring the
    // middle of a corridor.
    double wall_cost_radius_{0.50};
    double wall_cost_gain_{4.0};
    // A hard centre-point clearance from an observed obstacle.  A cell inside
    // this radius is marked occupied for planning, so a corridor must be wide
    // enough for the vehicle rather than merely containing a one-cell route.
    double hard_clearance_radius_{0.50};

    const float look_ahead = 3.0f;

    bool have_odom_;
    std::string planning_frame_{"odom"};
    std::string robot_pose_topic_;
    std::string odometry_topic_;

    Coord start_cell_;
    Coord goal_cell_;

    std::unique_ptr<Grid> belief_grid_;
    std::unique_ptr<DStarLite> planner_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    Eigen::Vector3d position_enu_{0.0, 0.0, 0.0};
    Eigen::Quaterniond q_enu_flu_{1.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d R_NED2ENU, R_FLU2FRD, R_ENU_FLU;

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
        planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
        robot_pose_topic_ = declare_parameter<std::string>("robot_pose_topic", "");
        odometry_topic_ = declare_parameter<std::string>(
            "odometry_topic", "/fmu/out/vehicle_odometry");
        wall_cost_radius_ = declare_parameter<double>("wall_cost_radius", 0.50);
        wall_cost_gain_ = declare_parameter<double>("wall_cost_gain", 4.0);
        hard_clearance_radius_ = declare_parameter<double>("hard_clearance_radius", 0.50);
        if (wall_cost_radius_ < 0.0 || wall_cost_gain_ < 0.0 ||
            hard_clearance_radius_ < 0.0) {
            throw std::runtime_error(
                "wall_cost_radius, wall_cost_gain, and hard_clearance_radius "
                "must be non-negative");
        }
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
        publish_goal();
    }

    void publish_goal(){
        geometry_msgs::msg::Vector3 goal_msg;
        goal_msg.x = goal_x_;
        goal_msg.y = goal_y_;
        // Planning is 2D; z is unused by this node and left at 0 for now.
        goal_msg.z = 0.0;
        goal_publisher_->publish(goal_msg);
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

        const Eigen::Matrix3d R_ENU_FLU = R_NED2ENU* q_measured.toRotationMatrix() * R_FLU2FRD;
        q_enu_flu_ = Eigen::Quaterniond(R_ENU_FLU);

        q_enu_flu_.normalize();

        robot_pose_.position.x = position_enu_.x();
        robot_pose_.position.y = position_enu_.y();
        robot_pose_.position.z = position_enu_.z();

        robot_pose_.orientation.w = q_enu_flu_.w();
        robot_pose_.orientation.x = q_enu_flu_.x();
        robot_pose_.orientation.y = q_enu_flu_.y();
        robot_pose_.orientation.z = q_enu_flu_.z();

        update_robot_position();
    }

    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != planning_frame_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring robot pose in frame '%s'; expected planning_frame '%s'",
                msg->header.frame_id.c_str(), planning_frame_.c_str());
            return;
        }

        robot_pose_ = msg->pose;
        position_enu_ << robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z;
        q_enu_flu_ = Eigen::Quaterniond(
            robot_pose_.orientation.w,
            robot_pose_.orientation.x,
            robot_pose_.orientation.y,
            robot_pose_.orientation.z);
        if (q_enu_flu_.norm() < 1e-6) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring robot pose with invalid quaternion");
            return;
        }
        q_enu_flu_.normalize();
        update_robot_position();
    }

    void update_robot_position()
    {
        const bool first_odom = !have_odom_;
        have_odom_=true;

        Coord new_start;

        if (!world2grid(position_enu_.x(), position_enu_.y(), new_start))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Robot outside belief grid in %s: (%.2f, %.2f)",
                planning_frame_.c_str(), position_enu_.x(), position_enu_.y());
            return;
        }
        world2grid(position_enu_.x(), position_enu_.y(), new_start);
        const bool changed_cell = new_start.x != start_cell_.x || new_start.y != start_cell_.y;
        
        if (!changed_cell && !first_odom) {
            return;
        }
        if (changed_cell) {
            start_cell_=new_start;
            planner_->moveStart(start_cell_);
        }
        planner_->computeShortestPath();

        publish_path();
    }   

    bool apply_observation(const Coord& cell, int observed_state){
        // check if observation should be applied
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

    void voxel_slice_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->info.resolution <= 0.0f || msg->data.size() !=
            static_cast<std::size_t>(msg->info.width) * msg->info.height) {
            RCLCPP_WARN(get_logger(), "Ignoring malformed voxel slice");
            return;
        }

        // Build the desired planning grid from this complete source-map
        // snapshot. This lets clearance cells disappear as soon as the source
        // obstacle disappears, rather than leaving stale inflated obstacles.
        std::vector<int> desired_states(
            static_cast<std::size_t>(map_width_) * static_cast<std::size_t>(map_height_), -1);
        const auto desired_index = [this](const Coord& cell) {
            return static_cast<std::size_t>(cell.x) +
                static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(map_width_);
        };

        for (std::uint32_t y = 0; y < msg->info.height; ++y) {
            for (std::uint32_t x = 0; x < msg->info.width; ++x) {
                const std::size_t source_index =
                    static_cast<std::size_t>(x) + static_cast<std::size_t>(y) * msg->info.width;
                const std::int8_t occupancy = msg->data[source_index];
                const int state = occupancy < 0 ? -1 : (occupancy >= occupied_threshold ? 1 : 0);

                Coord target_cell;
                const double world_x = msg->info.origin.position.x +
                    (static_cast<double>(x) + 0.5) * msg->info.resolution;
                const double world_y = msg->info.origin.position.y +
                    (static_cast<double>(y) + 0.5) * msg->info.resolution;
                if (!world2grid(world_x, world_y, target_cell) ||
                    (target_cell == start_cell_)) {
                    continue;
                }

                const std::size_t index = desired_index(target_cell);
                // A source grid with a different resolution can map several
                // cells into one planner cell. Occupied wins over free; free
                // wins over unknown.
                if (state == 1 || (state == 0 && desired_states[index] != 1)) {
                    desired_states[index] = state;
                }
            }
        }

        // Multi-source Dijkstra gives every planner cell its Euclidean-like
        // distance from the nearest observed obstacle.  It is much cheaper
        // than comparing every map cell against every wall point.
        const std::size_t cell_count = desired_states.size();
        std::vector<double> wall_distance(cell_count,
            std::numeric_limits<double>::infinity());
        using DistanceEntry = std::pair<double, Coord>;
        const auto farther_first = [](const DistanceEntry& left,
                                      const DistanceEntry& right) {
            return left.first > right.first;
        };
        std::priority_queue<DistanceEntry, std::vector<DistanceEntry>,
            decltype(farther_first)> open_distances(farther_first);

        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const Coord cell{x, y};
                if (desired_states[desired_index(cell)] == 1) {
                    wall_distance[desired_index(cell)] = 0.0;
                    open_distances.push({0.0, cell});
                }
            }
        }

        const std::array<Coord, 8> distance_neighbors{{
            {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
            {1, 0}, {-1, 1}, {0, 1}, {1, 1},
        }};
        while (!open_distances.empty()) {
            const auto [distance, cell] = open_distances.top();
            open_distances.pop();
            if (distance > wall_distance[desired_index(cell)] + 1e-9) {
                continue;
            }

            for (const Coord& direction : distance_neighbors) {
                const Coord neighbor{cell.x + direction.x, cell.y + direction.y};
                if (!belief_grid_->inBounds(neighbor)) {
                    continue;
                }
                const double step = (direction.x != 0 && direction.y != 0)
                    ? std::sqrt(2.0) * map_resolution_
                    : map_resolution_;
                const double candidate = distance + step;
                const std::size_t neighbor_index = desired_index(neighbor);
                if (candidate + 1e-9 >= wall_distance[neighbor_index]) {
                    continue;
                }
                wall_distance[neighbor_index] = candidate;
                open_distances.push({candidate, neighbor});
            }
        }

        bool state_changed = false;
        bool planning_cost_changed = false;
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const Coord target_cell{x, y};
                if (target_cell == start_cell_) {
                    continue;
                }

                const int source_state = desired_states[desired_index(target_cell)];
                const double wall_distance_m = wall_distance[desired_index(target_cell)];
                // Keep the source-map state separate from the planning state:
                // the distance transform is seeded only by real observations,
                // not by cells previously made impassable for clearance.
                const bool lacks_clearance =
                    source_state != 1 && hard_clearance_radius_ > 0.0 &&
                    wall_distance_m < hard_clearance_radius_;
                const int planning_state = lacks_clearance ? 1 : source_state;
                double traversal_cost = 1.0;
                if (planning_state != 1 && wall_cost_radius_ > 0.0 &&
                    wall_distance_m < wall_cost_radius_)
                {
                    const double normalized_distance =
                        1.0 - wall_distance_m / wall_cost_radius_;
                    traversal_cost += wall_cost_gain_ *
                        normalized_distance * normalized_distance;
                }

                const double old_cost = belief_grid_->traversalCost(target_cell);
                if (belief_grid_->state(target_cell) == planning_state &&
                    costs_equal(old_cost, planning_state == 1
                        ? std::numeric_limits<double>::infinity() : traversal_cost)) {
                    continue;
                }
                state_changed = true;
                planner_->updateCell(target_cell, planning_state, traversal_cost);
                const double new_cost = belief_grid_->traversalCost(target_cell);
                planning_cost_changed |= !costs_equal(old_cost, new_cost);
            }
        }

        if (!state_changed) {
            return;
        }
        if (planning_cost_changed) {
            planner_->computeShortestPath();
        }
        publish_belief_map();
        publish_path();
    }

    void obstacle_callback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg){
        constexpr double pi = 3.14159265358979323846;
        constexpr double degrees_to_radians = pi / 180.0;
        constexpr std::uint16_t unknown_distance = std::numeric_limits<std::uint16_t>::max();
        const std::uint32_t clear_value = static_cast<std::uint32_t>(msg->max_distance) + 1U;

        if (!have_odom_) {
            return;
        }
        if (msg->frame != px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring ObstacleDistance frame %u; x500_lidar_2d must publish BODY_FRD (%u)",
                msg->frame, px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD);
            return;
        }

        std::vector<Coord> occupied_cells;
        std::vector<Eigen::Vector3d> free_ray_endpoints;
        // Raw ObstacleDistance is not part of the active mapping path. Keep
        // its legacy callback free of hard inflation as well.
        const int inflation_extent = 0;

        for (std::size_t i = 0; i < msg->distances.size(); i++){
            const std::uint16_t distance_cm = msg->distances[i];

            if (distance_cm == unknown_distance) {
                continue; //unknown distance
            }
            const bool clear_ray = static_cast<std::uint32_t>(distance_cm) == clear_value;
            if (!clear_ray &&
                (distance_cm < msg->min_distance || distance_cm > msg->max_distance)) {
                continue; //invalid obstacle measuremtns
            }

            const double range = clear_ray
                ? static_cast<double>(msg->max_distance) / 100.0
                : static_cast<double>(distance_cm) / 100.0;
            // ObstacleDistance angles are clockwise in BODY_FRD. Convert the
            // endpoint to FLU before rotating it into the ENU planning frame.
            const double angle_frd =
                (static_cast<double>(msg->angle_offset) +
                 static_cast<double>(i) * static_cast<double>(msg->increment)) * degrees_to_radians;
            const Eigen::Vector3d endpoint_flu{
                range * std::cos(angle_frd), -range * std::sin(angle_frd), 0.0};
            const Eigen::Vector3d endpoint_enu = position_enu_+q_enu_flu_.toRotationMatrix()*endpoint_flu;
            free_ray_endpoints.push_back(endpoint_enu);

            if (clear_ray) {
                continue;
            }

            Coord obstacle_cell;
            if(!world2grid(endpoint_enu.x(), endpoint_enu.y(), obstacle_cell)){
                continue;
            }
            occupied_cells.push_back(obstacle_cell);
        }

        bool planning_cost_changed=false;

        // Every measured ray is free up to its endpoint. Marking these cells
        // prevents the belief map from remaining unknown between obstacles.
        for (const Eigen::Vector3d& endpoint_enu : free_ray_endpoints) {
            const Eigen::Vector2d ray =
                (endpoint_enu - position_enu_).head<2>();
            const int samples = std::max(
                1, static_cast<int>(std::ceil(ray.norm() / (0.5 * map_resolution_))));

            for (int sample = 1; sample < samples; ++sample) {
                const Eigen::Vector3d point = position_enu_ +
                    (static_cast<double>(sample) / samples) * (endpoint_enu - position_enu_);
                Coord free_cell;
                if (!world2grid(point.x(), point.y(), free_cell) ||
                    (free_cell == start_cell_)) {
                    continue;
                }
                planning_cost_changed |= apply_observation(free_cell, 0);
            }
        }

        for (const Coord& obstacle_cell: occupied_cells){
            for (int dy=-inflation_extent; dy<=inflation_extent; dy++){
                for(int dx=-inflation_extent; dx<=inflation_extent; dx++){
                    const double distance_squared = (double) (dx*dx+dy*dy);
                    if((distance_squared)>(inflation_extent*inflation_extent)){
                        continue;
                    }
                    const Coord inflated_cell{obstacle_cell.x+dx, obstacle_cell.y+dy};

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
                    if(apply_observation(inflated_cell,1)){
                        planning_cost_changed=true;
                    }
                }
            }
        }

        if(planning_cost_changed){
            planner_->computeShortestPath();
        }
        publish_belief_map();
        publish_path();

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
    
    // Instead of publishing waypoint, prune to smaller segments
    std::vector<Coord> prune(const std::vector<Coord>& path){
        if(path.empty()){
            return{};
        }

        // First, collapse colinear runs
        if(path.size()<2){
            return path; //nothing to remove
        }
        std::vector<Coord> cleaned_path;
        cleaned_path.reserve(path.size()); // path is at most the current size of  the path
        cleaned_path.push_back(path.front());
        
        int dx_before = 0;
        int dy_before = 0;
        bool is_turn = false;

        for(std::size_t i=1; i<path.size(); i++){
            int dx = path[i].x - path[i-1].x;
            int dy = path[i].y - path[i-1].y;
            
            if(i != 1) {
                is_turn = dx_before!=dx || dy_before!=dy;
            }
            
            if(is_turn){
                cleaned_path.push_back(path[i-1]);
            }
            dx_before=dx;
            dy_before=dy;
        }
        // BUGFIX: the loop above only ever pushes path[i-1] (turn points before the end), so
        // path.back() -- which is always exactly the goal cell, per extractPath() -- was never
        // included. That silently truncated every published path one segment short of the goal
        // (and, for a path with no turns at all, left cleaned_path with only the start point).
        cleaned_path.push_back(path.back());

        // Next, los
        if (cleaned_path.size()<=2){
            return cleaned_path;
        } 
        std::vector<Coord> pruned_path;
        pruned_path.reserve(cleaned_path.size());
        pruned_path.push_back(cleaned_path.front());
        std::size_t current = 0;

        while(current<cleaned_path.size()-1){
            // at each point, try connecting to goal, and backtrack form there
            std::size_t next = cleaned_path.size()-1;
            while(next>current+1){
                if(line_of_sight_free(cleaned_path[current], cleaned_path[next])){
                    break;
                }
                next -= 1;
            }
            pruned_path.push_back(cleaned_path[next]);
            current=next;
        }
        return pruned_path;
    }

    bool line_of_sight_free(const Coord& from, const Coord& to ){
        //check if diagonal path between from and to is free
        const int dx = to.x-from.x;
        const int dy = to.y-from.y;
        int x = from.x;
        int y = from.y;

        // Figure out how many grid boundaries must be crossed
        const int nx = std::abs(dx);
        const int ny = std::abs(dy);

        int sign_x;
        if (dx>0){
            sign_x=1;
        }else if(dx<0){
            sign_x=-1;
        }else{
            sign_x=0;
        }
        int sign_y;
        if (dy>0){
            sign_y=1;
        }else if(dy<0){
            sign_y=-1;
        }else{
            sign_y=0;
        }

        // Track how far we've gotteen to nx/ny
        int ix = 0;
        int iy = 0;

        // Helper function to check if given cell is free, 
        auto is_free = [this](const Coord& cell) {
            if (!belief_grid_->inBounds(cell)) {
                return false; //first check if it's even in map
            }
            return belief_grid_->state(cell) == 0; //only ok shortcut if free 
        };

        // Iterae thorugh path:
        while(ix<nx || iy <ny){
            //figures out to go horizontal (lhs) or vertical (rhs) if we don't want pure diagonal
            const long lhs = static_cast<long>(1 + 2 * ix) * static_cast<long>(ny);
            const long rhs = static_cast<long>(1 + 2 * iy) * static_cast<long>(nx);
            if(lhs==rhs){ //Pure diagonal!, check all 3 cells it passes through
                const Coord side_x{x + sign_x, y};
                const Coord side_y{x, y + sign_y};
                const Coord diagonal{x + sign_x, y + sign_y};

                if(!is_free(side_x)||!is_free(side_y)||!is_free(diagonal)){
                    return false;
                }
                // else, keep going toward nx,ny
                x+=sign_x;
                y+=sign_y;
                ix++;
                iy++;
            } else if(lhs<rhs){ //move horizontally
                x+=sign_x;
                ix++;
                if(!is_free(Coord{x,y})){
                    return false;
                }
            } else{ // move veritcally
                y+=sign_y;
                iy++;
                if(!is_free(Coord{x,y})){
                    return false;
                }
            }

        }
        // If it iterates through entire path to nx,ny, return true
        return true;
    }    

    void publish_path(){
        if (!planner_) {
            return;
        }

        const std::vector<Coord> path = planner_->extractPath();
        const std::vector<Coord> pruned_path = prune(path);

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = planning_frame_;

        for (std::size_t i = 0; i < pruned_path.size();i++)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = path_msg.header;
            pose_msg.pose.position = grid2world(pruned_path[i]);

            if (i + 1 < pruned_path.size()) {
                pose_msg.pose.orientation = path_orientation(pruned_path[i], pruned_path[i + 1]);
            } else {
                pose_msg.pose.orientation.w = 1.0;
            }
            path_msg.poses.push_back(pose_msg);
        }

        path_publisher_->publish(path_msg);
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

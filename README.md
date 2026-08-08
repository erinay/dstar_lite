# D* Lite ROS 2 nodes

## TurtleBot / standard 2D lidar node

`turtlebot_dstar_lite_node` is independent of PX4. It uses the standard ROS 2
interfaces that TurtleBot3 publishes:

- Input: `nav_msgs/msg/Odometry` on `/odom` in the `odom` frame.
- Input: `sensor_msgs/msg/LaserScan` on `/scan`, using standard ROS FLU scan
  angles (positive angles turn left).
- Input: `geometry_msgs/msg/PoseStamped` on `/goal_pose`.
- Output: `nav_msgs/msg/OccupancyGrid` on `/belief_map`.
- Output: `nav_msgs/msg/Path` on `/path`.
- Output: a look-ahead `geometry_msgs/msg/PoseStamped` on `/waypoint`.

The node does not publish `/cmd_vel`; use the path or waypoint with the
controller of your choice. This keeps planning separate from base control and
allows it to be tested safely in RViz before autonomous driving.

Build and run the node:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/drone_ws
colcon build --packages-select dstar_lite --symlink-install
source install/setup.bash
ros2 run dstar_lite turtlebot_dstar_lite_node
```

The default 10 m by 10 m map is centred at the `odom` origin and starts with a
goal at `(2.0, 0.0)`. In RViz, set the fixed frame to `odom` and publish a new
goal on `/goal_pose`. For a Burger-sized TurtleBot, the default 0.20 m
inflation radius is a reasonable starting point.

To use the TurtleBot3 Gazebo simulation, install `ros-jazzy-turtlebot3-gazebo`
and then launch your chosen TurtleBot3 world before starting this node.

# D* Lite ROS 2 nodes

## Active full-stack LiDAR contract

`scripts/fullstack_tmux_start.sh` uses the following 3-D LiDAR path. This is
the current contract for the PX4/Gazebo/Poisson stack.

```text
Gazebo PointCloudPacked
  -> /Drone1/lidar/raw_cloud       (unmodified, LiDAR frame; Bonxai input)
  -> Bonxai finite/range/self mask
  -> /Drone1/lidar/point_cloud     (fresh propeller/body-filtered scan)
       -> Spark Fast-LIO and Poisson

Bonxai persistent occupied voxels (0.10 m, odom frame)
  -> /mapping_scan                 (RViz)
  -> /Drone1/sdf_map/occupancy     (Poisson)
  -> /voxel_slice                  (D* Lite's 2-D projection)
```

There is **no cropped PointCloud2** in the active path. Bonxai accepts the
complete usable scan: non-finite points, points closer than 0.28 m or farther
than 40 m, and points in the drone self-filter box are removed. The self box
is tested in body axes after the configured LiDAR-to-body rotation, so it
removes the drone/propeller returns even though the outgoing cloud remains in
the original `Drone1/livox_frame/lidar` frame. Its active 30 degree sensor
pitch is supplied by the tmux launcher. All surviving input fields, including
any intensity or ring fields, are retained on `/Drone1/lidar/point_cloud`.

`point_stride=4` decimates only Bonxai map insertion; Fast-LIO and Poisson see
every accepted point in the fresh filtered scan. A 5 x 5 x 5 m observation
cube is deliberately not active.

D* Lite treats unknown cells as provisionally traversable, but it never leaves
the configured 2-D grid. The active grid spans `x = [-14.0, 30.0)` and
`y = [-4.0, 30.0)` m (44.0 x 34.0 m). It covers the complete maze footprint
and at least a 1.4 m exterior perimeter, allowing the intended long exit route
to the reachable yellow exit cell `(25.0, 0.0)`. Bonxai and D* Lite use the
same bounds and resolution.

The `sdf_map/occupancy` name is historical: it is a `PointCloud2` of all
currently occupied Bonxai voxel centers in `odom`, not a signed-distance map.
It is equivalent in point population to `/mapping_scan` (a separate
publication for the Poisson consumer). Bonxai's map is persistent for the run.
The 2-D `/voxel_slice` is instead bounded to 44.0 x 34.0 m at 0.10 m
resolution, origin `(-14.0, -4.0)` in `odom` (440 x 340 cells). Its vertical
band is dynamic, using the exact Gazebo `odom -> Drone1/gazebo_body` TF on each
scan. This separate frame avoids conflicting with Spark Fast-LIO's estimated
`Drone1/base_link` TF. While body height is at or below 2.0 m it projects the
ground/takeoff band `[0.1, 2.0] m`; above 2.0 m it projects only
`[body_z - 0.1, body_z + 0.1] m`. This affects `/voxel_slice` only—not the
persistent 3-D Bonxai map or the Poisson occupancy topic.

## Gazebo 3D LiDAR to RViz

`gazebo_pointcloud_bridge.launch.py` converts a Gazebo
`gz.msgs.PointCloudPacked` 3D LiDAR topic directly into a ROS
`sensor_msgs/msg/PointCloud2` stream. It is a one-way bridge, so the simulator
remains the sole producer of sensor data.

With Gazebo running, find the LiDAR's Gazebo topic and verify its type:

```bash
gz topic -l | rg -i 'points|point_cloud|lidar'
gz topic -i -t <gazebo-point-cloud-topic>
```

The second command must report `gz.msgs.PointCloudPacked`. Then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 launch dstar_lite gazebo_pointcloud_bridge.launch.py \
  gz_topic:=<gazebo-point-cloud-topic>
```

The cloud is published as `/sim_lidar/points`. In RViz, set **Fixed Frame** to
the cloud's `header.frame_id` (or to a frame connected to it by TF), add a
**PointCloud2** display, and select `/sim_lidar/points`. Use another output
topic if needed with `ros_topic:=/my_lidar/points`.

## LiDAR pose broadcaster nodes

Two standalone nodes publish a `<map_frame> -> <sensor_frame>` TF (plus the
same pose on `/sim_lidar/pose`) for a mapping consumer such as
`pointcloud_bonxai_node` to register scans against:

- `src/gazebo_lidar_tf_broadcaster.cpp` builds
  `gazebo_lidar_tf_broadcaster_node`. It subscribes directly to Gazebo
  Transport's dynamic-pose topic, finds the selected model, and rotates the
  configured sensor offset by the model attitude. Gazebo uses ENU world
  coordinates and FLU body axes, so this node needs no PX4 frame conversion.
  It stamps TF from the Gazebo pose message timestamp rather than the latest
  available transform, so a mapping consumer can look up the pose at the
  scan's own simulation timestamp. This is the broadcaster used by the active
  full-stack contract above.

- `src/px4_lidar_tf_broadcaster.cpp` builds `px4_lidar_tf_broadcaster_node`.
  It subscribes to `/fmu/out/vehicle_odometry`, converts PX4 NED/FRD position
  and attitude to ROS ENU/FLU, applies the configured LiDAR offset, and
  publishes the same transform/pose pair from PX4 odometry instead of
  Gazebo's dynamic pose. Its defaults describe a centred mock LiDAR; set
  `sensor_frame` and the offset parameters to match the model being used.

## Planner clearance and wall preference

`dstar_lite_node` computes an 8-connected distance field from the observed
occupied cells for every `/voxel_slice` update. A cell whose centre is closer
than `hard_clearance_radius` to an obstacle becomes non-traversable. This
closes gaps that cannot fit the vehicle; it is recomputed from each complete
source-map snapshot, so clearance cells disappear if their observed obstacle
disappears.

The full-stack launcher uses `hard_clearance_radius=0.50 m`, matching the
approximately `0.40 m` X500 propeller-envelope radius plus a `0.10 m` margin.
Set it to `0.0` to disable hard clearance. A distinct finite proximity cost
then prefers the centre of the remaining traversable corridors. Its defaults
are `wall_cost_radius=0.50 m` and `wall_cost_gain=4.0`:

```bash
ros2 run dstar_lite dstar_lite_node --ros-args \
  -p hard_clearance_radius:=0.50 \
  -p wall_cost_radius:=0.50 -p wall_cost_gain:=4.0
```

## TurtleBot / standard 2D lidar node

`turtlebot_dstar_lite_node` is independent of PX4. It uses the standard ROS 2
interfaces that TurtleBot3 publishes:

- Input: `nav_msgs/msg/Odometry` on `/odom` in the `odom` frame.
- Input: `sensor_msgs/msg/LaserScan` on `/scan`, using standard ROS FLU scan
  angles (positive angles turn left).
- Input: `geometry_msgs/msg/PoseStamped` on `/goal_pose`.
- Output: `nav_msgs/msg/OccupancyGrid` on `/belief_map`.
- Output: `nav_msgs/msg/Path` on `/dstar_path`.
- Output: a look-ahead `geometry_msgs/msg/PoseStamped` on `/waypoint`.

`turtlebot_waypoint_controller` is the companion controller for simulation.
It follows `/waypoint`, brakes when the forward laser sector is too close, and
publishes `geometry_msgs/msg/TwistStamped` to `/cmd_vel` (the type required by
the Jazzy TurtleBot3 Gazebo bridge). It is intentionally independent of the
PX4 controller.

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

## One-command TurtleBot3 validation

The workspace includes a separate test stack that starts the TurtleBot3 Burger
world, scan mapper, D* Lite planner, and the simple waypoint controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 launch dstar_lite turtlebot3_dstar_demo.launch.py
```

It spawns at `(-2.0, -0.5)` and initially targets `(2.0, 0.0)`. For RViz use
fixed frame `odom` and add `/tb3/belief_map` (Map) and `/tb3/path` (Path).
To select a different target, set the RViz **2D Goal Pose** topic to
`/tb3/goal_pose`; both the planner and controller then use that new goal.
The simulation is headless by default so the test is independent of Gazebo GUI
availability; use `gui:=true` when the local Gazebo client is working.
It also uses the dedicated Gazebo transport partition `dstar_lite_tb3`, so it
does not share Gazebo's `/clock` or `/stats` transport topics with PX4 worlds.

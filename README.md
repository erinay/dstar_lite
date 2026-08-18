# D* Lite ROS 2 nodes

## PX4/Gazebo LiDAR mapping and D* Lite

This package contains two ways to turn a 2D LiDAR stream into an OctoMap and
a 2D grid for D* Lite:

1. The **raw Gazebo scan path** is the recommended simulation-validation path.
   It preserves every `LaserScan` ray and uses Gazebo's ground-truth pose.
2. The **PX4 `ObstacleDistance` path** consumes the sectorized distance message
   emitted by PX4 and reconstructs the map from PX4 odometry.

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

### X500 3D LiDAR OctoMap launch

For the workspace's `x500_lidar_3d` model in `maze_2d`, use the complete 3D
mapping pipeline instead of the bridge-only launch:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/drone_ws
colcon build --packages-select dstar_lite --symlink-install
source install/setup.bash
ros2 launch dstar_lite px4_raw_lidar_3d_octomap.launch.py
```

It bridges this model's raw Gazebo `scan/points` topic, publishes the exact
Gazebo pose transform `map -> link`, and inserts the registered cloud into a
persistent OctoMap. The outputs are:

- `/sim_lidar/points` — raw sensor-frame `PointCloud2`.
- `/mapping_scan` — filtered, world-frame points inserted into the map.
- `/octomap_binary` — 3D binary OctoMap.
- `/voxel_slice` — planner-friendly 2D projection.
- `/belief_map`, `/path`, and `/waypoint` — D* Lite's map, path, and next
  waypoint, planned from `/sim_lidar/pose` to `(12.5, -2.75)` in `map`.

Open the supplied RViz configuration with
`rviz2 -d $(ros2 pkg prefix dstar_lite)/share/dstar_lite/rviz/dstar_maze.rviz`;
its fixed frame is `map` and it already displays `/mapping_scan` and `/voxel_slice`.
The 3D mapper keeps every fourth point by default, which is appropriate for
the 720×32, 30 Hz simulated LiDAR. Tune `point_stride` in the launch file if
you need a denser map or lower CPU use.

The raw `/sim_lidar/points` stream includes the X500's own propeller returns,
as a physical LiDAR would. The mapper excludes their measured `link`-frame
box before publishing `/mapping_scan` or inserting the OctoMap, so use
`/mapping_scan` in RViz to inspect the filtered cloud. The box can be disabled
or adjusted through the `self_filter_*` parameters in the 3D launch file.

For the 3D pipeline, prefer the dedicated configuration. It has no `/belief_map`
display, which belongs to the optional D* Lite planner and otherwise reports
"No map received" when the planner is not running:

```bash
rviz2 -d $(ros2 pkg prefix dstar_lite)/share/dstar_lite/rviz/x500_lidar_3d_octomap.rviz
```

Both mappers publish `/octomap_binary`, `/voxel_slice`, `/mapping_pose`, and
`/mapping_scan`; run only one mapper at a time.

### Raw Gazebo LiDAR launch

The `px4_raw_lidar_octomap.launch.py` launch file starts the complete
ground-truth mapping/planning path after PX4 and Gazebo are already running:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/drone_ws
colcon build --packages-select dstar_lite --symlink-install
source install/setup.bash
ros2 launch dstar_lite px4_raw_lidar_octomap.launch.py
```

By default, the launch selects `x500_mock_lidar_2d`. Its real Gazebo
`gpu_lidar` is centred on the vehicle (`x=0`, `y=0`) and its ray origin is
`z=0.315 m` above `base_link`; it scans the actual `maze_2d` walls. There is
no separate mock room. Select the existing offset LiDAR model instead with:

```bash
ros2 launch dstar_lite px4_raw_lidar_octomap.launch.py lidar_model:=x500_lidar_2d
```

The two supported model choices, their sensor frames, offsets, and Gazebo scan
topic are kept together in `LIDAR_MODELS` in the launch file. The launch then:

```text
Gazebo raw LaserScan
    -> ros_gz_bridge (`/sim_lidar/scan`)
    -> laser_scan_octomap_node
    -> `/octomap_binary`, `/voxel_slice`, `/mapping_pose`, `/mapping_scan`
    -> dstar_lite_node (from `/voxel_slice`)
    -> `/belief_map`, `/path`, `/waypoint`

Gazebo dynamic pose
    -> gazebo_lidar_tf_broadcaster_node
    -> `map -> <sensor frame>` TF and `/sim_lidar/pose`
    -> laser_scan_octomap_node and dstar_lite_node
```

It configures the mapper and planner with the same 20.5 m by 17.0 m,
0.10 m-resolution map, with origin `(-6.5, -3.0)`. D* Lite runs in the `map`
frame, takes its robot pose from `/sim_lidar/pose`, and starts with goal
`(12.5, -2.75)`. Consequently, `/belief_map`, `/path`, `/waypoint`, and
`/voxel_slice` all refer to the same world frame.

#### Launch and node parameter examples

The launch file exposes one argument, `lidar_model`. Use the default centred
mock LiDAR or select the offset X500 LiDAR model explicitly:

```bash
# Default: x500_mock_lidar_2d
ros2 launch dstar_lite px4_raw_lidar_octomap.launch.py

# Existing X500 LiDAR model: frame `link`, offset (0.12, 0.0, 0.315) m
ros2 launch dstar_lite px4_raw_lidar_octomap.launch.py \
  lidar_model:=x500_lidar_2d
```

For debugging or a custom world, start the raw-scan nodes separately. These
commands reproduce the default launch configuration for
`x500_mock_lidar_2d` (run each long-running command in its own terminal):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/maze_2d/model/x500_mock_lidar_2d_0/link/link/sensor/mock_lidar_2d/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  --ros-args \
  -r /world/maze_2d/model/x500_mock_lidar_2d_0/link/link/sensor/mock_lidar_2d/scan:=/sim_lidar/scan

ros2 run dstar_lite gazebo_lidar_tf_broadcaster_node --ros-args \
  -p map_frame:=map \
  -p model_name:=x500_mock_lidar_2d_0 \
  -p sensor_frame:=mock_lidar_link \
  -p pose_topic:=/world/maze_2d/dynamic_pose/info \
  -p sensor_offset_x:=0.0 -p sensor_offset_y:=0.0 -p sensor_offset_z:=0.315

ros2 run dstar_lite laser_scan_octomap_node --ros-args \
  -p map_frame:=map -p scan_topic:=/sim_lidar/scan \
  -p slice_topic:=/voxel_slice -p use_latest_tf:=false \
  -p resolution:=0.10 -p world_width:=20.5 -p world_height:=17.0 \
  -p origin_x:=-6.5 -p origin_y:=-3.0
```

To register a LiDAR pose from PX4 vehicle odometry instead, configure
`px4_lidar_tf_broadcaster_node` with the LiDAR frame and physical offset. This
example is for the centred mock LiDAR:

```bash
ros2 run dstar_lite px4_lidar_tf_broadcaster_node --ros-args \
  -p map_frame:=odom -p sensor_frame:=mock_lidar_link \
  -p odometry_topic:=/fmu/out/vehicle_odometry \
  -p sensor_offset_x:=0.0 -p sensor_offset_y:=0.0 -p sensor_offset_z:=0.315
```

For the PX4-sectorized mapping path, run the `ObstacleDistance` mapper by
itself. Its default forward offset is `0.12 m`; override it to `0.0 m` for the
centred mock LiDAR shown here:

```bash
ros2 run dstar_lite obstacle_distance_octomap_node --ros-args \
  -p map_frame:=odom -p slice_topic:=/voxel_slice \
  -p resolution:=0.10 -p world_width:=20.5 -p world_height:=17.0 \
  -p origin_x:=-6.5 -p origin_y:=-3.0 \
  -p sensor_offset_x:=0.0 -p sensor_offset_y:=0.0 -p sensor_offset_z:=0.315
```

When using the offset `x500_lidar_2d` model, set `sensor_frame:=link` and
`sensor_offset_x:=0.12` in either TF broadcaster or mapper as applicable.

#### Source files

- `src/gazebo_lidar_tf_broadcaster.cpp` builds
  `gazebo_lidar_tf_broadcaster_node`. It subscribes directly to Gazebo
  Transport's `/world/maze_2d/dynamic_pose/info`, finds the selected model,
  and rotates the configured sensor offset by the model attitude. It publishes
  the resulting `map -> <sensor_frame>` transform and the same sensor pose on
  `/sim_lidar/pose`. Gazebo uses ENU world coordinates and FLU body axes, so
  this node needs no PX4 frame conversion. It stamps TF from the Gazebo pose
  message, letting the raw mapper look up the pose at the scan's simulation
  timestamp rather than using a newer pose.

- `src/laser_scan_octomap.cpp` builds `laser_scan_octomap_node`. It consumes a
  normal ROS `sensor_msgs/msg/LaserScan` (the launch gives it
  `/sim_lidar/scan`) and obtains `map -> scan_frame` from TF. Valid finite
  ranges become obstacle endpoints; positive infinite ranges clear space out to
  `range_max`; NaNs and invalid/out-of-range samples remain unknown. It inserts
  the observations into a persistent 3D `octomap::OcTree`, publishes the binary
  map on `/octomap_binary`, and publishes occupied/free cells on
  `/voxel_slice`. Every observed obstacle receives a 3x3-cell buffer in that
  2D projection. `/mapping_pose` and `/mapping_scan` provide the pose and
  world-frame obstacle points used by the current update for RViz/debugging.
  The default `use_latest_tf:=false` is important for the raw launch: it uses
  the scan timestamp instead of the latest available transform.

### PX4 `ObstacleDistance` alternative

The following source files support a PX4-centric mapping path. They are not
started by `px4_raw_lidar_octomap.launch.py` because that launch deliberately
uses the full-resolution Gazebo scan.

- `src/px4_lidar_tf_broadcaster.cpp` builds
  `px4_lidar_tf_broadcaster_node`. It subscribes to
  `/fmu/out/vehicle_odometry`, converts PX4 NED/FRD position and attitude to
  ROS ENU/FLU, applies the configured LiDAR offset, and publishes
  `odom` (or `map_frame`) to the LiDAR frame plus `/sim_lidar/pose`. Use it
  when the LiDAR pose must come from PX4 odometry rather than Gazebo's dynamic
  pose. Its defaults describe the centred mock LiDAR; set `sensor_frame` and
  the offset parameters to match the model being used.

- `src/obstacle_distance_octomap.cpp` builds
  `obstacle_distance_octomap_node`. It listens to
  `/fmu/out/vehicle_odometry` and `/fmu/out/obstacle_distance`. After
  converting odometry from NED/FRD to ENU/FLU, it interprets the
  `ObstacleDistance` sectors as BODY_FRD angles, converts them to FLU, and
  inserts their rays into the same persistent OctoMap and 2D projection format
  as `laser_scan_octomap_node`. An unknown sector is ignored; a sample equal to
  `max_distance + 1 cm` clears its ray; valid finite distances mark obstacles.
  The message's angular sectors are coarser than the raw scan, which is why
  this path is useful for PX4-interface testing but not the default Gazebo
  mapping validation path.

### Planner wall preference

`dstar_lite_node` keeps walls blocked but applies a finite proximity cost to
nearby free cells. It therefore uses a narrow passage when required while
preferring the centre when there is room. The defaults are a `0.50` m cost
radius and gain `4.0`; tune them without changing the launch:

```bash
ros2 run dstar_lite dstar_lite_node --ros-args \
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
- Output: `nav_msgs/msg/Path` on `/path`.
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

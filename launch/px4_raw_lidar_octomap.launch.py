"""Direct raw Gazebo lidar -> TF -> OctoMap mapping for X500 lidar models."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LIDAR_MODELS = {
    # Keep the copied mock model as the default so the existing command keeps
    # its exact behaviour.
    "x500_mock_lidar_2d": {
        "model_name": "x500_mock_lidar_2d_0",
        "sensor_frame": "mock_lidar_link",
        "sensor_offset": (0.0, 0.0, 0.315),
        "gz_scan": (
            "/world/maze_2d/model/x500_mock_lidar_2d_0/link/link/"
            "sensor/mock_lidar_2d/scan"
        ),
    },
    # Existing PX4 X500 lidar model.  lidar_2d_v2 publishes a LaserScan in
    # the frame named "link"; its Gazebo mount is x=0.12 m and the ray origin
    # is z=0.315 m from the vehicle model origin.
    "x500_lidar_2d": {
        "model_name": "x500_lidar_2d_0",
        "sensor_frame": "link",
        "sensor_offset": (0.12, 0.0, 0.315),
        "gz_scan": (
            "/world/maze_2d/model/x500_lidar_2d_0/link/link/"
            "sensor/lidar_2d_v2/scan"
        ),
    },
}


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    if lidar_model not in LIDAR_MODELS:
        valid_models = ", ".join(LIDAR_MODELS)
        raise RuntimeError(
            f"Unsupported lidar_model '{lidar_model}'. Choose one of: {valid_models}")

    lidar = LIDAR_MODELS[lidar_model]
    offset_x, offset_y, offset_z = lidar["sensor_offset"]
    gz_scan = lidar["gz_scan"]

    return [
        # This is the original, full-resolution Gazebo scan. It bypasses the
        # PX4 5-degree ObstacleDistance sector conversion entirely.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="raw_lidar_bridge",
            output="screen",
            arguments=[
                f"{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "--ros-args", "-r", f"{gz_scan}:=/sim_lidar/scan",
            ],
        ),
        Node(
            package="dstar_lite",
            executable="gazebo_lidar_tf_broadcaster_node",
            name="gazebo_lidar_tf_broadcaster_node",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "sensor_frame": lidar["sensor_frame"],
                "model_name": lidar["model_name"],
                "pose_topic": "/world/maze_2d/dynamic_pose/info",
                "sensor_offset_x": offset_x,
                "sensor_offset_y": offset_y,
                "sensor_offset_z": offset_z,
            }],
        ),
        Node(
            package="dstar_lite",
            executable="laser_scan_octomap_node",
            name="laser_scan_octomap_node",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "scan_topic": "/sim_lidar/scan",
                # The Gazebo pose TF and raw LaserScan have the same sim-time
                # stamp, so each scan is registered at its own pose.
                "use_latest_tf": False,
                "world_width": 20.5,
                "world_height": 17.0,
                "resolution": 0.10,
                "origin_x": -6.5,
                "origin_y": -3.0,
            }],
        ),
        # Keep planning in the exact Gazebo map frame too. /sim_lidar/pose is
        # the ground-truth pose published by gazebo_lidar_tf_broadcaster_node.
        Node(
            package="dstar_lite",
            executable="dstar_lite_node",
            name="dstar_lite_node",
            output="screen",
            parameters=[{
                "planning_frame": "map",
                "robot_pose_topic": "/sim_lidar/pose",
                "world_width": 20.5,
                "world_height": 17.0,
                "resolution": 0.10,
                "origin_x": -6.5,
                "origin_y": -3.0,
                "start_x": 0.0,
                "start_y": 0.0,
                "goal_x": 12.5,
                "goal_y": -2.75,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_model",
            default_value="x500_mock_lidar_2d",
            description=(
                "PX4 Gazebo model supplying the raw scan: "
                "x500_mock_lidar_2d or x500_lidar_2d"),
        ),
        OpaqueFunction(function=launch_setup),
    ])

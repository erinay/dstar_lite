"""TurtleBot3 Gazebo validation stack: scan mapping, D* Lite, and motion."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    gui = LaunchConfiguration("gui")
    gz_partition = LaunchConfiguration("gz_partition")

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    turtlebot_launch_dir = os.path.join(turtlebot_share, "launch")
    world_path = os.path.join(turtlebot_share, "worlds", "turtlebot3_world.world")

    # This mirrors turtlebot3_world.launch.py but keeps the GUI optional. The
    # server, robot spawner, bridge, planner, and controller all work headless.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": "-g -v2 ",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    turtlebot_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    common_planner_params = {
        "use_sim_time": use_sim_time,
        "world_width": 10.0,
        "world_height": 10.0,
        "resolution": 0.05,
        "origin_x": -5.0,
        "origin_y": -5.0,
        "goal_x": goal_x,
        "goal_y": goal_y,
        "inflation_radius": 0.20,
        "goal_topic": "/tb3/goal_pose",
        "belief_topic": "/tb3/belief_map",
        "path_topic": "/tb3/path",
        "waypoint_topic": "/tb3/waypoint",
    }

    planner = Node(
        package="dstar_lite",
        executable="turtlebot_dstar_lite_node",
        name="turtlebot_dstar_lite_node",
        output="screen",
        parameters=[common_planner_params],
    )

    controller = Node(
        package="dstar_lite",
        executable="turtlebot_waypoint_controller",
        name="turtlebot_waypoint_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_topic": "/tb3/goal_pose",
            "waypoint_topic": "/tb3/waypoint",
            "cmd_vel_topic": "/cmd_vel",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="burger"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("goal_x", default_value="2.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="Start the Gazebo GUI as well as the headless simulation server"),
        DeclareLaunchArgument(
            "gz_partition", default_value="dstar_lite_tb3",
            description="Gazebo transport partition, kept separate from PX4 Gazebo worlds"),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", model),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(turtlebot_share, "models")),
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        turtlebot_spawner,
        planner,
        controller,
    ])

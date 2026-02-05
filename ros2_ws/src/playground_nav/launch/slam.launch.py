import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    namespace = LaunchConfiguration("namespace", default="")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation time"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Robot namespace"
    )

    # Get package share directory
    pkg_share = FindPackageShare(package="playground_nav").find("playground_nav")
    slam_params_file = PathJoinSubstitution(
        [pkg_share, "launch", "config", "slam_params.yaml"]
    )

    # Pointcloud to laserscan node
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace=namespace,
        remappings=[
            ("cloud_in", "/pc"),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        output="screen",
    )

    # SLAM Toolbox async node
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)

    # Add nodes
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_toolbox_node)

    return ld

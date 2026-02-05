import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Paths to config files
    nav_params_file = PathJoinSubstitution(
        [pkg_share, "launch", "config", "navigation_params.yaml"]
    )
    slam_params_file = PathJoinSubstitution(
        [pkg_share, "launch", "config", "slam_params.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, "launch", "config", "config.rviz"]
    )

    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace,
        }.items(),
    )

    # Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(package="nav2_bringup").find("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": nav_params_file,
        }.items(),
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)

    # Add launches and nodes
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld

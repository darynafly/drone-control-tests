import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Generate a launch description for a UAV with ROS-Gazebo bridge."""
    
    # Path to the configuration file (ensure this path is correct)
    gz_ros_bridge_config = "/home/ws/drones_control/px4/simulation/camera/gz_ros_bridge.yaml"
    
    # Bridge Node: ros_gz_bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": gz_ros_bridge_config,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    # Relay Node (only if 'use_gz_tf' is set to True)
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/gz/tf", "/tf"],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    return LaunchDescription([
        # Declare launch argument for using Gazebo TF
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        
        # Add the bridge node
        bridge,

        # Register event handler to start relay when bridge starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=bridge,
                on_start=[topic_tools_tf]
            )
        ),
    ])

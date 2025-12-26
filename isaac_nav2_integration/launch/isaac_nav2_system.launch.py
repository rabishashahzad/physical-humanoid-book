from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Isaac Sim Bridge Node
        Node(
            package='isaac_sim_integration',
            executable='isaac_sim_bridge',
            name='isaac_sim_bridge',
            output='screen',
            parameters=[]
        ),

        # Isaac VSLAM Node
        Node(
            package='isaac_vslam_demos',
            executable='vslam_node',
            name='vslam_node',
            output='screen',
            parameters=[]
        ),

        # Isaac VSLAM Mapper
        Node(
            package='isaac_vslam_demos',
            executable='vslam_mapper',
            name='vslam_mapper',
            output='screen',
            parameters=[]
        ),

        # Isaac Path Planner
        Node(
            package='isaac_nav2_integration',
            executable='isaac_path_planner',
            name='isaac_path_planner',
            output='screen',
            parameters=[]
        ),

        # Isaac Nav2 Controller
        Node(
            package='isaac_nav2_integration',
            executable='isaac_nav2_controller',
            name='isaac_nav2_controller',
            output='screen',
            parameters=[]
        ),

        # Isaac Behavior Manager
        Node(
            package='isaac_nav2_integration',
            executable='isaac_behavior_manager',
            name='isaac_behavior_manager',
            output='screen',
            parameters=[]
        )
    ])
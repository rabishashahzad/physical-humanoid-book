from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Whisper speech recognition node
        Node(
            package='vla_speech_recognition',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
            parameters=[]
        ),

        # Audio capture node (simulated)
        Node(
            package='vla_speech_recognition',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen',
            parameters=[]
        ),

        # LLM planning node
        Node(
            package='vla_llm_integration',
            executable='llm_planning_node',
            name='llm_planning_node',
            output='screen',
            parameters=[]
        ),

        # Command parser node
        Node(
            package='vla_llm_integration',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen',
            parameters=[]
        )
    ])
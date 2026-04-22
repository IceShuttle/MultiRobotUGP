#!/usr/bin/env python3
"""
Launch file for the complete simulation:
- map_publisher: publishes the map from CSV
- entity_sim: simulates entities (humans, fires, robots)
- map_visualizer: visualizes the map and entities
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map Publisher Node
        Node(
            package='sim',
            executable='map_publisher',
            name='map_publisher',
            output='screen'
        ),
        
        # Entity Simulation Node
        Node(
            package='sim',
            executable='entity_sim',
            name='entity_sim',
            output='screen'
        ),
        
        # Map Visualizer Node
        Node(
            package='sim',
            executable='map_visualizer',
            name='map_visualizer',
            output='screen'
        ),
    ])
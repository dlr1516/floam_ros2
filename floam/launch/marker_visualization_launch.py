from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import ament_index_python.packages
import os
import yaml

def generate_launch_description():
    my_launch_description = LaunchDescription()

    loam_marker_visualization_node = Node(
        package='floam',
        executable='floam_marker_visualization_node',
        name='loam_marker_visualization_node',
        #prefix=['qterminal -e gdb -ex run --args'],
        #output='screen'
        # parameters=[{'nome_param1': 'valore_param1', 'nome_param2': 'valore_param2'}]
    )
    my_launch_description.add_action(loam_marker_visualization_node)
    
    return my_launch_description
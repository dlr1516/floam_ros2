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

    cloud_publisher_node = Node(
        package='cloud_file_publisher',
        executable='cloud_publisher',
        name='cloud_publisher',
        #prefix=['qterminal -e gdb -ex run --args'],
        output='screen',
        parameters=[{
          'cloud_topic': '/velodyne_points', 
          'cloud_file_glob': '/media/dario/Seagate\ Basic/Datasets/Kitti/data_odometry_velodyne/dataset/sequences/00/velodyne/*.bin',
          'frame_id':'laser_link',
          'pub_period_ms': 0.100
        }]
    )
    my_launch_description.add_action(cloud_publisher_node)
    
    return my_launch_description

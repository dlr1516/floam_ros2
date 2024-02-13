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

    # Cloud publisher from Kitti repo
    cloud_publisher_node = Node(
        package='cloud_file_publisher',
        executable='cloud_publisher',
        name='cloud_publisher',
        #prefix=['qterminal -e gdb -ex run --args'],
        output='screen',
        parameters=[{
          'cloud_topic': '/velodyne_points', 
          'cloud_file_glob': '/media/dario/Seagate\ Basic/Datasets/Kitti/data_odometry_velodyne/dataset/sequences/01/velodyne/*.bin',
          'frame_id': 'base_link',
          'pub_period_ms': 0.200
        }]
    )
    my_launch_description.add_action(cloud_publisher_node)
    
    # Feature computation node
    loam_feature_node = Node(
        package='floam',
        executable='floam_laser_processing_node',
        name='loam_feature_node',
    )
    my_launch_description.add_action(loam_feature_node)

    # Odometry estimation node
    loam_odometry_node = Node(
        package='floam',
        executable='floam_odom_estimation_node',
        name='loam_odometry_node',
    )
    my_launch_description.add_action(loam_odometry_node)

    # Mapping node
    loam_mapping_node = Node(
        package='floam',
        executable='floam_laser_mapping_node',
        name='loam_mapping_node',
    )
    my_launch_description.add_action(loam_mapping_node)

    # Trajectory visualization node
    loam_marker_visualization_node = Node(
        package='floam',
        executable='floam_marker_visualization_node',
        name='loam_marker_visualization_node',
    )
    my_launch_description.add_action(loam_marker_visualization_node)
    
    # Path saver
    path_save_node = Node(
        package='cloud_file_publisher',
        executable='path_save_node',
        name='path_save_node',
        #prefix=['qterminal -e gdb -ex run --args'],
        output='screen',
        parameters=[{
          'input_mode': 'odometry', 
          'odometry_topic': '/odom',
          'output_mode': 'mat',
          'output_filename': '/home/dario/colcon_ws/src/floam_ros2/cloud_file_publisher/data/path.txt'
        }]
    )
    my_launch_description.add_action(path_save_node)
    
    return my_launch_description

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    # Processing node - this is the entry node that gets data from sensors (or bag)
    loam_feature_node = Node(
        package='floam',
        executable='floam_laser_processing_node',
        name='loam_feature_node',
    )
    launch_description.add_action(loam_feature_node)

    # Odomotry estimation node
    loam_odometry_node = Node(
        package='floam',
        executable='floam_odom_estimation_node',
        name='loam_odometry_node',
    )
    launch_description.add_action(loam_odometry_node)

    # Mapping node
    loam_mapping_node = Node(
        package='floam',
        executable='floam_laser_mapping_node',
        name='loam_mapping_node',
    )
    launch_description.add_action(loam_mapping_node)

    # Trajectory visualization node
    loam_marker_visualization_node = Node(
        package='floam',
        executable='floam_marker_visualization_node',
        name='loam_marker_visualization_node',
    )
    launch_description.add_action(loam_marker_visualization_node)
    
    return launch_description
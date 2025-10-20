from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.2,  # Livox数据可能有延迟
                'queue_size': 10,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,   # 0.5度，适合Livox
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 20.0,
                'use_inf': True
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                ('scan', '/scan')
            ]
        )
    ])

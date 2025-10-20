import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_self = get_package_share_directory('dog_guide')

    map_file = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints', default=os.path.join(pkg_self, 'params', 'waypoints.yaml'))
    sounds_dir = os.path.join(pkg_self, 'sounds')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_self, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': map_file,
                              'params_file': os.path.join(pkg_self, 'params', 'nav2_params.yaml'),
                              'use_sim_time': use_sim_time}.items()),

        # 自定义 BT 节点加载器
        Node(package='dog_guide',
             executable='bt_creater',
             parameters=[{'waypoints_file': waypoints_file,
                          'sounds_dir': sounds_dir,
                          'use_sim_time': use_sim_time}],
             output='screen'),

        # TF发布器
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
        ),

        # mid360点云转换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_self, 'launch', 'mid360_to_laserscan.launch.py')
            ),
        ),
    ])

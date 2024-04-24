import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


"""
FIXME: change these two declaration of default map/path file name based on your actual setup
N.B.: Although we are using install path for demo purposes, a more convenient way
is to provide the absolute path starting with `~` or `/`. If you are using an install
path and not building with `--symlink-install`, you need to do a `colcon build`
and source `install/setup.bash` for the ros2 utilities to find the file.
"""
default_map_file = os.path.join(get_package_share_directory('rallycar'),
                                'resources', 'maps', 'crane_500_map.yaml')
default_path_file = os.path.join(get_package_share_directory('rallycar'),
                                 'resources', 'paths', 'knoy_speedway_demo_path.yaml')


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    map_file = LaunchConfiguration('map_file')
    path_file = LaunchConfiguration('path_file')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value = default_map_file,
        description = 'Full path of the yaml file to be loaded for a pgm map'
    )
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value = default_path_file,
        description = 'Full path of the yaml file to be saved as a desired trajectory, overwrites existing file'
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            { 'yaml_filename': map_file, },
        ]
        # topic: /map with qos=[keep_last, reliable, transient_local]
    )
    nav2_activation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    path_saver_node = Node(
        package='rallycar',
        executable='path_recorder.py',
        output='screen',
        parameters=[
            { 'path_file': path_file, },
        ]
    )
    rviz_interactive_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2',
        arguments=['-d' + os.path.join(
            get_package_share_directory('rallycar'),
            'resources', 'rviz_configs', 'build_path.rviz')
        ]
    )

    return LaunchDescription([
        map_file_arg,
        path_file_arg,
        map_server_node,
        nav2_activation_node,
        path_saver_node,
        rviz_interactive_node,
    ])

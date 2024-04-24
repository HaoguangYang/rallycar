import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


"""
FIXME: change this declaration of default map file name based on your actual setup,
or use the example at the end of this document to include this launch file into your
project.
N.B.: Although we are using install path for demo purposes, a more convenient way
is to provide the absolute path starting with `~` or `/`. If you are using an install
path and not building with `--symlink-install`, you need to do a `colcon build`
and source `install/setup.bash` for the ros2 utilities to find the file.
"""
default_map_yaml_file = os.path.join(
    get_package_share_directory('rallycar'),
    'resources', 'maps', 'crane_500_map.yaml'
)


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    map_file = LaunchConfiguration('map_file')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value = default_map_yaml_file,
        description = 'Full path of the yaml file to be loaded for a pgm map'
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file}
        ]
        # topic: /map with qos=[keep_last, reliable, transient_local]
    )


    return LaunchDescription([
        map_file_arg,
        map_server_node,
    ])


"""
This launch example can be included as:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rallycar'),
                'launch', 'examples', 'load_map.launch.py')
        ),
        launch_arguments={
            'map_file' : os.path.join(get_package_share_directory('YOUR_PACKAGE'),
                'resources', 'maps', 'YOUR_PATH_FILE.yaml'),
        }.items()
    )

Also, in your launch file, remember to add a nav2_lafecycle_manager node:
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                OTHER_NAV2_EXECUTABLES_YOU_STARTED
            ]
        }],
    )
"""

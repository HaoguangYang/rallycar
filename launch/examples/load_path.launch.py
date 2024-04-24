import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


"""
FIXME: change this declaration of default map file name based on your actual setup
or use the example at the end of this document to include this launch file into your
project.
N.B.: Although we are using install path for demo purposes, a more convenient way
is to provide the absolute path starting with `~` or `/`. If you are using an install
path and not building with `--symlink-install`, you need to do a `colcon build`
and source `install/setup.bash` for the ros2 utilities to find the file.
"""
default_path_file = os.path.join(
    get_package_share_directory('rallycar'),
    'resources', 'paths', 'crane_500_default_path.yaml'
)


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    path_file = LaunchConfiguration('path_file')
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value = default_path_file,
        description = 'Full path of the yaml file to be loaded as a desired trajectory'
    )
    path_server_node = Node(
        package='rallycar',
        executable='path_server.py',
        output='screen',
        parameters=[
            { 'file_name': path_file, },
        ]
    )

    return LaunchDescription([
        path_file_arg,
        path_server_node,
    ])


"""
This launch example can be included as:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rallycar'),
                'launch', 'examples', 'load_path.launch.py')
        ),
        launch_arguments={
            'path_file' : os.path.join(get_package_share_directory('YOUR_PACKAGE'),
                'resources', 'paths', 'YOUR_PATH_FILE.yaml'),
        }.items()
    )
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """

    # we can create nodes and put into the launch description as well
    scanmatching_slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_node',
        output='screen',
        namespace='',
        parameters=[
          get_share_file('rallycar', 'param', 'slam_toolbox_odomless.param.yaml'),
          {
            'use_lifecycle_manager': False,
            'use_sim_time': False
          }
        ],
        arguments=['--ros-args', '--log-level', 'slam_node:=warn'],
    )

    # include other launch files
    scanmatching_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file(
                'rallycar', 'launch', 'include', 'scanmating_odom_tf_publisher.launch.py'
            )
        )
    )

    return LaunchDescription([
        scanmatching_slam_node,
        scanmatching_odom_launch,
    ])

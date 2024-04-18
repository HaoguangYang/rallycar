import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
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

    odom_spoofer = Node(
        package='rallycar',
        executable='pose_tf_publisher.py',
        name='odom_spoofer_node',
        output='screen',
        parameters=[
            {
                'init_source_frame_name': 'scanmatcher_frame',
                'target_frame_name': 'base_link',
                'init_tf_pose': [0., 0., 0., 0., 0., 0.],
                # stuck at an identity transform without further updating
                'updater_topic': '',
                # broadcast tf at 40Hz
                'min_tf_broadcast_frequency': 40.0,
            },
        ],
    )

    return LaunchDescription([
        scanmatching_slam_node,
        odom_spoofer,
    ])

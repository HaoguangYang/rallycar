import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    hokuyo_node = Node(
        package='urg_node',
        name='hokuyo_lidar_driver_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rallycar'), 'param', 'lidar.param.yaml'),
        ],
    )
    return LaunchDescription([
        hokuyo_node,
    ])

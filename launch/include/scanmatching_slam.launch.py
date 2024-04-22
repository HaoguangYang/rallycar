import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


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
          os.path.join(get_package_share_directory('rallycar'),
                       'param', 'slam_toolbox_scanmatching.param.yaml'),
          {
            # these parameters adds to the list of parameters included by the file above.
            # since this block is defined later, if some parameter have duplicate names,
            # the param defined below will overwrite whatever included from the file above.
            'map_frame': 'map',
            'base_frame': 'base_link'
            'odom_frame': 'odom'
            'mode': 'mapping'                   # 'mapping' or 'localization'
            'transform_publish_period': 0.025   # if 0 never publishes map->odom
            'use_sim_time': False
          }
        ],
        # suppress console output -- only when level "warning" and above
        arguments=['--ros-args', '--log-level', 'slam_node:=warn'],
    )

    scanmatching_odom_node = Node(
        package='rallycar',
        executable='scanmatcher_odom_repub.py',
        name='scanmatcher_odom_repub_node',
        output='screen',
        parameters=[
            {
                'scanmatcher_pose_topic': '/pose',
                'odom_topic': '/odom',
                'odom_header_frame_id': 'odom',
                'odom_child_frame_id': 'base_link',
            }
        ],
    )

    odom_tf_node = Node(
        package='rallycar',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher_node',
        output='screen',
        parameters=[
            {
                'init_source_frame_name': 'odom',
                'target_frame_name': 'base_link',
                'init_tf_pose': [0., 0., 0., 0., 0., 0.],
                'updater_topic': '/odom',
                # broadcast tf at >= 40Hz
                'min_tf_broadcast_frequency': 40.0,
            },
        ],
    )

    return LaunchDescription([
        scanmatching_slam_node,
        scanmatching_odom_node,
        odom_tf_node,
    ])
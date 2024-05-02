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
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_node",
        output="screen",
        namespace="",
        parameters=[
            os.path.join(
                get_package_share_directory("rallycar"),
                "param",
                "slam_toolbox_scanmatching.param.yaml",
            ),
            {
                # these parameters adds to the list of parameters included by the file above.
                # since this block is defined later, if some parameter have duplicate names,
                # the param defined below will overwrite whatever included from the file above.
                "map_frame": "map",
                "base_frame": "base_link",
                "odom_frame": "map",
                # as "odom_frame" and "map_frame" are set the same, we don't need the tf below
                # instead, we will start another node to publish "map->base_link" for visualization
                "transform_publish_period": 0.0,  # if 0 never publishes map_frame->odom_frame
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("pose", "/scanmatching_odom/pose"),
        ],
        # suppress console output -- only when level "warning" and above
        arguments=["--ros-args", "--log-level", "slam_node:=warn"],
    )

    # This node is for visualization (providing the map->base_link transform):
    odom_tf_node = Node(
        package="rallycar",
        executable="odom_tf_publisher.py",
        name="odom_tf_publisher_node",
        output="screen",
        parameters=[
            {
                "init_source_frame_name": "map",
                "target_frame_name": "base_link",
                "init_tf_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "updater_topic": "/scanmatching_odom/pose",  # published by slam_toolbox
                "min_tf_broadcast_frequency": 40.0,  # broadcast tf at >= 40Hz
            },
        ],
    )

    return LaunchDescription([scanmatching_slam_node, odom_tf_node])

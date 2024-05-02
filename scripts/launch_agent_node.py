#!/usr/bin/env python3
"""
File: launch_agent_node.py
Author: Haoguang Yang (yang1510@purdue.edu)
Date: 2024-05-01

Description:
This script act as a ROS2 launch agent -- it is a ROS2 node that listens to a
topic for instructions to launch other ROS2 nodes.
This script is usually used for demo purposes, e.g. to be used with the RViz2
"rviz2_ros2launch" plugin to start a launch file within RViz2.

This ROS2 node subscribes to:
- start_launch_file (std_msgs/String): full path of launch file to be started
- shutdown_launch_file (std_msgs/String): full path of launch file to be
    stopped
"""

import multiprocessing
import nest_asyncio
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String

from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def meta_launcher(
    launch_desc: LaunchDescription = None, launch_desc_list: list = [], ros_domain_id=-1
) -> "tuple[multiprocessing.Process]":
    if ros_domain_id >= 0:
        # override domain ID if it is set
        if ros_domain_id <= 101:
            os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
        else:
            print(
                "WARNING: ROS_DOMAIN_ID > 101. For compatibility reasons it is NOT set."
            )
    lsPool = [LaunchService(noninteractive=True) for i in launch_desc_list]
    i = 0
    for ls in lsPool:
        ls.include_launch_description(launch_desc_list[i])
        i += 1
    if launch_desc:
        lsPool.append(LaunchService(noninteractive=True))
        lsPool[-1].include_launch_description(launch_desc)
    nodeProcs = [multiprocessing.Process(target=ls.run) for ls in lsPool]
    for p in nodeProcs:
        p.start()
    return tuple(nodeProcs)


def meta_launch_shutdown(procs: "tuple[multiprocessing.Process]"):
    for item in procs:
        # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
        os.kill(item.pid, signal.SIGINT)
        # item.terminate()
    for item in procs:
        item.join(1.0)


class Ros2LaunchListener(Node):
    def __init__(self):
        super().__init__("ros2_launch_agent_node")
        self.create_subscription(
            String,
            "start_launch_file",
            self.start_launch_file_cb,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.create_subscription(
            String,
            "shutdown_launch_file",
            self.shutdown_launch_file_cb,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.launched_configs = {}

    def start_launch_file_cb(self, msg):
        # this function essentially calls self.launch_child with reduced API
        config = self.launch_child(msg.data)
        if config:
            self.launched_configs.update({msg.data: config})

    def shutdown_launch_file_cb(self, msg):
        if msg.data not in self.launched_configs:
            self.get_logger().error("Unable to shutdown %s: Not started." % (msg.data,))
            return
        meta_launch_shutdown(self.launched_configs.pop(msg.data))

    # TODO: "launch_child" uses full API. We need a customized message to handle all fields.
    def launch_child(self, launch_file: str, launch_args: dict = {}, ros_domain_id=-1):
        launch_desc = LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments=launch_args.items(),
                )
            ]
        )
        config = None
        try:
            config = meta_launcher(launch_desc, ros_domain_id=ros_domain_id)
        except Exception as e:
            self.get_logger().error(
                "Failed to start launch file: %s\n\tTraceback: %s"
                % (
                    launch_file,
                    str(e),
                )
            )
        return config


def main():
    # apply nest_asyncio because we are nesting parallel processes
    nest_asyncio.apply()
    rclpy.init()
    node = Ros2LaunchListener()
    try:
        # this line guards the process until Ctrl-C is pressed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()

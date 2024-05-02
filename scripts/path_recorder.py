#!/usr/bin/env python3
"""
File: path_recorder.py
Author: Haoguang Yang (yang1510@purdue.edu)
Date: 2024-04-29

Description:
This script implements a ROS2 node that listens to the "/goal_pose" topic for
poses, then concatenates the poses in reception sequence to form a path. In
addition, the node listens to the "/cancel" topic, and removes the last-
received pose from the resultant path when a message comes in. The node saves
the path at a user-specified location upon exiting.

This script is meant to work with RViz2, which publishes the user selection
with "2D Goal Pose" tool to the "/goal_pose" topic. The pose removal portion
works with the "rallycar/RemoveLastNavGoal" tool
(src/remove_last_nav_goal_tool.cpp) that ships with this package. The saved
file is in yaml format, recording position (x, y, z) and orientation
(quaternion x, y, z, w) of each point in the path sequentially.

Additionally, the node also publishes the live resultant path to
"/desired_path" topic. This topic can be visualized in RViz2 to close the user
interaction loop.

User-specified parameter:
- path_file: full path of the yaml file where the user wants to save the
    resultant trajectory in.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSPresetProfiles,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID


def list_of_pose_dict_to_path_msg(inp):
    path = Path()
    path.header.frame_id = "map"
    path.poses = []
    for item in inp:
        p = PoseStamped()
        p.header.frame_id = "map"
        pos = p.pose.position
        pos.x, pos.y, pos.z = (
            item["position"]["x"],
            item["position"]["y"],
            item["position"]["z"],
        )
        ori = p.pose.orientation
        ori.x, ori.y, ori.z, ori.w = (
            item["orientation"]["x"],
            item["orientation"]["y"],
            item["orientation"]["z"],
            item["orientation"]["w"],
        )
        path.poses.append(p)
    return path


class PathRecorder(Node):
    """The Path Recorder node.
    It subscribes to `/goal_pose` and records every received message as
    sequential poses.
    A subscriber is set up at `/cancel` to delete the last pose when requested.
    Upon exit, it saves the path to the filename specified as a yaml file.
    """

    def __init__(self):
        """Class constructor that subscribes to the topic `/goal_pose` and
        saves the path. The `/goal_pose` topic is published by Rviz that
        records user click on the UI.
        """
        super().__init__("path_recorder_node")
        self.declare_parameter("path_file", "")
        self.path_file = self.get_parameter("path_file").value
        if not len(self.path_file):
            raise Exception("invalid filename")
        self.path = []
        self.path_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.clicked_pose_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        path_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, "/desired_path", path_pub_qos)
        self.path_rm_last_sub = self.create_subscription(
            GoalID,
            "/cancel",
            self.del_last_pose_callback,
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

    def clicked_pose_callback(self, msg):
        """Callback function that records the pose when the user clicks on
        RViz2.

        Args:
            msg (PoseStamped): Message published by Rviz
        """
        self.path.append(
            {
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            }
        )
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def del_last_pose_callback(self, msg):
        """Callback function that deletes the last entered point from the
        record.

        Args:
            msg (actionlib_msgs/GoalID): Empty indicator, not used.
        """
        if len(self.path):
            self.path.pop()
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def save_path(self):
        """Function to be executed during node shutdown. It saves the path to
        disk.
        """
        if len(self.path) == 0:
            return
        with open(self.path_file, "w+") as f:
            yaml.dump(self.path, f)
            self.get_logger().info("Trajectory saved to %s" % (self.path_file,))


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_path()
    node.destroy_node()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
File: odom_tf_publisher.py
Author: Haoguang Yang (yang1510@purdue.edu)
Date: 2024-05-01

Description:
This script implements a ROS2 node that subscribes to a pose-related topic, and
converts the pose information into tf format. The converted tf is then sent out
with a transform broadcaster into the tf tree.

If the specified topic is empty, does not exist, or is not updating, then the
resultant tf from this node will also have a fixed pose value. However, the
timestamp of the resultant tf will be updated to the current time.

The user may specify the following parameters:
- init_source_frame_name: initial value of the "header.frame_id" field of the
    resultant tf. This field will be overrided by the "header.frame_id" of
    incoming messages. Defaults to "odom"
- target_frame_name: initial value of the "child_frame_id" field of the
    resultant tf. This field will be overrided by the "child_frame_id" of
    incoming messages if the message is of "nav_msgs/Odometry" type. Defaults to
    "base_link".
- init_tf_pose: initial value of the pose fields of the resultant tf. The list
    is in "x, y, z, roll, pitch, yaw" order, from "header.frame_id" to
    "child_frame_id". Defaults to [0, 0, 0, 0, 0, 0].
- updater_topic: topic to listen for latest pose updates. If empty, then the
    initial values specified by parameters above will never be updated. The
    result is equivalent to a static transform. Defaults to an empty string ''.
    Note: "updater_topic" should be pointed to a topic with message type being:
    geometry_msgs/PoseWithCovarianceStamped or nav_msgs/Odometry.
- min_tf_broadcast_frequency: The resultant tf will be sent out at this
    frequency, despite the updater_topic is not updating as often. The resultant
    tf will have a refreshed "header.stamp" field, reflecting the current time.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np

# for creating a subscription with automatically-determined message type
# ref: https://answers.ros.org/question/359681/generic-subscriber-in-ros2/
from ros2topic.api import get_msg_class


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler angles -- roll (x), pitch (y), yaw (z), angles are in
    radians, into a Quaternion [qx, qy, qz, qw], returns a list of the four
    numbers.
    """
    ehalf = np.array([roll, pitch, yaw]) * 0.5
    ci, cj, ck = np.cos(ehalf)
    si, sj, sk = np.sin(ehalf)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk
    # return q in [x, y, z, w] order
    return [cj * sc - sj * cs, cj * ss + sj * cc, cj * cs - sj * sc, cj * cc + sj * ss]


class OdomTfPublisher(Node):
    """
    Converts an updating Odometry message into a tf pointing to a specified
    baselink frame. A typical use case of this node is a helper to bridge a
    missing tf from odom to base_link. To achieve this functionality, the user
    should set the "updater_topic" parameter to the topic with a publisher of
    Odometry message. On the publisher side, the user should set the
    header.frame_id field of the Odometry message as "odom", and the pose
    field of the message being the integrated robot pose since it started
    moving.

    This node Subscribes to a specified topic with message type Odometry, and
    extracts the header, pose.position, and pose.orientation fields. This node
    then packs these fields into a TransformStamped message type and publishes
    to /tf.

    The user also needs to specify the initial values of the transform, if not
    identity.

    A guard of minium update frequency is set, such that when the update topic
    is not specified or not updating, old values are packed with a current
    timestamp and re-published at that guard frequency.
    """

    def __init__(self):
        super().__init__("odom_tf_publisher_node")

        # Declare and acquire `odom_frame_name` parameter
        self.declare_parameter("init_source_frame_name", "odom")
        self.declare_parameter("target_frame_name", "base_link")
        self.declare_parameter("init_tf_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("updater_topic", "")
        self.declare_parameter("min_tf_broadcast_frequency", 10.0)

        # build the initial value
        self.tf = TransformStamped()
        self.tf.header.frame_id = self.get_parameter("init_source_frame_name").value
        self.tf.child_frame_id = self.get_parameter("target_frame_name").value
        tf_init = self.get_parameter("init_tf_pose").value
        # unpack the first three values into translation
        t = self.tf.transform.translation
        t.x, t.y, t.z = tf_init[0:3]
        # unpack the last three values into rotation (convert to quaternion)
        q = self.tf.transform.rotation
        q.x, q.y, q.z, q.w = quaternion_from_euler(tf_init[3], tf_init[4], tf_init[5])

        # User may specify a topic where this node receives updates.
        # if this parameter is non-empty, a subscriber of types:
        # 'nav_msgs/Odometry' or 'geometry_msgs/PoseWithCovarianceStamped'
        # is created at that topic, based on existing type of the publisher.
        # This node directly picks the header, pose.position, and pose.orientation,
        # fields, and updates the published transform.
        self.sub_topic = self.get_parameter("updater_topic").value
        self.subscription = None

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # set a timer callback to guard the minimum update frequency.
        tf_freq = self.get_parameter("min_tf_broadcast_frequency").value
        self.send_tf_timer = self.create_timer(1.0 / tf_freq, self.send_tf_callback)

    def send_tf_callback(self):
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf)
        # check if updater (message subscriber) needs to be set up
        if not self.subscription and self.sub_topic:
            # automatically determine the message type of the topic
            message_type = None
            try:
                message_type = get_msg_class(
                    self, self.sub_topic, include_hidden_topics=True
                )
            except Exception as e:
                pass
            if not message_type:
                # log an error and return
                self.get_logger().error(
                    "Automatic determination of message type "
                    'failed on topic: "%s", retrying...' % (self.sub_topic,)
                )
                return
            # set up a subscriber on the topic with best effort QoS, if type is determined
            self.subscription = self.create_subscription(
                message_type,
                self.sub_topic,
                self.handle_tf_update,
                rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
            )
            # for logging
            msg_type_str_arr = str(message_type).split("'")[1].split(".")
            self.get_logger().info(
                'Subscribed to "%s" with automatically determined type: '
                '"%s/%s"' % (self.sub_topic, msg_type_str_arr[0], msg_type_str_arr[-1])
            )
            # once subscriber is up, this "if" block is never called again.

    def handle_tf_update(self, msg):
        # The callback function works with messages of types:
        # - nav_msgs/Odometry, or
        # - geometry_msgs/PoseWithCovarianceStamped
        self.tf.header = msg.header
        if hasattr(msg, "child_frame_id"):
            # this field only exists when subscribed to "nav_msgs/Odometry"
            self.tf.child_frame_id = msg.child_frame_id

        # translate the field names
        pos = msg.pose.pose.position
        t = self.tf.transform.translation
        t.x, t.y, t.z = (pos.x, pos.y, pos.z)
        self.tf.transform.rotation = msg.pose.pose.orientation

        # we just received an update and have published, therefore we postpone
        # the next time-driven update by resetting the timer.
        self.send_tf_timer.reset()
        # sending the translated tf
        self.tf_broadcaster.sendTransform(self.tf)


def main():
    rclpy.init()
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()

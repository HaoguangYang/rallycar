#!/usr/bin/env python3

# modified from teleop_twist_keyboard package
# (https://github.com/ros2/teleop_twist_keyboard/blob/dashing/teleop_twist_keyboard.py)
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# adaptation made by Haoguang Yang

# This script reads keyboard at 10Hz, and publishes Float32 messages at
# /steering_cmd and /accelerator_cmd. Follow instruction prompt on the screen.

import sys
import select
import threading

from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class KeyboardTwistTeleop(Node):
    def __init__(self, name = "rally_teleop_keyboard_node"):
        super().__init__(name)
        self.settings = termios.tcgetattr(sys.stdin)
        self.msg = """
        This node takes keypresses from the keyboard and publishes them to
        /accelerator_cmd and /steering_cmd (std_msgs/msg/Float32 type).
        It works best with a US keyboard layout.
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        anything else : stop

        q/z : increase/decrease both liner and angular speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
        """

        self.moveBindings = {
            'i': (1, 0),
            'o': (1, -1),
            'u': (1, 1),
            'l': (0, -1),
            'j': (0, 1),
            ',': (-1, 0),
            '.': (-1, -1),
            'm': (-1, 1),
        }

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }

        self.declare_parameter("accelerator_pedal_cmd_topic", "accelerator_cmd")
        gasCmdTopic = self.get_parameter("accelerator_pedal_cmd_topic").value
        self.declare_parameter("steering_cmd_topic", "steering_cmd")
        steerCmdTopic = self.get_parameter("steering_cmd_topic").value
        self.accPub = self.create_publisher(Float32, gasCmdTopic,
                                            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.steerPub = self.create_publisher(Float32, steerCmdTopic,
                                              rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        # initial values
        self.declare_parameter("init_pedal_cmd_mag", 0.5)
        self.speed = self.get_parameter("init_pedal_cmd_mag").value
        self.declare_parameter("init_steer_cmd_mag", 1.0)
        self.turn = self.get_parameter("init_steer_cmd_mag").value
        # termios timeout defines max spinning rate
        self.declare_parameter("update_frequency", 10.0)
        self.keyboard_timeout = 1./self.get_parameter("update_frequency").value
        self.gasCmd = Float32()
        self.steerCmd = Float32()

    def getKey(self):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # select.select listens on the keyboard input with timeout of 0.1s.
            res, _, _ = select.select([sys.stdin], [], [], self.keyboard_timeout)
            # sys.stdin.read() returns a string on Linux
            key = '' if not res else sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self, speed, turn):
        return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


    def run(self):
        x = 0
        th = 0
        status = 0
        try:
            if rclpy.ok():
                print(self.msg)
                print(self.vels(self.speed, self.turn))
            while rclpy.ok():
                key = self.getKey()  # blocks here
                if key in self.moveBindings.keys():
                    x = self.moveBindings[key][0]
                    th = self.moveBindings[key][1]
                elif key in self.speedBindings.keys():
                    self.speed *= self.speedBindings[key][0]
                    self.turn *= self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15
                elif not key:
                    pass
                else:
                    x = 0
                    th = 0
                    if (key == '\x03'):
                        break
                self.gasCmd.data = x * self.speed
                self.steerCmd.data = th * self.turn
                self.accPub.publish(self.gasCmd)
                self.steerPub.publish(self.steerCmd)
        except Exception as e:
            print(e)

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.gasCmd.data = 0.
        self.steerCmd.data = 0.
        self.accPub.publish(self.gasCmd)
        self.steerPub.publish(self.steerCmd)


if __name__ == "__main__":
    rclpy.init()
    node = KeyboardTwistTeleop()
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()
    node.run()
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    spinner.join()

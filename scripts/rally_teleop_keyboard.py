#!/usr/bin/env python3

# modified from teleop_twist_keyboard package (http://wiki.ros.org/teleop_twist_keyboard)
# author: Austin Hendrix (namniart@gmail.com)
# adaptation made by Haoguang Yang

# This script reads keyboard at 10Hz, and publishes Float32 messages at
# /steering_cmd and /accelerator_cmd. Follow instruction prompt on the screen.

from __future__ import print_function

import rospy

from std_msgs.msg import Float32

import sys
import select
import termios
import tty


class keyboardTwistTeleop:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.msg = """
        Reading from the keyboard  and Publishing to Twist!
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        anything else : stop

        q/z : increase/decrease both liner and angular speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        p/P : stop/start the base motion

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

        gasCmdTopic = rospy.get_param('accelerator_pedal_cmd_topic', 'accelerator_cmd')
        steerCmdTopic = rospy.get_param('steering_cmd_topic', 'steering_cmd')
        self.accPub = rospy.Publisher(gasCmdTopic, Float32, queue_size=1)
        self.steerPub = rospy.Publisher(steerCmdTopic, Float32, queue_size=1)
        # initial values
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        rospy.on_shutdown(self.shutdown)
        # hold there until the subsecribers are ready
        r = rospy.Rate(10)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # select.select listens on the keyboard input with timeout of 0.1s.
        res, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = '' if not res else sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def run(self):
        x = 0
        th = 0
        status = 0
        try:
            if not rospy.is_shutdown():
                print(self.msg)
                print(self.vels(self.speed, self.turn))
            while not rospy.is_shutdown():
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
                gasCmd = Float32(data=x * self.speed)
                steerCmd = Float32(data=th * self.turn)
                self.accPub.publish(gasCmd)
                self.steerPub.publish(steerCmd)
        except Exception as e:
            print(e)

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.accPub.publish(Float32(data=0.))
        self.steerPub.publish(Float32(data=0.))


if __name__ == "__main__":
    rospy.init_node('teleop_twist_keyboard')
    kb = keyboardTwistTeleop()
    kb.run()

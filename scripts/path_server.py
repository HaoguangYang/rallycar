#!/usr/bin/env python3
import rospy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

"""
The path server node. It works in either publish mode (default) or record mode.
In publish mode, it loads a path from `resources/paths` file, and publishes it
at `/desired_path` topic with `latch=True`. The `latch` option allows any future
subscribers to receive the message. The path will be published only once for a
static path.
In record mode, it subscribes to `/move_base_simple/goal` and records every
received message as sequential poses. A subscriber is set up at `/move_base/cancel`
to delete the last pose when requested. Upon exit, it saves the path to the
filename specified as a yaml file.
"""

def list_of_pose_dict_to_path_msg(inp):
    path = Path(header=Header(stamp=rospy.Time.now(),
                            frame_id="map"))
    path.poses = []
    for item in inp:
        path.poses.append(PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(position=Point(
                item['position']['x'], item['position']['y'], item['position']['z']
            ),
                orientation=Quaternion(
                    item['orientation']['x'], item['orientation']['y'],
                    item['orientation']['z'], item['orientation']['w']
            ))
        ))
    return path


class PathServer:
    """The Path Server mode, invoked when running in publish mode.
    """

    def __init__(self, file_name):
        """Class constructor that loads the path from file and publishes it.

        Args:
            file_name (str): Input path file name
        """
        with open(file_name, 'r') as f:
            inp = yaml.safe_load(f)
        self.path = list_of_pose_dict_to_path_msg(inp)
        print(self.path)
        self.path_pub = rospy.Publisher("/desired_path", Path, latch=True, queue_size=1)
        self.path_pub.publish(self.path)


class PathRecorder:
    """The Path Recorder mode, invoked when running in record mode.
    """

    def __init__(self, file_name):
        """Class constructor that subscribes to the topic `/move_base_simple/goal` and saves the path.
        The `/move_base_simple/goal` topic is published by Rviz, and records user click on the UI.

        Args:
            file_name (str): path file name to save with.
        """
        self.path_file = file_name
        rospy.on_shutdown(self.save_path)
        self.path = []
        self.path_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                                         self.clicked_pose_callback)
        self.path_pub = rospy.Publisher("/desired_path", Path, queue_size=1)
        self.path_rm_last_sub = rospy.Subscriber("/move_base/cancel", GoalID,
                                                 self.del_last_pose_callback)

    def clicked_pose_callback(self, msg):
        """Callback function that records the pose when the user clicks on the UI.

        Args:
            msg (PoseStamped): Message published by Rviz
        """
        self.path.append({
            'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
            'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                            'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
        })
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def del_last_pose_callback(self, msg):
        """Callback function that deletes the last entered point from the record.

        Args:
            msg (actionlib_msgs/GoalID): Empty indicator
        """
        if len(self.path):
            self.path.pop()
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def save_path(self):
        """Function to be executed during node shutdown. It saves the path to disk.
        """
        if len(self.path) == 0:
            return
        with open(self.path_file, 'w+') as f:
            yaml.dump(self.path, f)


def main():
    rospy.init_node('path_server')
    mode = rospy.get_param('~mode', 'publish')
    file_name = rospy.get_param('~file_name')
    if mode != 'record':
        ps = PathServer(file_name)
    else:
        ps = PathRecorder(file_name)
    rospy.spin()


if __name__ == '__main__':
    main()

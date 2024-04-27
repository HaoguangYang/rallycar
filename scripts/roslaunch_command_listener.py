import roslaunch
import rospy
from std_msgs.msg import String

"""
Wrapper for launching threads and ROS components within Python
"""
class Launcher:
    def __init__(self, launcherName:str, auto_start=True):
        self.roscoreProc = None
        self.nodeLauncher = None
        self.name = launcherName
        self.launch_sub = None
        self.launched_proc = {}
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.roscoreProc = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_files=[], is_core=True)
        self.nodeLauncher = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_files=[], is_core=False)
        if auto_start:
            self.activate()

    def activate(self):
        # start roscore
        self.roscoreProc.start()
        self.roscoreProc.spin_once()
        # start interface to accept roslaunch files
        self.nodeLauncher.start(auto_terminate=False)
        self.spin_once()
        rospy.init_node(self.name)
        self.spin_once()
        rospy.sleep(0.05)
        self.launch_sub = rospy.Subscriber(
            "rviz_run_launch_file", String, self.__launch_cb)
        self.shutdown_sub = rospy.Subscriber(
            "rviz_cancel_launch_file", String, self.__launch_shutdown_cb)
        self.spin_once()
        rospy.sleep(0.05)

    def __launch_cb(self, msg):
        try:
            self.launched_proc.update({
                msg.data : self.launch(fullPathList=[msg.data])
            })
        except Exception as e:
            print(e.what())

    def __launch_shutdown_cb(self, msg):
        if msg.data not in self.launched_proc:
            rospy.logerr("Launch record for file: %s not found", msg.data)
            return;
        try:
            self.stop(self.launched_proc[msg.data])
        except Exception as e:
            print(e.what())

    def launch(self, pkgName:str='', fileName:str='', args:tuple=(), fullPathList:list=[]):
        if (pkgName and fileName):
            fp = roslaunch.rlutil.resolve_launch_arguments([pkgName, fileName, *args])
            if args:
                fp = [(fp[0], list(args))]
        else:
            fp = []
        for line in fullPathList:
            # fullPathList APPENDS description from pkgName/fileName.
            if len(line)>1:
                fp.append((roslaunch.rlutil.resolve_launch_arguments(line)[0], line[1:]))
            else:
                fp.append(roslaunch.rlutil.resolve_launch_arguments(line)[0])
        # wait until roscore is available to handle the state transition.
        roslaunch.rlutil.get_or_generate_uuid(None, True)
        cfg = roslaunch.config.load_config_default(fp, None, verbose=False)
        self.nodeLauncher.runner.config.params.update(cfg.params)
        # hack to update parameter server...
        self.nodeLauncher.runner._load_parameters()
        nodeProcs = []
        # only launches local nodes.
        local_nodes = [n for n in cfg.nodes if roslaunch.core.is_machine_local(n.machine)]
        for node in local_nodes:
            self.nodeLauncher.spin_once()
            self.roscoreProc.spin_once()
            proc, success = self.nodeLauncher.runner.launch_node(node)
            if success:
                nodeProcs.append(proc)
            self.nodeLauncher.spin_once()
            self.roscoreProc.spin_once()
        return tuple(nodeProcs)

    def stop(self, proc):
        for p in proc:
            p.shutdown()

    def status(self):
        # returns status of the launcher
        coreStat = Fstat.NULL
        nodeStat = Fstat.NULL
        if hasattr(self.roscoreProc, 'runner'):
            if not (self.roscoreProc.runner is None):
                if not self.roscoreProc.pm.is_shutdown:
                    coreStat = Fstat.RUNNING
                else:
                    coreStat = Fstat.OFF
        if hasattr(self.nodeLauncher, 'runner'):
            if not (self.nodeLauncher.runner is None):
                if not self.nodeLauncher.pm.is_shutdown:
                    nodeStat = Fstat.RUNNING
                else:
                    nodeStat = Fstat.OFF
        return {'rosCore': coreStat, 'nodeLauncher': nodeStat}

    def spin(self):
        rate = rospy.Rate(20.0)
        while (not self.roscoreProc.pm.is_shutdown) or (not self.nodeLauncher.pm.is_shutdown):
            self.roscoreProc.spin_once()
            self.nodeLauncher.spin_once()
            rate.sleep()
        rospy.signal_shutdown("ROS Launcher Exiting...")

    def spin_once(self):
        self.nodeLauncher.spin_once()
        self.roscoreProc.spin_once()

    def shutdown(self):
        self.nodeLauncher.shutdown()
        self.roscoreProc.shutdown()

if __name__ == "__main__":
    # a simple test case with two empty modules.
    launcher = Launcher("RosNodeRunner")
    launcher.spin()
    # Ctrl-C triggers exit
    launcher.shutdown()

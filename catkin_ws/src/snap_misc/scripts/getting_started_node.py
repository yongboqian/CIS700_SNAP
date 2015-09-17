#!/usr/bin/python
import rospy
#import roslaunch
import subprocess

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from snap_misc.cfg import GettingStartedConfig

class ManagedLaunch(object):
    def __init__(self, name, package, fname, args=[]):
        self.name = name
        self.package = package
        self.fname = fname
        self.args = args
        self.process = None

    @staticmethod
    def factory(s):
        args = s.split(" ")
        name = args.pop(0)
        package = args.pop(0)
        fname = args.pop(0)
        #rospy.loginfo("factory(%s) -> %s,%s,%s,%s"%(s, name, package, fname, str(args)))
        return ManagedLaunch(name, package, fname, args)

    def is_alive(self):
        if self.process is None:
            return False
        if self.process.poll() is None:
            return True
        return False

    def died(self):
        if self.process is None:
            # wasn't alive
            rospy.logdebug("%s wasn't alive"%self.name)
            return False
        elif self.process.poll() is not None:
            # process finished
            rospy.logdebug("%s finished"%self.name)
            return True
        else:
            # process hasn't finished yet
            rospy.logdebug("%s still going"%self.name)
            return False

    def start(self, args=[]):
        if self.is_alive(): return

        #assert self.process is None

        rospy.loginfo("Launching %s: '%s %s %s'" % (self.name, self.package, self.fname, " ".join(self.args+args)))
        self.process = subprocess.Popen(['roslaunch', self.package, self.fname]+self.args+args,
                stdin=None, stdout=None, stderr=None) #, shell=True)
        rospy.loginfo("%s PID: %i" % (self.name, self.process.pid))

    def stop(self):
        if not self.is_alive(): return

        assert self.process is not None
        rospy.loginfo("Terminating %s (%i)..." % (self.name, self.process.pid))
        self.process.terminate()
        rospy.loginfo("Killing %s (%i)..." % (self.name, self.process.pid))
        self.process.kill()
        #rospy.loginfo("Waiting to die %s (%i)..." % (self.name, self.process.pid))
        #self.process.wait()
        #assert self.died()
        self.process = None

    def __str__(self):
        return " ".join([self.name, self.package, self.fname]+self.args)

launches = [ # base
        "minimal turtlebot_bringup minimal.launch",
        "kinect turtlebot_bringup 3dsensor.launch",
        # mapping
        "gmapping snap_misc gmapping.launch.xml",
        "move_base snap_misc move_base.launch.xml",
        "reset_limit_params_defaults snap_misc limit_params.launch.xml",
        "save_map snap_misc map_saver.launch",
        "load_map snap_misc map_loader.launch",
        # frontier exploration
        "frontier_exploration_server frontier_exploration global_map.launch sensor_range:=10.0",
        "frontier_exploration_client snap_misc frontier_exploration_client.launch",
        # localization
        "amcl turtlebot_navigation amcl.launch.xml"]


class LaunchManager(object):
    def __init__(self):
        #self.launch = roslaunch.scriptapi.ROSLaunch()
        #self.launch.start()

        managed_launches = map(ManagedLaunch.factory, launches)
        self.managed_launches = dict([(ml.name, ml) for ml in managed_launches])

        self.config = None
        self.srv = Server(GettingStartedConfig, self.dynparam_cb)
        self.client = Client("getting_started_node")
        
    def dynparam_cb(self, config, level):
        rospy.logdebug("Reconfigure Request: (%i) %s"%(level, (str(config))))
        rospy.logdebug("keys: %s" % str(config.keys()))

        for name, ml in self.managed_launches.iteritems():
            rospy.logdebug("cb: %s (%s)" % (name, str(ml)))
            if config[name]:
                args = []
                if name.endswith("_map"):
                    args = ["map_filename:=%s"%config.map_filename] 
                    rospy.loginfo("Map args: %s %s" % (name, args))
                ml.start(args)
            else:
                ml.stop()

        self.config = config
        return config

    def spin_once(self):
        config_updates = {}
        for name, ml in self.managed_launches.iteritems():
            rospy.logdebug("Spin: check if %s has died..."%name)
            if ml.died():
                rospy.logwarn("Launch %s (%i) has died..." % (name, ml.process.pid))
                ml.process = None
                config_updates[name] = False
        if config_updates:
            rospy.loginfo('updating config: %s'%str(config_updates))
            #self.srv.update_configuration(config_updates)
            self.client.update_configuration(config_updates)

    def spin(self, rate=5.0):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.spin_once()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("getting_started_node")

    lm = LaunchManager()
    lm.spin()

#!/usr/bin/python
import rospy
#import roslaunch
import subprocess

from dynamic_reconfigure.server import Server
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
        return self.process is not None and self.process.poll() is not None

    def start(self):
        if self.is_alive(): return

        assert self.process is None

        rospy.loginfo("Launching %s: '%s %s %s'" % (self.name, self.package, self.fname, " ".join(self.args)))
        self.process = subprocess.Popen(['roslaunch', self.package, self.fname]+self.args) #, shell=True)
        rospy.loginfo("%s PID: %i" % (self.name, self.process.pid))

    def stop(self):
        if not self.is_alive(): return

        assert self.process is not None
        rospy.loginfo("Terminating %s (%i)..." % (self.name, self.process.pid))
        self.process.terminate()
        #rospy.loginfo("Killing %s (%i)..." % (self.name, self.process.pid))
        #self.process.kill()
        #rospy.loginfo("Waiting to die %s (%i)..." % (self.name, self.process.pid))
        #self.process.wait()
        #assert self.died()
        self.process = None

    def __str__(self):
        return " ".join([self.name, self.package, self.fname]+self.args)

launches = [
        "minimal turtlebot_bringup minimal.launch",
        "kinect turtlebot_bringup 3dsensor.launch"]

class LaunchManager(object):
    def __init__(self):
        #self.launch = roslaunch.scriptapi.ROSLaunch()
        #self.launch.start()

        managed_launches = map(ManagedLaunch.factory, launches)
        self.managed_launches = dict([(ml.name, ml) for ml in managed_launches])

        self.config = None
        self.srv = Server(GettingStartedConfig, self.dynparam_cb)
        
    def dynparam_cb(self, config, level):
        rospy.loginfo("Reconfigure Request: (%i) %s"%(level, (str(config))))
        rospy.loginfo("keys: %s" % str(config.keys()))

        for name, ml in self.managed_launches.iteritems():
            rospy.loginfo("cb: %s (%s)" % (name, str(ml)))
            if config[name]:
                ml.start()
            else:
                ml.stop()

        self.config = config
        return config

    def spin_once(self):
        config_changed = False
        for name, ml in self.managed_launches.iteritems():
            if ml.died():
                rospy.logwarn("Launch %s (%i) has died..." % (name, ml.process.pid))
                ml.process = None
                if self.config is not None:
                    self.config[name] = False
                    config_changed = True
        if config_changed:
            rospy.logwarn("Config has changed. TODO: update it globally...")

    def spin(self, rate=5.0):
        r = rospy.rate(rate)
        while not rospy.is_shutdown():
            self.spin_once()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("getting_started_node")

    lm = LaunchManager()
    rospy.spin()

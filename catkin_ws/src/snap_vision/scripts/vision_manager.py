#!/usr/bin/env python
PACKAGE="snap_vision"
import rospy
import collections
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from snap_vision_msgs.msg import *
from snap_vision_msgs.srv import *
from snap_low_level_detectors.cfg import *
import pdb


models = collections.defaultdict(list, ALL=['__ALL_MODELS__'],
        duck =  ["duck_16x16_HAAR",
                 "duck_16x16_LBP"],
        bottle =  ["bottle_16x16_HAAR",
                   "bottle_16x16_LBP"],
        tape =  ["tape_16x16_HAAR",
                 "tape_16x16_LBP"],
        bot =  ["bot_16x16_HAAR",
                "bot_16x16_LBP"])

class SyncedMessages(object):
    def __init__(self, **kwargs):
        self.keys = kwargs.keys()
        self.stamp = None
        for k,v in kwargs.iteritems():
            setattr(self, k, v)
            if v is not None:
                self.stamp = v.header.stamp

    def get_stamp(self):
        if self.stamp is None:
            for k in self.keys:
                v = getattr(self, k)
                if v is not None:
                    self.stamp = v.header.stamp
        return self.stamp

class VisionManager(rospy.SubscribeListener):
    def __init__(self):
        self.synced_msgs = collections.defaultdict(lambda : SyncedMessages(image=None,detections=None,cam_info=None))

        self.buffer_size = rospy.Duration(1.0)

        self.detections_pub = rospy.Publisher("~detections", DetectionsStamped, queue_size=10,
                subscriber_listener=self)
        self.has_subscribers = False
        self.game_label_sub = rospy.Subscriber("game_label", String, self.game_label_cb, queue_size=10)

        rospy.loginfo("connecting to services")
        self.load_detector = rospy.ServiceProxy("detector_manager_node/load_detector", Detector)
        self.load_models = rospy.ServiceProxy("detector_manager_node/load_models", Models)
        self.start_stream = rospy.ServiceProxy("detector_manager_node/start_stream", Stream)
        self.unload_detector = rospy.ServiceProxy("detector_manager_node/unload_detector", Detector)
        self.unload_models = rospy.ServiceProxy("detector_manager_node/unload_models", Models)
        self.stop_stream = rospy.ServiceProxy("detector_manager_node/stop_stream", Stream)
        rospy.loginfo("connected")

        ret = self.load_detector(type="OCVCascadeDetector")
        rospy.loginfo(ret)
        ret = self.start_stream(topic_name="/image_raw", queue_size=1)
        rospy.loginfo(ret)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if not self.has_subscribers:
            rospy.loginfo("Got a subscription on %s, so subscribing in turn"%topic_name)
            self.has_subscribers = True
            self.images_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=10)
            self.detections_sub = rospy.Subscriber("detector_manager_node/detections", DetectionsStamped,
                    self.detections_cb, queue_size=10)
            self.cam_infos_sub = rospy.Subscriber("camera_info", CameraInfo, self.cam_info_cb, queue_size=10)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.has_subscribers and num_peers == 0:
            rospy.loginfo("No more subscribed peers on %s, so unsubscribing in turn", topic_name)
            self.has_subscribers = False
            self.images_sub = None
            self.detections_sub = None
            self.cam_infos_sub = None

    def image_cb(self, msg):
        self.do_message_sync(msg, "image")

    def detections_cb(self, msg):
        self.do_message_sync(msg, "detections")
        if len(msg.detections) > 0:
            self.detections_pub.publish(msg)
            rospy.loginfo("Republished")
        else:
            rospy.loginfo("No incomming detections, so suppressed")

    def cam_info_cb(self, msg):
        self.do_message_sync(msg, "cam_info")

    def game_label_cb(self, msg):
        ret = self.unload_models(models['ALL'])
        rospy.loginfo(ret)
        to_load = models[msg.data]
        rospy.loginfo("will try to load: %s -> %s"%(msg.data, to_load))
        ret = self.load_models(to_load)
        rospy.loginfo(ret)

    def do_message_sync(self, msg, name):
        now = rospy.Time.now()
        # clean up old messages
        for k in self.synced_msgs.keys():
            if now - k > self.buffer_size:
                del self.synced_msgs[k]
        stamp = msg.header.stamp
        msgs = self.synced_msgs[stamp]
        setattr(msgs, name, msg)

        #rospy.loginfo("Syncing: %s -- %s" % (str(stamp), str(now)))
        #pdb.set_trace()

import dynamic_reconfigure.client


if __name__ == '__main__':
    rospy.init_node('vision_manager')
    vm = VisionManager()
    rospy.spin()

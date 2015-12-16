#!/usr/bin/env python
PACKAGE="snap_vision"
import rospy
from sensor_msgs.msg import Image, CameraInfo
from snap_vision_msgs.msg import *
from snap_low_level_detectors.cfg import *

class VisionManager(rospy.SubscribeListener):
    def __init__(self):
        self.images = {}
        self.detections = {}
        self.cam_infos = {}
        self.synced_msgs = {}

        self.buffer_size = rospy.Duration(1.0)

        self.detections_pub = rospy.Publisher("~detections", DetectionsStamped, queue_size=10,
                subscriber_listener=self)
        self.has_subscribers = False

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if not self.has_subscribers:
            rospy.loginfo("Got a subscription, so subscribing in turn")
            self.has_subscribers = True
            self.images_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=10)
            self.detections_sub = rospy.Subscriber("detections_in", DetectionsStamped, self.detections_cb, queue_size=10)
            self.cam_infos_sub = rospy.Subscriber("camera_info", CameraInfo, self.cam_info_cb, queue_size=10)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.has_subscribers and num_peers == 0:
            rospy.loginfo("No subscribed peers, so unsubscribing in turn")
            self.has_subscribers = False
            self.images_sub = None
            self.detections_sub = None
            self.cam_infos_sub = None

    def image_cb(self, msg):
        self.images[msg.header.stamp] = msg
        self.do_message_sync(msg.header.stamp)

    def detections_cb(self, msg):
        self.detections[msg.header.stamp] = msg
        self.do_message_sync(msg.header.stamp)

    def cam_info_cb(self, msg):
        self.cam_infos[msg.header.stamp] = msg
        self.do_message_sync(msg.header.stamp)

    def do_message_sync(self, stamp):
        now = rospy.Time.now()
        rospy.loginfo("Syncing: %s -- %s" % (str(stamp), str(now)))

import dynamic_reconfigure.client


if __name__ == '__main__':
    rospy.init_node('vision_manager')
    vm = VisionManager()
    rospy.spin()

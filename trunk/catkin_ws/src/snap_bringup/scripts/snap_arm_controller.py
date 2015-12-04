#!/usr/bin/env python


import rospy



'''
Listen and Talk
by David Isele

'''


import roslib
import rospy
import os
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import *

class my_process():

	def __init__(self):
		# setup
		rospy.init_node("snap_arm_positions")	
		self.pub = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=5)
		self.sub = rospy.Subscriber("/arm_test/listener",Float64,self.SensorPowerEventCallback)
		rate = rospy.Rate(10) # 10hz

		rospy.spin();


	def SensorPowerEventCallback(self,data):

		my_message = "I have recieved a command!"

		rospy.loginfo(my_message)
		print data
		#pub.publish(data)
		#for somereason rate is not defined. i must be missing a library
		#rate.sleep()

if __name__ == '__main__':
	try:
		my_process()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")

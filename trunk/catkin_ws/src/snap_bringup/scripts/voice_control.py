#!/usr/bin/env python

# High level intelligence script. Using voice command to control the robot
# Yongbo Qian @ 2016

import sys
import copy
import rospy
import os
from std_msgs.msg import *
from geometry_msgs.msg import *

os.system("rosrun sound_play say.py 'Hey folks, I am snap, what can I do for you'")

received_voice = None
while received_voice is None or received_voice.data != "happy hunting": 
      if rospy.is_shutdown():
          os._exit(0)
      try:
         received_voice = rospy.wait_for_message("/recognizer/output",String,timeout=10)
      except Exception, e:
         os.system("rosrun sound_play say.py 'what should i hunt'")
         rospy.sleep(2)
         break
         

obj = "duck"
while received_voice is None or received_voice.data != obj: #model name
      if rospy.is_shutdown():
          os._exit(0)
      try:
         received_voice = rospy.wait_for_message("/recognizer/output",String,timeout=10)
      except Exception, e:
         cmd = "rosservice call /detector_manager_node/load_models 'model_names: ['"+ str(obj)+"_32x32_haar']'"
        # cmd = "rosservice call /detector_manager_node/load_models 'model_names: ['duck_32x32_haar']'"
         os.system(cmd)
         #os.system("rosservice call /detector_manager_node/start_stream "{topic_name: '/camera/arm/image_raw', queue_size: 3}"")
         os.system("rosrun sound_play say.py 'detector loaded, start hunting'")
         rospy.sleep(2)
         break

print('todo: search object')

os.system("rostopic pub -1 /simple_search/activeCmd std_msgs/Bool True")

search_status = None
#search_status = rospy.wait_for_message("/status/updates",String)
#print (search_status.data)
#if search_status.data == "Simple search has timed out":
#   os.system("rosrun sound_play say.py 'I cannot see duck can you lead me to it'")
#   rospy.sleep(3)
   

while search_status is None or search_status.data != "Simple search has timed out":
      if rospy.is_shutdown():
         os._exit(0)
      try:
         search_status = rospy.wait_for_message("/status/updates",String,timeout=40)
         print (search_status.data)
      except Exception, e:
         os.system("rosrun sound_play say.py 'I cannot see duck, can you lead me to it'")
         rospy.sleep(3)
         break

#os.system("roslaunch turtlebot_follower follower.launch")
'''
if: #detected 
   os.system("rosnode kill turtlebot_follower")
   print ("todo: duck hunt")
   os.system("rosrun snap_bringup duck_hunt2.py")



while received_voice is None or received_voice.data != "success":
      if rospy.is_shutdown():
          os._exit(0)
      try:
         received_voice = rospy.wait_for_message("/recognizer/output",String,timeout=10)
      except Exception, e:
         #shutdown everything
         rospy.sleep(2)
         break
   
'''

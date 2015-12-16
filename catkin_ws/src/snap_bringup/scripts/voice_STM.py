#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
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
#  * Neither the name of SRI International nor the names of its
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
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import os
#import moveit_commander
#import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from snap_vision_msgs.msg import *
from numpy import *
from std_msgs.msg import String

class voice_STM_node():

  def __init__(self):
      ## BEGIN_TUTORIAL
      ##
      ## Setup
      ## ^^^^^
      ## CALL_SUB_TUTORIAL imports
      ##
      ## First initialize  rospy.
      ## .
      print "============ Starting SNAP State Machine"
      #import pdb; pdb.set_trace()
      #moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('voice_STM',
                      anonymous=True)
      # Execute this function when shutting down
      rospy.on_shutdown(self.shutdown)
      
      # Set the update rate to 1 second by default
      self.rate = rospy.get_param("~node_rate", 10)
      r = rospy.Rate(self.rate)
      self.state = 'free'
      self.state_prev = 'free'
      self.attemps = 0.0
      self.obj = ['duck','bottle','tape']
      #move base veolicty and arm traj topic

      print "Begining STM"
      # Begin the main loop
      while not rospy.is_shutdown():
           self.STM()
           r.sleep()

      
  def STM(self):
     
     if self.state == 'free':
        received_voice = self.listen("/recognizer/output")
        if received_voice == 'hello':
            self.state = 'w2d'
        else:
            self.state = 'free'
        
     elif self.state == 'w2d':
        os.system("rosrun sound_play say.py 'Hey folks, I am snap, what can I do for you'")
        received_voice = self.listen("/recognizer/output")
        if received_voice == 'happy hunting':
            self.state = 'w2h'
        elif received_voice == 'nothing':
            self.state = 'free'
            os.system("rosrun sound_play say.py 'Okay. Good Bye'")
        elif received_voice:
            #os.system("rosrun sound_play say.py 'Unknown Command.'")
            #self.state = 'speech_timeout'
            print 'unknown cmd heared'
        elif received_voice is None:
            os.system("rosrun sound_play say.py 'I cannot hear you'")
            self.state = 'free'
        
     elif self.state == 'w2h':
        os.system("rosrun sound_play say.py 'what should i hunt'")
        received_voice = self.listen("/recognizer/output")
        if received_voice in self.obj:
            cmd = "rosservice call /detector_manager_node/load_models 'model_names: ['"+ str(received_voice)+"_32x32_haar']'"
            os.system(cmd)
            #os.system("rosservice call /detector_manager_node/start_stream "{topic_name: '/camera/arm/image_raw', queue_size: 3}"")
            os.system("rosrun sound_play say.py 'detector loaded, start searching'")
            rospy.sleep(2)
            self.state = 'searching'
            os.system("rostopic pub -1 /simple_search/activeCmd std_msgs/Bool True")
        elif received_voice:
            #os.system("rosrun sound_play say.py 'Unknown Command.'")
            #self.state = 'speech_timeout'
            print 'unknown cmd heared'
        elif received_voice is None or received_voice == 'stop':
            os.system("rosrun sound_play say.py 'I cannot hear you'")
            self.state = 'free'

     elif self.state == 'searching': 
        try:
         search_status = rospy.client.wait_for_message("/status/updates",String,timeout=130) #slightly larger than node timeout
         #import pdb; pdb.set_trace()
         #print (search_status.data)
        except Exception, e:
         os.system("rostopic pub -1 /simple_search/activeCmd std_msgs/Bool False")
         #os.system("rosrun sound_play say.py 'I cannot see duck, can you lead me to it'")
         print 'exception in searching state'
         search_status = None
         rospy.sleep(3)
         
         if search_status == 'Duck Found':
            os.system("rosrun sound_play say.py 'Target Aquired. Hunting Continue.'")
            os.system("rostopic pub -1 /duckhunter/activeCmd  std_msgs/Bool True")
            self.state = 'hunting'
         elif search_status == 'Simple search has timed out' or None:
            #both lead to follow person
            self.state = 'follow_person'
            os.system("rosrun sound_play say.py 'I cannot see duck, can you lead me to it'")
            #os.system("roslaunch turtlebot_follower follower.launch")
         #elif search_status == 'search ongoing':
            #self.state = 'searching'         
         
     elif self.state == 'hunting':
        try:
         hunt_status = rospy.client.wait_for_message("/status/updates",String,timeout=130) #slightly larger than node timeout
        except Exception, e:
         os.system("rostopic pub -1 /duckhunting/activeCmd std_msgs/Bool False")
         #os.system("rosrun sound_play say.py 'I cannot see duck, can you lead me to it'")
         print 'exception in hunting state'
         hunt_status = None
         rospy.sleep(3)
         
        if hunt_status == 'Hunt Success':
            #listen to person if I have the duck?
            os.system("rosrun sound_play say.py 'Was I successful? Is, so say SUCCESS'")
            received_voice = self.listen("/recognizer/output")
            if received_voice == 'success'
                self.state ='w2d'
            elif received_voice:
                self.state = 'a4help'
            elif received_voice is None:
                self.state = 'a4help'
        else: #hunt_status is None:
            self.state = 'a4help'
            print 'unknown cmd heared'
        
        
     elif self.state == 'follow_person':
        #either listen to person or just look for duck.
        #looking for duck
        try:
         duck_data = rospy.client.wait_for_message("/detector_manager_node/detections", DetectionsStamped,timeout=180) #
        except Exception, e:
         os.system("rosrun sound_play say.py 'I am too tired. Going to give up.'")
         print 'exception in person following'
         duck_data = None
         rospy.sleep(3)
         
        if duck_data.detections:
            os.system("rosrun sound_play say.py 'Thank you. Going to hunt.'")
            self.state = 'hunting'
        else:
            self.state = 'free'
        
     
     elif self.state == 'a4help':
        os.system("rosrun sound_play say.py 'Attempt Failed. Can you help me? Say Yes, Or Say Try Again.'"
        received_voice = self.listen("/recognizer/output")
            if received_voice == 'yes'
                self.state ='w2d'
            elif received_voice == 'try again':
                os.system("rosrun sound_play say.py ' Hunting Continue.'")
                os.system("rostopic pub -1 /duckhunter/activeCmd  std_msgs/Bool True")
                self.state = 'hunting'
            else:
                os.system("rosrun sound_play say.py ' I am giving up.'")
                self.state = 'free'
     
     elif self.state == 'speech_timeout':
        os.system("rosrun sound_play say.py 'I cannot hear you'")
     else:
        rospy.loginfo('ERROR UNROCOGNIZED STATE')
        
  def listen(self, topic):
      try:
         #import pdb; pdb.set_trace()
         received_voice = rospy.client.wait_for_message(str(topic),String,timeout=10)
      except Exception, e:
         #os.system("rosrun sound_play say.py 'I cannot hear you'")
         #rospy.sleep(2)
         received_voice = None
         #self.state_prev = self.state
         #self.state = 'speech_timeout'
      return received_voice
          
  def shutdown(self):
      rospy.loginfo("Shutting down search Node...")


      ## When finished shut down 


      ## END_TUTORIAL

      print "============ STOPPING"


if __name__=='__main__':
  try:
    my_node = voice_STM_node()
  except rospy.ROSInterruptException:
    pass

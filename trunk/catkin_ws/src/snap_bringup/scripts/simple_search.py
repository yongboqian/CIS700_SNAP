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
#import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from snap_vision_msgs.msg import *
from numpy import *
## END_SUB_TUTORIAL

from std_msgs.msg import String

class simple_search_node():

  def __init__(self):
      ## BEGIN_TUTORIAL
      ##
      ## Setup
      ## ^^^^^
      ## CALL_SUB_TUTORIAL imports
      ##
      ## First initialize  rospy.
      ## THis requires the move_base to be up and for the trajectory node to be up.
      print "============ Starting Search"
      #import pdb; pdb.set_trace()
      #moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('simple_search',
                      anonymous=True)
      # Execute this function when shutting down
      rospy.on_shutdown(self.shutdown)
      
      # Set the update rate to 1 second by default
      self.rate = rospy.get_param("~node_rate", 10)
      r = rospy.Rate(self.rate)
      #move base veolicty and arm traj topic 
      self.base_pub = rospy.Publisher('~/cmd_vel_mux/input/teleop', Twist, queue_size=10)                                    
      self.arm_pub = rospy.Publisher("~/simple_move", String, queue_size=1)
      self.grip_pub = rospy.Publisher('~/gripper_joint/command', Float64, queue_size=2) 
      self.status_info_pub = rospy.Publisher('~/status/updates', String, queue_size=10) 
      rospy.Subscriber("~/detector_manager_node/detections", DetectionsStamped, self.DetectionCb)
      rospy.Subscriber("~/simple_search/activeCmd", Bool, self.ActiveCb)
      rospy.Subscriber("~/scan", LaserScan, self.LaserScanCb)
      

      self.hunting_tolerance = 10 # tolerance on how zeroed in the arm needs to be
      self.hunting_tolerance_bb = 10
      self.hunting_turn_speed = 0.6 # not sure what this is measured in
      self.hunting_for_speed = 0.06 #0.02 is okay
      self.hunting_box_width_goal = 170 #ideal size of BB
      #self.hunting_box_height_goal = 100 #ideal size of BB
      self.hunting_img_width = 1280 #width of webcam image
      self.hunting_img_height = 720 #height of webcam image
      self.hunting_img_center_x = self.hunting_img_width/2
      self.hunting_img_center_y = self.hunting_img_height/2
      self.control_turn = 0.0
      self.control_froward = 0.0
      self.turn_error = 0.0
      self.turn_error_prev = 0.0
      self.kp_turn = 0.01
      self.kd_turn = 0.0
      self.grip_close = 1.6
      self.grip_open = 0
      self.active = False
      self.active_time0 = 0.0
      self.timeout_time = 0.0
      self.active_timeout = 120 #30seconds to timeout
      self.turn_timeout = 5
      self.turn_timeout0 = 0.0
      self.turn_last = False
      self.too_close = 1.0
      self.closest_point = 0.0
      self.rand_turn = 0.0
      ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
      #print "============ Waiting for RVIZ..."
      #rospy.sleep(5)
      print "============ Sending arm to Rest Position and opening gripper"
      data = 'rest' #start cmd for moving arm
      self.move_arm(data)
      rospy.sleep(5)
      self.grip_pub.publish(self.grip_open)
      rospy.sleep(5)
      #import pdb; pdb.set_trace()
      #data = rospy.client.wait_for_message("/simple_search/activeCmd", Bool, timeout = 10)
      print "Begining Search"
      # Begin the main loop
      while not rospy.is_shutdown():
           self.SimpleSearch()
           r.sleep()
      #rospy.spin()  

      ## Now, we call the planner to compute the plan
      ## and visualize it if successful
      ## Note that we are just planning, not asking move_group 
      ## to actually move the robot
      #plan1 = group.plan()
  def ActiveCb(self, data):
      self.active = data.data # can deactivate
      self.active_time0 = rospy.get_time()
      self.timeout_time = self.active_time0+self.active_timeout
      if self.active:
        print "============ Search Started"
      #set timeout Time maybe?
      
  def SimpleSearch(self):
      #print self.timeout_time
      #print rospy.get_time()
      #print self.active
      timein= self.timeout_time>rospy.get_time()
      #self print.active
      #print timein
      if self.active and timein:
        #search
        print 'searching...'
        self.status_info_pub.publish ('search ongoing')
        if self.closest_point >self.too_close:
            
            forward_speed = self.hunting_for_speed
            if self.turn_last:
                #TODO add random code
                self.rand_turn = 0.0
            #TODO add a timer to perodically stop and do a 360 to check for ducks
            #maybe add a random turn speed that will induce random path and hopefull lead to searhcing
            self.move_base(forward_speed,self.rand_turn)
            #reset that it turned
            self.turn_last = False
        else:
            #turn for x seconds. maybe send command to plannar so turtle bot reaches a pose
            if not self.turn_last:
                #this can be used to toggle turning behavours
                self.turn_last = True
            self.turn_timeout0 = rospy.get_time()
            self.move_base(0.0,self.hunting_turn_speed)
      elif not timein and self.active:
        #deactivate and prepare shutdown
        self.active = False
        good_data = False
        self.status_info_pub.publish ('Simple search has timed out')
        self.shutdown()
      #else:
        #do nothing
        #print 'waiting'
        
  def LaserScanCb(self, data):
      if data.ranges:
        #if you have good data
        #convert to an array and get rid of nans
        array = asarray(data.ranges)
        bad_ind = isnan(array)
        #import pdb; pdb.set_trace()
        #print bad_ind
        array[bad_ind] = data.range_max
        min_value = min(array)
        self.closest_point = min_value
        #print min_value
        #import pdb; pdb.set_trace()
            
      
  def DetectionCb(self, data):
      ## I have recieved data on a duck that has been detected. I will now process this data and command the base to center in on the duck. Then I will command the arm to poke the duck
      #print "Callback"
      ## Processing data
      #check if there is good data
      #TODO
      print 'duck detected'
      self.move_base(0.0,0.0)
      timein= self.timeout_time>rospy.get_time()
      #self print.active
      print timein
      if data.detections and self.active and timein:#is array is empty this will return false else true
        good_data = True #assume data will always be good
      elif not timein and self.active:
        #deactivate and prepare shutdown
        self.active = False
        good_data = False
        self.status_info_pub.publish ('Simple search has timed out')
        self.shutdown()
      else:
        good_data = False
      #
      #Find detection with best confidence
      #print good_data
      if good_data:
          #send cmd to next node to start
          self.move_base(0.0,0.0)
          self.status_info_pub.publish ('Duck Found')
          print 'finished searching. Found duck. Continue to next node'
          
            
          
          
      
      
  def move_arm(self,data):
      ## send command to arm
      #data.data = 'rest' #start cmd for moving arm
      print "sending cmd to arm"
      self.arm_pub.publish(data)
      
  def move_base(self, forward_speed, turning_speed):
      ## method will handel cmding the base to turn an amount based on how much offset
      ## there is. The offset is directional, the sign defines the rotation, CCW or CW
      
      
      print "Turn Speed"
      print turning_speed
      #moveing forward code
      twist = Twist()
      twist.linear.x = forward_speed; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turning_speed
      self.base_pub.publish(twist)

      
  def shutdown(self):
      rospy.loginfo("Shutting down search Node...")


      ## When finished shut down 
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      self.base_pub.publish(twist)

      ## END_TUTORIAL

      print "============ STOPPING"


if __name__=='__main__':
  try:
    my_node = simple_search_node()
  except rospy.ROSInterruptException:
    pass

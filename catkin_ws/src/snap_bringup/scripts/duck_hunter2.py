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
from snap_vision_msgs.msg import *
## END_SUB_TUTORIAL

from std_msgs.msg import String

class duck_hunter_node():

  def __init__(self):
      ## BEGIN_TUTORIAL
      ##
      ## Setup
      ## ^^^^^
      ## CALL_SUB_TUTORIAL imports
      ##
      ## First initialize  rospy.
      ## THis requires the move_base to be up and for the trajectory node to be up.
      print "============ Starting Duck Hunter"
      #import pdb; pdb.set_trace()
      #moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('duck_hunter',
                      anonymous=True)
      # Execute this function when shutting down
      rospy.on_shutdown(self.shutdown)
      
      # Set the update rate to 1 second by default
      self.rate = rospy.get_param("~node_rate", 10)
      r = rospy.Rate(self.rate)
      #move base veolicty and arm traj topic 
      self.base_pub = rospy.Publisher('~/cmd_vel_mux/input/teleop', Twist, queue_size=5)                                    
      self.arm_pub = rospy.Publisher("~/simple_move", String, queue_size=1)
      self.grip_pub = rospy.Publisher('~/gripper_joint/command', Float64, queue_size=2) 
      self.status_info_pub = rospy.Publisher('~/status/updates', String, queue_size=10) 
      rospy.Subscriber("~/detector_manager_node/detections", DetectionsStamped, self.DetectionCb)
      rospy.Subscriber("~/duckhunter/activeCmd", Bool, self.ActiveCb)
      

      self.hunting_tolerance = 10 # tolerance on how zeroed in the arm needs to be
      self.hunting_tolerance_bb = 10
      self.hunting_turn_speed = 0.6 # not sure what this is measured in
      self.hunting_for_speed = 0.0 #0.02 is okay
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
      ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
      #print "============ Waiting for RVIZ..."
      #rospy.sleep(5)
      print "============ Sending arm to Rest Position and opening gripper"
      data = 'rest' #start cmd for moving arm
      #self.move_arm(data)
      #rospy.sleep(10)
      #self.grip_pub.publish(self.grip_open)
      #rospy.sleep(5)
      print "============ Duck Hunter Started"

     
      # Begin the main loop
      #while not rospy.is_shutdown():
           #r.sleep()
      rospy.spin()  

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
        print 'active'
      #set timeout Time maybe?
      
  def DetectionCb(self, data):
      ## I have recieved data on a duck that has been detected. I will now process this data and command the base to center in on the duck. Then I will command the arm to poke the duck
      #print "Callback"
      ## Processing data
      #check if there is good data
      #TODO
      timein= self.timeout_time>rospy.get_time()
      #self print.active
      print 'timein'
      print timein
      if data.detections and self.active and timein:#is array is empty this will return false else true
        good_data = True #assume data will always be good
      elif not timein and self.active:
        #deactivate and prepare shutdown
        self.active = False
        good_data = False
        self.status_info_pub.publish ('DuckHunter has timed out')
        rospy.sleep(2)
        self.shutdown()
      else:
        good_data = False
      #
      #Find detection with best confidence
      #print good_data
      if good_data:
          i = []
          best_conf = 0
          for j in range(0,len(data.detections)):
            if data.detections[j].confidence > best_conf and data.detections[j].label =="duck_16x16_HAAR":
                best_conf = data.detections[j].confidence
                i = j
          #we should have the index of the most duckly object
          print('best_config',best_conf)
          if best_conf is not None:#if we have a duck 
              print "Quack"   
              #Find the center of the bbox and calculate the offset
              bbox_center_x = data.detections[i].bbox.x+data.detections[i].bbox.width/2# to be calculated from msg
              bbox_center_y = data.detections[i].bbox.y+data.detections[i].bbox.height/2
              center_offset_x = self.hunting_img_center_x-bbox_center_x#^
              center_offset_y = self.hunting_img_center_y-bbox_center_y#0.0#^
              bb_width = data.detections[i].bbox.width#100#^
              bb_height = data.detections[i].bbox.height#100#^
              #bb_offset = bb_width -hunting_box_width_goal
              bb_offset = bb_width -  self.hunting_box_width_goal#
              print "offset"
              print center_offset_x
              print "bb_offset"
              print bb_offset
              #print "bbbox center"
              #print bbox_center_x
              #print "img center" 
              #print self.hunting_img_center_x
              
              #estimate how far away robot is based on the expected size of the bounding box
              #function of: width and/or height
              
              
              target_aqrd = False
              # logic to control how to move base. and then cmd base movement
              target_aqrd = self.move_base(center_offset_x, bb_offset) # i think it gets x. might need y instead
              # check how far away I am
              #TODO
              #target_aqrd = False
              #Do arm stuff
              #TODO
              if target_aqrd: #assume we are within pokeing distance to the 
                #do arm stuff
                if 1:
                    rospy.loginfo("target aquired")
                    data = 'straight' #start cmd for moving arm
                    rospy.loginfo("Pokeing")
                    self.move_arm(data)
                    rospy.sleep(10)
                    self.grip_pub.publish(self.grip_close)
                    rospy.sleep(5)
                    #wait som time
                    rospy.loginfo("resting")
                    data = 'rest' #start cmd for moving arm
                    self.move_arm(data)
                    rospy.sleep(10)
                    self.grip_pub.publish(self.grip_open)
                    rospy.sleep(5)
                self.active = False # deactiveate loop. mission complete
                self.status_info_pub.publish ('Hunt Success')
                rospy.sleep(2)
                #send to reset
                #go
            
          
          
      
      
  def move_arm(self,data):
      ## send command to arm
      #data.data = 'rest' #start cmd for moving arm
      print "sending cmd to arm"
      self.arm_pub.publish(data)
      
  def move_base(self, rot_offset, dist_off):
      ## method will handel cmding the base to turn an amount based on how much offset
      ## there is. The offset is directional, the sign defines the rotation, CCW or CW
      self.turn_error = rot_offset
      target_aqrd = False  
      target_aqrd_for = False
      
      # I may want to make this a propostional controller with limits
      turn_speed = self.hunting_turn_speed
      #Kp = 1
      turn_speed = self.turn_error*self.kp_turn +(self.turn_error-self.turn_error_prev)*self.kd_turn
      #limit speed
      if turn_speed < -self.hunting_turn_speed:
        turn_speed = -self.hunting_turn_speed
        print 'turning too fast'
      elif turn_speed > self.hunting_turn_speed:
        turn_speed = self.hunting_turn_speed
        print 'turning too fast'
      #set speed and if close enough stop        
      if abs(self.turn_error) < self.hunting_tolerance:
        print "Target Aquired"
        target_aqrd = True
        self.control_turn = 0.0
      else:
        self.control_turn = turn_speed
      
      
      #figure out how to go forward
      forward_speed = self.hunting_for_speed
      
      if dist_off > self.hunting_tolerance_bb:
        forward_speed = forward_speed
      elif dist_off < -self.hunting_tolerance_bb:
        forward_speed = -forward_speed
      else:
        forward_speed = 0
        target_aqrd_for = True
        
      #if self.turn_error > self.hunting_tolerance:
        #turn CW i think
        #self.control_turn = 0.7*self.control_turn + 0.3*turn_speed
        #rospy.loginfo("Turning CCW")
      #elif self.turn_error < -self.hunting_tolerance:
        #turn CCW i think
        #self.control_turn = 0.7*self.control_turn - 0.3*turn_speed
        #rospy.loginfo("Turning CW")
      #else:
        #print "Target Aquired"
        #target_aqrd = True
        #self.control_turn = 0#0.9*self.control_turn
      
      print "Turn Speed"
      print self.control_turn
      #moveing forward code
      twist = Twist()
      twist.linear.x = forward_speed; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
      self.base_pub.publish(twist)
      
      return (target_aqrd)# and target_aqrd_for)

      
  def shutdown(self):
      rospy.loginfo("Shutting down moveit Node...")


      ## When finished shut down 
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      self.base_pub.publish(twist)

      ## END_TUTORIAL

      print "============ STOPPING"


if __name__=='__main__':
  try:
    my_node = duck_hunter_node()
  except rospy.ROSInterruptException:
    pass

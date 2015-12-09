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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from snap_vision_msgs import *
## END_SUB_TUTORIAL

from std_msgs.msg import String

class move_traj_node():

  def __init__(self):
      ## BEGIN_TUTORIAL
      ##
      ## Setup
      ## ^^^^^
      ## CALL_SUB_TUTORIAL imports
      ##
      ## First initialize moveit_commander and rospy.
      print "============ Starting tutorial setup"
      #import pdb; pdb.set_trace()
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('duck_hunter',
                      anonymous=True)
      # Execute this function when shutting down
      rospy.on_shutdown(self.shutdown)
      
      # Set the update rate to 1 second by default
      self.rate = rospy.get_param("~node_rate", 10)
      r = rospy.Rate(self.rate)

      ## Instantiate a RobotCommander object.  This object is an interface to
      ## the robot as a whole.
      self.robot = moveit_commander.RobotCommander()

      ## Instantiate a PlanningSceneInterface object.  This object is an interface
      ## to the world surrounding the robot.
      self.scene = moveit_commander.PlanningSceneInterface()

      ## Instantiate a MoveGroupCommander object.  This object is an interface
      ## to one group of joints.  In this case the group is the joints in the left
      ## arm.  This interface can be used to plan and execute motions on the left
      ## arm.
      self.group = moveit_commander.MoveGroupCommander("arm")#hi think this needs to match the group published by the initalized moveit node(not in this node. should have been run before this node starts)
      #import pdb; pdb.set_trace()

      ## We create this DisplayTrajectory publisher which is used below to publish
      ## trajectories for RVIZ to visualize.
      self.display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          moveit_msgs.msg.DisplayTrajectory)
      self.base_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)                                    
      rospy.Subscriber("/simple_move", String, self.moveCb)
      rospy.Subscriber("/Detection", DetectionStamped, self.DetectionCb)
      
      
      self.hunting.tolerance = 0.01 # tolerance on how zeroed in the arm needs to be
      self.hunting.turn_speed = 0.1 # not sure what this is measured in
      self.hunting.box_width_goal = 100 #ideal size of BB
      self.hunting.box_height_goal = 100 #ideal size of BB
      self.hunting.image.width = 640 #width of webcam image
      self.hunting.image.height = 360 #height of webcam image
      self.hunting.image.center.x = self.hunting.image.width/2
      self.hunting.image.center.y = self.hunting.image.height/2
      ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
      print "============ Waiting for RVIZ..."
      rospy.sleep(5)
      print "============ Starting Duck Hunter "

      ## Getting Basic Information
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^
      ##
      ## We can get the name of the reference frame for this robot
      print "============ Reference frame: %s" % self.group.get_planning_frame()

      ## We can also print the name of the end-effector link for this group
      print "============ Reference frame: %s" % self.group.get_end_effector_link()

      ## We can get a list of all the groups in the robot
      print "============ Robot Groups:"
      print self.robot.get_group_names()

      ## Sometimes for debugging it is useful to print the entire state of the
      ## robot.
      print "============ Printing robot state"
      print self.robot.get_current_state()
      print "============"
      #set goal tolerance to be very large and hopefully solve my problems
      #self.group.set_goal_orientation_tolerance(3.14)
      ## Planning to a Pose goal
      ## ^^^^^^^^^^^^^^^^^^^^^^^
      ## We can plan a motion for this group to a desired pose for the 
      ## end-effector
      #put a while loop here to keep trying to update position or put this as a callback for joystick updates..
      pose_target = geometry_msgs.msg.Pose()
      #import pdb; pdb.set_trace()
      # Begin the main loop
      #while not rospy.is_shutdown():
           #r.sleep()
      rospy.spin()  

      ## Now, we call the planner to compute the plan
      ## and visualize it if successful
      ## Note that we are just planning, not asking move_group 
      ## to actually move the robot
      #plan1 = group.plan()
  def DetectionCb(self, data):
      ## I have recieved data on a duck that has been detected. I will now process this data and command the base to center in on the duck. Then I will command the arm to poke the duck
      
      ## Processing data
      #check if there is good data
      #TODO
      if data.detections:#is array is empty this will return false else true
        good_data = True #assume data will always be good
      else:
        good_data = False
      #
      #Find detection with best confidence
      if good_data:
          i = []
          best_conf = 0
          for j in range(0,len(data.detections)):
            if data.detections[j].confidence > best_conf and data.detections[j].label =="duck":
                best_conf = data.detections[j].confidence
                i = j
          #we should have the index of the most duckly object
          if best_conf != 0:#if we have a duck    
              #Find the center of the bbox and calculate the offset
              bbox_center.x = data.detection[i].bbox.x-data.detection[i].bbox.width/2# to be calculated from msg
              bbox_center.y = data.detection[i].bbox.y-data.detection[i].bbox.height/2
              center_offset.x = self.hunting.image.center.x-bbox_center.x #^
              center_offset.y = self.hunting.image.center.y-bbox_center.y#0.0#^
              bb_width = data.detection[i].bbox.width#100#^
              bb_height = data.detection[i].bbox.height#100#^
              
              #estimate how far away robot is based on the expected size of the bounding box
              #function of: width and/or height
              distEst = bbwidth -  self.hunting.box_width_goal#
              
              
              # logic to control how to move base. and then cmd base movement
              target_aqrd = self.move_base(center_offset.x, distEst) # i think it gets x. might need y instead
              # check how far away I am
              #TODO
              
              #Do arm stuff
              #TODO
              if target_aqrd: #assume we are within pokeing distance to the 
                #do arm stuff
                rospy.loginfo("target aquired")
                data.data = 'rest' #start cmd for moving arm
                self.moveCb(data)
            
          
          
      
      
  def move_base(self, rot_offset, dist_off)
      ## method will handel cmding the base to turn an amount based on how much offset
      ## there is. The offset is directional, the sign defines the rotation, CCW or CW
      
      # I may want to make this a propostional controller with limits
      turn_speed = self.hunting.turn_speed
      #Kp = 1
      #turn_speed = rot_offset*Kp
      if turn_speed > self.hunting.turn_speed:
        turn_speed = self.hunting.turn_speed
      elif turn_speed < self.hunting.turn_speed:
        turn_speed = -self.hunting.turn_speed
      target_aqrd = False  
      
      if rot_offset > self.hunting.tolerance:
        #turn CW i think
        control_turn = turn_speed
        rospy.loginfo("Turning CW")
      elif rot_offset < -self.hunting.tolerance:
        #turn CCW i think
        control_turn = -turn_speed
        rospy.loginfo("Turning CCW")
      else:
        target_aqrd = True
        control_turn = 0
        
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
      self.base_pub.publish(twist)
      return target_aqrd
          
  def moveCb(self, data):

      ## Moving to a pose goal
      ## ^^^^^^^^^^^^^^^^^^^^^
      ##
      ## Moving to a pose goal is similar to the step above
      ## except we now use the go() function. Note that
      ## the pose goal we had set earlier is still active 
      ## and so the robot will try to move to that goal. We will
      ## not use that function in this tutorial since it is 
      ## a blocking function and requires a controller to be active
      ## and report success on execution of a trajectory.

      # Uncomment below line when working with a real robot
      #group.go(wait=True)
      
      ## Planning to a joint-space goal 
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      ##
      ## Let's set a joint space goal and move towards it. 
      ## First, we will clear the pose target we had just set.

      self.group.clear_pose_targets()
      #import pdb; pdb.set_trace()
      ## Then, we will get the current set of joint values for the group
      group_variable_values = self.group.get_current_joint_values()
      print "============ Joint values: ", group_variable_values

      ## Now, let's modify one of the joints, plan to the new joint
      ## space goal and visualize the plan
      #group_variable_values[0] = 1.0
      straight_joint_goals = [-1.45,0,0,0,0]
      rest_joint_goals = [0.0,2.3,-2.0,0,0]
      cmd = data.data
      if cmd == str('rest'):
        group_variable_values = rest_joint_goals
        rospy.loginfo('Attempting Rest State Plan')
      elif cmd =='straight':
        group_variable_values = straight_joint_goals
        rospy.loginfo('Attempting Straight State Plan')
      elif cmd == 'wave':
        rospy.loginfo('Attempting Wave State Plan')
        num_waves = 1
        time_wait = 1
        wave_joint_goals0 = [0.0,1.8,-0.2,-1.5,0]
        wave_joint_goals1 = [(0.9-1.45),1.8,-0.2,-1.5,0]
        #wave_joint_goals1[0] =1.6-0.7 
        wave_joint_goals2 = [(2.3-1.45),1.8,-0.2,-1.5,0]
        #wave_joint_goals2[0] = 1.6+0.7 
        #import pdb; pdb.set_trace()
      else:
        #do not move
        group_variable_values=group_variable_values
        rospy.loginfo('Unrecongized trajectory. Loading current State as goal')
      #import pdb; pdb.set_trace()
      if (cmd == 'wave'): # sinlge else for loop wave for time or until something happens?
        count = 0
        while (not rospy.is_shutdown()) and count <num_waves :
            #exicute wave
            group_variable_values = wave_joint_goals0
            self.group.set_joint_value_target(group_variable_values)
            plan2 = self.group.plan()
            self.group.go(wait=True)
            #rospy.sleep(time_wait)
            self.group.clear_pose_targets()
            
            
            group_variable_values = wave_joint_goals1
            self.group.set_joint_value_target(group_variable_values)
            plan2 = self.group.plan()
            self.group.go(wait=True)
            #rospy.sleep(time_wait)
            self.group.clear_pose_targets()
            
            group_variable_values = wave_joint_goals2
            self.group.set_joint_value_target(group_variable_values)
            plan2 = self.group.plan()
            self.group.go(wait=True)
            #rospy.sleep(time_wait)
            self.group.clear_pose_targets()
            
            group_variable_values = wave_joint_goals0
            self.group.set_joint_value_target(group_variable_values)
            plan2 = self.group.plan()
            self.group.go(wait=True)
            #rospy.sleep(time_wait)
            self.group.clear_pose_targets()
            count = count + 1
            
      else:
        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        self.group.go(wait=True)

      print "============ Waiting while RVIZ displays plan2 (5sec)..."
      rospy.sleep(5)

      print "============ exiting movement.."

    # Shutdown function just prints a message.
  def shutdown(self):
      rospy.loginfo("Shutting down moveit Node...")
      ## Adding/Removing Objects and Attaching/Detaching Objects
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      ## First, we will define the collision object message
      collision_object = moveit_msgs.msg.CollisionObject()

      ## When finished shut down moveit_commander.
      moveit_commander.roscpp_shutdown()
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      self.base_pub.publish(twist)

      ## END_TUTORIAL

      print "============ STOPPING"


if __name__=='__main__':
  try:
    my_node = move_traj_node()
  except rospy.ROSInterruptException:
    pass

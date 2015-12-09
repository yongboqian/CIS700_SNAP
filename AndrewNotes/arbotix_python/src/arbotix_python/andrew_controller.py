#!/usr/bin/env python

"""
  linear_controller.py - controller for a linear actuator with analog feedback
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy, actionlib

from joints import *
from controllers import *
from std_msgs.msg import Float64
from diagnostic_msgs.msg import *
from std_srvs.srv import *
from arbotix import *

from struct import unpack

class LinearJoint(Joint):
    def __init__(self, device, name):
        Joint.__init__(self, device, name)

        self.dirty = False
        self.position = 0.0                     # current position, as returned by feedback (meters)
        self.desired = 0.0                      # desired position (meters)
        self.desired_direction = 0.0
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()
        self.position_cmd = False         #used to turn on and off position control loop
        # TODO: load these from URDF
        self.min = rospy.get_param('~joints/'+name+'/min_position',0.0)
        self.max = rospy.get_param('~joints/'+name+'/max_position',0.5)
        self.max_speed = rospy.get_param('~joints/'+name+'/max_speed',250)
        self.linear_motor_id = rospy.get_param('~joints/'+name+'/servoid',6)
        self.high_pin = rospy.get_param('~joints/'+name+'/high_pin')#ensure the pin for limiting height is specified!
        self.low_pin = rospy.get_param('~joints/'+name+'/low_pin')
        self.speed_filter = rospy.get_param('~joints/'+name+'/speed_filter',2000)
        self.ticks_const = rospy.get_param('~joints/'+name+'/ticks_const')

        # calibration data {reading: position}
        self.cal = { -1: -1, 1: 1 }
        self.cal_raw = rospy.get_param('~joints/'+name+'/calibration_data', self.cal)
        self.cal = dict()
        for key, value in self.cal_raw.items():
            self.cal[int(key)] = value
        self.keys = sorted(self.cal.keys())

        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        rospy.Subscriber(name+'/command_direction', Float64, self.command_directionCb)
    def interpolate(self, frame):
        """ Get new output: 1 = increase position, -1 is decrease position. """
        if self.dirty:
            cmd = self.desired - self.position
            if self.device.fake: 
                self.position = self.desired
                self.dirty = False
                return None
            if cmd > 0.01:
                return 1
            elif cmd < -0.01:
                return -1
            else:
                self.dirty = False
                return 0
        else:
            return None

    def setCurrentFeedback(self, reading):
        if reading >= self.keys[0] and reading <= self.keys[-1]:
            last_angle = self.position
            self.position = self.readingToPosition(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logerr(self.name + ": feedback reading out of range")

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in raw_data format. """
        if position <= self.max and position >= self.min:
            self.desired = position
            self.dirty = True
        else:
            rospy.logerr(self.name + ": requested position is out of range: " + str(position))
        return None # TODO
    
    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        if self.dirty:
            msg.message = "Moving"
        else:
            msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg
    def command_directionCb(self, req):
        """ Float64 style command input for direction -1.0 is down, 1.0 is up and 0.0 stop. """
        self.dirty = True
        if req.data == 1.0:
            direction = 1.0
        elif req.data == 0.0:
            direction = 0.0
        elif req.data == -1.0:
            direction = -1.0
        else:
            rospy.loginfo("Bad input direction to linear actuator.Seting direction to 0.0")
            direction = 0.0
            
        self.desired_direction = req.data 
        self.position_cmd = False # turn off position control   
    def commandCb(self, req):
        """ Float64 style command input. """
        rospy.loginfo("Recieved command for linear joint.")
        if self.device.fake:
            self.position = req.data
            #self.position_cmd = True # turn on position control
        else:
            if req.data <= self.max and req.data >= self.min:
                self.desired = req.data
                self.dirty = True
                self.position_cmd = True # turn on position contro
            else:
                rospy.logerr(self.name + ": requested position is out of range: " + str(req))

    def readingToPosition(self, reading):
        low = 0
        while reading > self.keys[low+1]:
            low += 1
        high = len(self.keys) - 1
        while reading < self.keys[high-1]:
            high += -1
        x = self.keys[high] - self.keys[low]
        y = self.cal[self.keys[high]] - self.cal[self.keys[low]]
        x1 = reading - self.keys[low]
        y1 = y * ( float(x1)/float(x) )
        return self.cal[self.keys[low]] + y1


class andrew_controller(Controller):
    """ A controller for a linear actuator, without absolute encoder. """
    POSITION_L  = 100
    POSITION_H  = 101
    DIRECTION   = 102

    def __init__(self, device, name):
        Controller.__init__(self, device, name) #goes into controller function?
        #import pdb; pdb.set_trace()
        self.pause = True
        #self.position_cmd = False              #used to turn on and off position control loop
        self.last = 0
        self.before = 0.0
        self.last_reading = 0
        self.dirty = False
        self.last_position = 0.0                #measured in meters
        self.last_speed = 0.0                   #measured in ticks
        self.tick_counter = 0.0                 # counts the ticks the elevator servo has moved
        self.position = 0.0
        self.desired = 0.0                      # desired position (meters)
        self.velocity = 0.0                     # moving speed
        # set servo to wheel mode and set speed limits?
        #load pitch and height information?

        self.delta = rospy.Duration(1.0/rospy.get_param('~controllers/'+name+'/rate', 10.0))
        self.next = rospy.Time.now() + self.delta
        self.joint = device.joints[rospy.get_param('~controllers/'+name+'/joint')[0]]#i added the[0] to prevent a list error i was getting
        #enable wheel mode (may not be nessesary)
        if not self.fake:
            #set last to to current to init
            self.last_tick = self.device.getPosition(self.joint.linear_motor_id)
            self.device.enableWheelMode(self.joint.linear_motor_id)
            
        
        #pins and position and speed limits are found in self.joint.<see LinearJoint above for definitions in the init>
        #rospy.Subscriber(name+'/command', Float64, self.commandCb)#already specified by LinearJoint
        rospy.Service(name+'/zero', Empty, self.zeroCb)
        rospy.loginfo("Started andrew_controller ("+self.name+").")
        
    def safe2move(self):
        high_limit = self.device.getDigital(self.joint.high_pin)
        low_limit  = self.device.getDigital(self.joint.low_pin)
        #calibration_error = self.position > self.joint.max or self.position < self.joint.min
        #if calibration_error:
        #    rospy.loginfo("estimated position beyond set limits. calibration error.")
        #    rospy.loginfo("estimated position is:("+str(self.position)+")")
        if high_limit == 0 or low_limit == 0: #or calibration_error:
            #stop motor imediately
            self.device.setWheelSpeed(self.joint.linear_motor_id,0,0)#this might not be the correct way to set the motor to stop
            rospy.loginfo("Linear limit reached. Stopping Motor.")
            self.joint.position_cmd = False # turn off position control loop
            #if hight_limit:
                #try to go back a bit
                #self.move(-1.0)
            #elif low_limit:
                #try to go up a bit
                #self.move(1.0)
            return 0
        else:
            #conintue
            return 1
            
    def startup(self):
        if not self.fake:
            self.zeroEncoder()
    def update(self):
        #check if it is safe to move
        safe2move = self.safe2move()
        now = rospy.Time.now()
        #import pdb; pdb.set_trace()
        if now > self.next:
            # read current position
            validcmd = self.joint.desired <= self.joint.max and self.joint.desired >= self.joint.min#this should call if limits or max are set.
            if validcmd and safe2move:
            #if self.joint.dirty: #dirty needs to be set to true for this to update
                if not self.fake:
                    try:#replace with code to calculate position
                        self.last_reading = self.device.getPosition(self.joint.linear_motor_id)
                        sensor_tick = self.device.getPosition(self.joint.linear_motor_id)
                        sensor_speed = self.device.getSpeed(self.joint.linear_motor_id)
                        self.position = self.estPoswSpeed( sensor_speed, sensor_tick, now)    
                        #rospy.loginfo(" ticks read:("+str(self.device.getPosition(self.joint.linear_motor_id))+") speed is: ("+str(self.device.getSpeed(self.joint.linear_motor_id))+")")
                        #rospy.loginfo("tick counter is:("+str(self.tick_counter)+")")
                        rospy.loginfo("estimated position is:("+str(self.position)+")")
                        pos_goal = self.joint.desired #from the commandCd in LinearJoint
                        if pos_goal > (self.position +0.01) and self.joint.position_cmd:#some tolenece 
                            #go up
                            self.joint.desired_direction = 1.0
                            #self.move(1.0)
                        elif pos_goal < (self.position -0.01) and self.joint.position_cmd: #some tolerance
                            #move down
                            #self.move(-1.0)
                            self.joint.desired_direction = -1.0
                        elif self.joint.position_cmd == False:
                            self.joint.position_cmd = False #basically do nothing
                        else: #stop
                            #stop
                            self.move(0.0)
                            self.joint.desired_direction = 0.0
                            self.joint.position_cmd = False
                            #publish that goal is achieved
                        #setSpeed of actuator
                        #direction = pos_goal-self.last_reading #if positive move up else move down
                        self.move(self.joint.desired_direction)
                        #need to make this safeer and more robust
                        self.joint.position = self.position#0.0 #change so this uses setCurrentFeedback
                        self.joint.velocity = 0.0
                        
                    except Exception as e:
                        print "linear error: ", e
                # update movement
                #output = self.joint.interpolate(1.0/self.delta.to_sec())
                #if self.last != output and not self.fake: 
                #    self.setSpeed(output)#self.setWheelSpeed(output)
                #    self.last = output
            self.next = now + self.delta
        
    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        if self.dirty:
            msg.message = "Moving"
        else:
            msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg
    
    def zeroEncoder(self, timeout=15.0):
        rospy.loginfo(self.name + ': zeroing encoder')
        downdirection = -1.0
        updirection = 1.0
        #move untill elevator hits a limit switch
        if self.safe2move():
            #move until lower limit is hit
            
            rospy.loginfo('Position not at zero. Moving elevator down...')
            while self.safe2move() and not rospy.is_shutdown():
                self.move(downdirection)
            #move just off of the switch
            while not self.safe2move() and not rospy.is_shutdown():
                self.moveOverride(updirection)
            self.move(0.0) #stop
            self.tick_counter = 0
            self.position = 0
            rospy.loginfo('Position at min. Exiting zero')
        elif not(self.device.getDigital(self.joint.high_pin)):
            #set height to max if upper limit is active
             #move just off of the switch
            while not self.safe2move() and not rospy.is_shutdown():
                self.moveOverride(downdirection)
            self.move(0.0) #stop
            self.tick_counter = self.joint.max/self.joint.ticks_const
            self.position = self.joint.max
            rospy.loginfo('Position at max. Exiting zero')
        elif not(self.device.getDigital(self.joint.low_pin)):
            #set height to lower limit is limit is active
             #move just off of the switch
            while not self.safe2move() and not rospy.is_shutdown():
                self.moveOverride(updirection)
            self.move(0.0) #stop
            self.tick_counter = 0
            self.position = 0
            rospy.loginfo('Position at min. Exiting zero')
    def zeroCb(self, msg):
        if not self.fake:
            self.zeroEncoder(15.0)
        return EmptyResponse()
    def estPoswSpeed(self, sensor_speed, sensor_tick, now):
        #check speed makes sense. if so use position or speed to add to position. filter out bad data
        #import pdb; pdb.set_trace()
        delta_time = 1/10.0#now-self.joint.last
        self.joint.last = now
        # I might need to check that the motor wants to be moved since the speed sensor skips sometimes even when the servo is 
        good_reading = (sensor_tick>= 1 and sensor_tick <600) or (sensor_tick >750 and sensor_tick <=1022)
        cmd_direction = self.joint.desired_direction
        cmded_move = cmd_direction > 0.001 or cmd_direction <-0.001 #is the robot cmded to move? this will help filter noise
        rospy.loginfo("cmd2move is:("+str(cmded_move)+")")
        if sensor_speed < self.joint.speed_filter and good_reading and cmded_move:
            #if the speed makes sense we will assume we can trust the sensor measure ments.
            if sensor_speed <1023:
                # if CW do this. ticks are expected to increase in general expect when the singularity is hit
                #this is negative so as to reduce the ticks
                delta_ticks = (sensor_speed)*delta_time# or (now-self.before)
            elif sensor_speed>=1023:
                #if CCW do this.ticks are expected to decrease in general expect when the singularity is hit
                delta_ticks = -(sensor_speed-1023)*delta_time# or (now-self.before)
            #update last_tick
            self.last_tick = sensor_tick
        elif not sensor_speed < 10 or not (sensor_speed<1030 and sensor_speed>1023) and cmded_move: #guess speed
            #estimate position from previous speed
            #if the speed makes sense we will assume we can trust the sensor measure ments.
            if self.last_speed <1023:
                # if CW do this. ticks are expected to increase in general expect when the singularity is hit
                #this is negative so as to reduce the ticks
                delta_ticks = (self.last_speed)*delta_time# or (now-self.before)
            elif self.last_speed>=1023:
                #if CCW do this.ticks are expected to decrease in general expect when the singularity is hit
                delta_ticks = -(self.last_speed-1023)*delta_time# or (now-self.before)
        else:
            #speed too slow
            delta_ticks = 0.0
        #update last position to be current position and return it as current position  
        self.tick_counter = self.tick_counter-delta_ticks#minus delta because I got the directions backwards
        self.last_position = self.tick_counter*self.joint.ticks_const
        delta_ticks = 0.0 #reset this varible
        if good_reading:
            #prevents bad reading from confusing the speed update
            self.last_speed = sensor_speed
        #update last speed      
        #import pdb; pdb.set_trace()
        return self.last_position
        
    def move(self, directionCmd):
        #for testiong direction commands
        if self.safe2move():
            if directionCmd == 1.0:
                self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,self.joint.max_speed)
            elif directionCmd== 0.0:
                #stop
                self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,0)
            elif directionCmd == -1.0:
                self.device.setWheelSpeed(self.joint.linear_motor_id,0,self.joint.max_speed)
            else:
                #stop
                self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,0)  
        #self.joint.desired_direction = 0.0 # reset direction to prevent drift
    def moveOverride(self, directionCmd):
        #for testiong direction commands
        if directionCmd == 1.0:
            self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,self.joint.max_speed)
        elif directionCmd== 0.0:
            #stop
            self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,0)
        elif directionCmd == -1.0:
            self.device.setWheelSpeed(self.joint.linear_motor_id,0,self.joint.max_speed)
        else:
            #stop
            self.device.setWheelSpeed(self.joint.linear_motor_id,1.0,0)        
        #self.joint.desired_direction = 0.0 # reset direction to prevent drift
    def estPosition(self, sensor_speed, sensor_tick):
        #check speed makes sense. if so use position or speed to add to position. filter out bad data
        if sensor_speed < self.joint.speed_filter and ((abs(sensor_tick - self.last_tick))<300 or(abs(sensor_tick - self.last_tick))>700):
            #if the speed makes sense we will assume we can trust the sensor measure ments.
            if sensor_speed <1023:
                # if CW do this. ticks are expected to increase in general expect when the singularity is hit
                delta_ticks = sensor_tick - self.last_tick
                if delta_ticks < 0:
                    #shift reference of last_tick to make sense with the sonsor tick position
                    self.last_tick = 1023 - self.last_tick
                    delta_ticks = sensor_tick - self.last_tick
            
            elif sensor_speed>=1023:
                #if CCW do this.ticks are expected to decrease in general expect when the singularity is hit
                delta_ticks = sensor_tick - self.last_tick
                if delta_ticks > 0:
                    #shift reference of last_tick to make sense with the sonsor tick position
                    self.last_tick = 1023 + self.last_tick
                    delta_ticks = sensor_tick - self.last_tick
            #update last_tick
            self.last_tick = sensor_tick
        else:
            #estimate position from previous position
            delta_ticks = 0.0 #not sure what to do here. i could mess things up by increaeing self.last)tick 
        #update last position to be current position and return it as current position  
        self.tick_counter = self.tick_counter+delta_ticks
        self.last_position = self.tick_counter*self.joint.ticks_const
        #update last speed      
        #import pdb; pdb.set_trace()
        return self.last_position
    def shutdown(self):
        if not self.fake:
            #self.setSpeed(0)
            self.device.setWheelSpeed(self.joint.linear_motor_id,0,0)


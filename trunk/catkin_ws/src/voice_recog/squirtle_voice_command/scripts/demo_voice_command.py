#!/usr/bin/env python

# THis node as a whole is a Finite-State-Machine with states listed below
# 1. Busy   -- means the robot is currently busy at other task
#              when busy, it can response(only) to summoning phrase and flip state to "wait for command"
#              if it was assigned new task it will ask if it should cancel current one
# 2. Free   -- means the robot is not doing anything
#              when free it can response(only) to summoning phrase and flip state to "wait for command"         
# 3. Wait for command (wfc)
#           -- means the robot is summoned, it will listen for command
#              when wait for command, if the command is not recognizable
#              state will return back to previous state(busy or free)  
# 4. mimic  -- means the robot is in mimic state which is a debugging state
#              in this state it will say anything it hears
#              terminate if it hears mimic mode termination phrase
#
#
# For voice command it has four different levels of recognition (from loose to strict)
# 1. Contains -- will recognize if the desired phrases is contained in input phrase
# 2. Exact    -- Will recognize only if the input phrase is exactly the same as desired one
# 3. Ask for confirmation
#             -- will ask for confirmation once recognized, for safety
# 4. Refuse   -- refuse such command and give a reason
# 
#
# Use multi thread to open up a action, track its state, once finished flip the state from busy to free
#
# TODO: upload a finite-state-machine graph to illustrate

import sys
import rospy
#from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import std_srvs.srv
import random
import threading
from geometry_msgs.msg import Twist


class Action_thread(threading.Thread):
    """this class is for multi-threading an action task"""

    def __init__(self, voice_command_obj,action_type,action_arg):
        # voice_command_obj: a voice command obj(the next class in this file)
        # action_type: string indicating action type, including "move", "turn", "spin", "goto"
        # action_arg: arguments specified for each action, e.g move is direction, turn is direction, goto is destination, spin is duration
        super(Action_thread, self).__init__()
        self.voice_command_obj = voice_command_obj
        self.action_type = action_type
        self.action_arg = action_arg

        # stop sign for ending a thread
        self._stop = threading.Event()

    def run(self):
        if self.action_type == "move":
            self.action_move(self.action_arg)

        elif self.action_type == "turn":
            self.action_turn(self.action_arg)

        elif self.action_type == "spin":
            self.action_spin(self.action_arg)

        elif self.action_type == "goto":
            self.action_goto(self.action_arg)

    def stop(self):
        # publish empty twist to stop it
        # set the _stop, to indicate stop, need to check the _stop state during running
        self._stop.set()
        self.voice_command_obj.say(["no problem, current task canceled"])
        self.cleanup()

    def cleanup(self):
        self.voice_command_obj.current_state = "free"
        self._stop.set()
        self.voice_command_obj.cmd_vel = Twist()
        self.voice_command_obj.cmd_vel_pub.publish(self.voice_command_obj.cmd_vel)

    def stopped(self):
        return self._stop.isSet()

    def action_move(self,direction):
        self.voice_command_obj.say(["ok sir preparing to move " + direction,"command received, going "+ direction])
        self.voice_command_obj.current_state = "busy"
        self.voice_command_obj.busy_task = "moving "+ direction
        
        # assign vel cmd msg
        self.voice_command_obj.cmd_vel = Twist()
        if direction == "forward":
            self.voice_command_obj.cmd_vel.linear.x = 0.3
        if direction == "backward":
            self.voice_command_obj.cmd_vel.linear.x = -0.3

        self.voice_command_obj.say(["moving " + direction + " for two second"])
        r = rospy.Rate(5)

        for x in xrange(1,10):
            if self.stopped():
                self.cleanup()
                return
            else:
                self.voice_command_obj.cmd_vel_pub.publish(self.voice_command_obj.cmd_vel)
                r.sleep()
        
        self.voice_command_obj.say([ self.voice_command_obj.busy_task + " complete, sir, return to free state"])
        self.cleanup()

    def action_turn(self,direction):
        self.voice_command_obj.say(["ok, sir, preparing to turn " + direction,"command received, turning "+ direction])
        self.voice_command_obj.current_state = "busy"
        self.voice_command_obj.busy_task = "turning "+ direction
       
        # assign empty cmd
        self.voice_command_obj.cmd_vel = Twist()
        if direction == "left":
            self.voice_command_obj.cmd_vel.angular.z = 0.3
        if direction == "right":
            self.voice_command_obj.cmd_vel.angular.z = -0.3

        self.voice_command_obj.say(["moving " + direction + " for thirty second"])
        r = rospy.Rate(5)   

        for x in xrange(1,150):
            if self.stopped():
                self.cleanup()
                return
            else:
                self.voice_command_obj.cmd_vel_pub.publish(self.voice_command_obj.cmd_vel)
                r.sleep()
        
        self.voice_command_obj.say([ self.voice_command_obj.busy_task + " complete, sir, return to free state"])
        self.cleanup()

    def action_goto(self,destination):
        self.voice_command_obj.say(["sorry, sir, this method is not implemented yet, going back to free state"])
        self.cleanup()

    def action_spin(self,duration):
        self.voice_command_obj.say(["sorry, sir, this method is not implemented yet, going back to free state"])
        self.cleanup()
        

class Demo_voice_command:
    """This script is for demo of voice command of Team4 squirtle"""
    def __init__(self):

        rospy.init_node('demo_voice_command')
        rospy.on_shutdown(self.cleanup)

        #create soundhandle for sound playing
        self.voice = 'voice_kal_diphone'
        #self.voice = 'voice_cmu_us_clb_arctic_clunits'
        #self.voice = 'voice_cmu_us_slt_arctic_clunits'
        # self.voice = 'voice_cmu_us_rms_arctic_clunits'
        self.soundhandle = SoundClient()
        rospy.sleep(1)

        # create clients for start and stop recognizer
        rospy.wait_for_service('/recognizer/stop')
        self.recognizer_stop = rospy.ServiceProxy('/recognizer/stop', std_srvs.srv.Empty)
        self.recognizer_start = rospy.ServiceProxy('/recognizer/start', std_srvs.srv.Empty)
        rospy.sleep(3)

        self.say(["Voice initial lize complete"])
        self.soundhandle.stopAll()


        self.current_state = "free"
        self.last_state = "free"
        self.busy_task = "free"

        # dictionary indicating phrases to command map
        self.phrases_to_command = {'summoning': ['turtlebot', 'snap'],
                                    'forward':['move forward','go forward'],
                                    'backward':['move backward','go backward'],
                                    'turn left': ['turn left'],
                                    'turn right': ['turn right'],
                                    'spin': ['spin','spinning'],
                                    'start mimic': ['start mimic','begin mimic','enter mimic'],
                                    'stop mimic': ['stop mimic','exit mimic'],
                                    'go to task': ['go to'],
                                    'introduce': ['introduce yourself'],
                                    'report state': ['are you busy','doing anything','what are you doing', "your task"],
                                    'nothing':['nothing','no thanks','dismiss'],
                                    'greetings':['how are you','hello turtlebot','greetings turtlebot','whats up'],
                                    
                                    # stop has been put to last priority, so that it will only recognized if it's nothing above
                                    'stop task':['stop', 'cancel']
                                    }

        # publish to turtlebot's velocity teleop topic
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size = 10)
        self.cmd_vel = Twist()

        # subcribe to speech recognizer from pocketsphinx
        rospy.Subscriber('/recognizer/output',String,self.receive_speech_callback)

        self.say(["squirtle start listening"])

    def say(self, strings_to_say):
        # choose randomly from giving strings to rendor diversity
        # note that strings_to_say is a LIST of string, do not forget to add []
        string_to_say = random.choice(strings_to_say)
        delay_length = len(string_to_say)/13.0

        self.recognizer_stop()
        self.soundhandle.say(string_to_say, self.voice)#, 1.0)
        rospy.sleep(delay_length)
        self.recognizer_start()


    def receive_speech_callback(self,msg):
        # print what it recognized


        rospy.loginfo(msg.data)
        command = self.parse_command(msg.data,"contain")

        # enter finite state machine state switcher
        self.state_switcher(msg,command)

        rospy.loginfo("now at " + self.current_state + " state")


    def state_switcher(self,msg,command):

        if self.current_state == "free":
            # free state
            # self.say(["i'm in free state"])
            if command == "summoning":
                # only summoning can activate turtlebot in this state
                self.say(["yes, sir, i'm waiting for your command","squirtle, standing by","waiting for command","yes, sir","what's your call, sir","I'm here sir", "ok, give me a task"])
                self.current_state = "wfc"
                self.last_state = "free"


        elif self.current_state == "busy":
            # busy state
            if command == "summoning":
                # only summoning can activate turtlebot in this state
                self.say(["yes, sir, i'm doing " + self.busy_task + ", what's your command"])
                self.current_state = "wfc"
                self.last_state = "busy"


        elif self.current_state == "wfc":
            # state of waiting for command, accepting command in this state
            if command == "forward":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"move","forward")
                self.thread1.start()

            elif command == "backward":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"move","backward")
                self.thread1.start()

            elif command == "turn left":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"turn","left")
                self.thread1.start()

            elif command == "turn right":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"turn","right")
                self.thread1.start()

            elif command == "spin":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"spin","forever")
                self.thread1.start()

            elif command == "go to task":
                try:
                    if not self.thread1.stopped():
                        self.thread1.stop()
                except Exception, e:
                    pass
                self.thread1 = Action_thread(self,"goto","some place")
                self.thread1.start()

            elif command == "introduce":
                self.say(['hello, everyone, my name is squirtle, from team for, this is my voice command demo, i will execute some easy tasks in this demo to show my voice architecture'])
                self.current_state = self.last_state

            elif command == 'stop task':
                try:
                    if self.thread1.stopped():
                        self.say(['task do not exist, sir'])
                    else:
                        self.thread1.stop()
                    self.current_state = "free"
                except Exception, e:
                    self.say(['task do not exist, sir'])

            elif command == "nothing":
                self.say(["ok, sir, good luck","sure, i will keep standing by","fine, call me when you need","any time, sir","ok, sir","as you wish"])
                self.current_state = self.last_state

            elif command == 'start mimic':
                self.say(["yes, sir, entering debugging mode, i will repeat what you say"])                
                self.say(["note that i will only say what i think i hear, sir"])
                self.current_state = "mimic"
            else:
                self.say(["sorry, i can't recognize your command","no command received","I can't hear you sir","back to previous state","i can't recognize","I'm not sure, sir","negative"])
                self.current_state = self.last_state

        elif self.current_state == "mimic":
            # mimic state for debugging voice, will repeat what it hear
            if command == "stop mimic" or command == "stop task":
                self.say(["yes, sir, exiting mimic state, now i'm at free state"])
                self.current_state = "free"
            else:
                self.say(["you are saying "+msg.data])


        else:
            self.say(["sir, my state is encountering an error, please check your code"])


    def parse_command(self,input_phrase,match_method):
        # this method is to convert an input string and output a command
        # has two method, 'exact' for matching exactly, 'contain' for containing phrases(loose) 
        # returned command type:
        # "error" -- indicating error
        # "no command" -- indicating no command found
        # other command -- check the keys of self.phrases_to_command 
        if match_method == "exact":
            for (command, keywords) in self.phrases_to_command.iteritems():
                for word in keywords:
                    if input_phrase == word:
                        return command

        elif match_method == "contain":
            for (command, keywords) in self.phrases_to_command.iteritems():
                for word in keywords:
                    if input_phrase.find(word) > -1:
                        return command
        else:
            self.say("sir, command parser is encountering error, invalid matching method")
            return "error"

        return "no command"

    def cleanup(self):
        # cleanup to ensure a decent shutdown
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down Demo_voice_command node...")

if __name__=="__main__":
    try:
        Demo_voice_command()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice command finished")




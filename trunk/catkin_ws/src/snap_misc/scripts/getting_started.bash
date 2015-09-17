#!/bin/bash

byobu new-session -d -s getting_started 'roscore'
byobu rename-window core

byobu new-window
byobu send-keys 'roslaunch turtlebot_bringup minimal.launch' 'C-m'
byobu rename-window minimal

byobu new-window
byobu send-keys 'roslaunch turtlebot_bringup 3dsensor.launch' 'C-m'
byobu rename-window 3dsensor


byobu

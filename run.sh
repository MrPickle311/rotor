#!/bin/bash
source /home/ubuntu/oze/6_rotor_antena/devel/setup.bash

gnome-terminal --window-with-profile=kss -e 'roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600'
sleep 15s
gnome-terminal --window-with-profile=kss -e 'rosservice call /mavros/set_stream_rate 0 10 1'
sleep 2s

#gnome-terminal --window-with-profile=kss -e 'roscore'
gnome-terminal --window-with-profile=kss -e 'rosrun rotor rotor.py'
gnome-terminal --window-with-profile=kss -e 'rosrun http_core http_client.py'
gnome-terminal --window-with-profile=kss -e 'rosrun drone_communication drone_communication.py'


sleep 5s
gnome-terminal --window-with-profile=kss -e 'rosrun http_core http_core.py'

# DEBUG STUFF
#rosservice call /send_mission  "/home/oze/mission_region0_route0_variant0.waypoint"

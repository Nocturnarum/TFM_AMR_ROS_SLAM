#!/bin/bash
# Para todos los nodos, roslaunch y roscore
source /opt/ros/melodic/setup.bash
rosnode kill -a
killall roslaunch rosmaster roscore

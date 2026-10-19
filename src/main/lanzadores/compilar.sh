#!/bin/bash
# Compila ~/catkin_ws; la terminal se cierra sola si va bien y queda abierta si hay error
gnome-terminal --title="Compilar" -- bash -c 'source /opt/ros/melodic/setup.bash; cd ~/catkin_ws && catkin_make && exit 0; echo; echo "ERROR en la compilación (ver arriba)"; exec bash'

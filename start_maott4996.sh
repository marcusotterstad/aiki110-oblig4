#!/bin/bash

LOG=~/maott4996.log

source /opt/ros/humble/install/setup.bash
source ~/ros2/install/setup.bash
ros2 run maott4996_bryter brytertjeneste >> $LOG 2>&1 &
ros2 run maott4996_bryter brytermonitor >> $LOG 2>&1 &
ros2 run maott4996_tilstand lysdiodenode >> $LOG 2>&1 &
ros2 run maott4996_tilstand bevegelsenode >> $LOG 2>&1 &
ros2 run maott4996_motor motornode >> $LOG 2>&1 &

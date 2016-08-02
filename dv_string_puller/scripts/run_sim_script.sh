#!/bin/bash

roslaunch davinci_gazebo sticky_davinci_gazebo.launch &

rosrun davinci_traj_streamer davinci_traj_interpolator_as & 

read A

killall davinci_traj_interpolator_as
killall roslaunch
#Note that this is NOT by any means safe- cancelling this script WILL kill all roslaunch processes on the computer, including those unrelated to you- use this ONLY if you are running in isolation.

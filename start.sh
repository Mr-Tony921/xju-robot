#!/usr/bin/bash

sudo apt install ros-noetic-mbf-msgs
sudo apt install ros-noetic-move-base-msgs
sudo apt install ros-noetic-tf2-msgs

cd .. # current directory is ~/YOUR_ws/src/xju-robot-noetic

git clone https://github.com/ros/geometry2.git
git clone https://github.com/Mr-Tony921/navigation.git

cd ..
catkin_make

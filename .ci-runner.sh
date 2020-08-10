#!/usr/bin/env bash
git checkout melodic-devel
rm -rf ~/catkin_ws/src/xbot
cp -r ../xbot ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make

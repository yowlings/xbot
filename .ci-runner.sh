#!/usr/bin/env bash
cd ~/catkin_ws/src/
rm -rf xbot
git clone -b melodic-devel git@yt.droid.ac.cn:xbot-u/xbot.git
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make
cd ~/catkin_ws/src/xbot/
echo "[remote \"github\"]
	url = git@github.com:DroidAITech/xbot.git
	fetch = +refs/heads/*:refs/remotes/origin/*" >> .git/config
git push github melodic-devel

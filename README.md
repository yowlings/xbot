# xbot ROS Packages

## Overview

Xbot is a double wheeled mobile robot which is suitable for most of common sensors and hardware such as Microsoft's Kinect and Asus' Xtion Pro and RPlidar. Users can easily integrate their own customized hardware and applications to the development platform by using ROS and the related series tutorials. For more details about xbot robot, please visit [http://robots.ros.org/xbot/](http://robots.ros.org/xbot/)

![image](xbot.jpg)

## Packages
This git repo contains 4 ROS packages for making up a complete xbot ROS bringup. They are xbot_driver, xbot_dock_drive, xbot_ftdi, xbot_keyop, xbot_msgs,xbot_node and xbot_safety_controller.

### xbot_driver
xbot_driver is the basic package of xbot_bringup. It provides the serial communication with the xbot move base including get sensors data and post the command data from navigation algorithms. 
There ars some test tools such as simple_keyop for you to debug your driver program. You can test if the received data is as expected or transfer the move command to see if it is moving as what you want.
xbot_driver is also a pure c++ program which means that you can run it as a c++ program or you can choose the ROS way to execute it.

### xbot_description

This package gives the urdf and 3d model of xbot robot, it descrips the relation of every body joint so the TF node could transform their coordinate.

### xbot_msgs
All of the message types that xbot publishes and subscribes. You can define your own message types in this package.

### xbot_node
xbot_node is the bridge between xbot_driver and ROS. In this package, all the related subscribed topics and published topics is defined and realised. 

### xbot_bringup

The launch door of all package, it assambly or standalong give out the launch file of every function module.

## Usage
There are three ways to launch the xbot.
### xbot.launch
>roslaunch xbot_bringup xbot.launch

Driving xbot .
### rplidar.launch
>roslaunch xbot_bringup rplidar.launch

Driving rplidar sensor.
### realsense.launch
>roslaunch xbot_bringup realsense.launch

Driving realsense camera.

### xbot-u.launch

> roslaunch xbot_bringup xbot-u.launch

Driving xbot-u product softwares, including xbot, realsense, rplidar, face recog camera, web_video_server and rosbridge_server.




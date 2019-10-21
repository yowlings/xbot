cmake_minimum_required(VERSION 2.8.3)
project(xbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)



generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs sensor_msgs
 )


catkin_package(
  LIBRARIES xbot_bringup
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs sensor_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS  scripts/xbot_scan180.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


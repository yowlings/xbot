cmake_minimum_required(VERSION 2.8.3)
project(xbot_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  message_generation
)

# add_message_files(
#    DIRECTORY msg
#    FILES

# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs visualization_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES xbot_bringup
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs visualization_msgs
  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS  scripts/control_publisher.py scripts/xbot_driver.py scripts/xbot_driver.pyc scripts/xbot_ros.py scripts/xbot_scan180.py scripts/xbot_state_publisher.py scripts/xbot_state_subscriber.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


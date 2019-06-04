cmake_minimum_required(VERSION 2.8.3)
project(xbot_node)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)
find_package(catkin REQUIRED COMPONENTS rospy roscpp nodelet pluginlib tf angles
                                        geometry_msgs sensor_msgs nav_msgs std_msgs
                                        xbot_msgs xbot_driver
                                        ecl_exceptions ecl_sigslots ecl_streams ecl_threads)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES xbot_ros xbot_nodelet xbot_talker
   CATKIN_DEPENDS rospy roscpp nodelet pluginlib tf angles
                  geometry_msgs sensor_msgs nav_msgs std_msgs
                  xbot_msgs xbot_driver
                  ecl_exceptions ecl_sigslots ecl_streams ecl_threads
)



include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/getOdom2D.py
                 scripts/getYaw.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

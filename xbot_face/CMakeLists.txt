cmake_minimum_required(VERSION 2.8.3)
project(xbot_face)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
add_message_files(
  DIRECTORY msg
  FILES
  FaceResult.msg
)

generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xbot_face
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



set(OpenCV_DIR /usr/local/share/OpenCV) 
message(STATUS "    version: ${OpenCV_VERSION}") 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS script/face_recog.py script/face_register.py script/camera_image.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

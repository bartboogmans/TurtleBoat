cmake_minimum_required(VERSION 3.0.2)
project(nausbot_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geonav_transform
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nausbot_sim
#  CATKIN_DEPENDS rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/main.py scripts/tnmodel1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(turtleboat)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/main.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

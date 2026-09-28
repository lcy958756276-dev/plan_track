cmake_minimum_required(VERSION 3.0.2)
project(map_saver)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  hector_map_saver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

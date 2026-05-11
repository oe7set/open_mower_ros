cmake_minimum_required(VERSION 3.0.2)
project(mower_sessions_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mower_msgs
  xbot_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

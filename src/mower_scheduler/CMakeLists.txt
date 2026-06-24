cmake_minimum_required(VERSION 3.0.2)
project(mower_scheduler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  xbot_msgs
  xbot_mqtt
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  nodes/scheduler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

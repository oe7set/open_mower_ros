cmake_minimum_required(VERSION 3.0.2)
project(mower_telemetry_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  mower_msgs
  xbot_msgs
  xbot_rpc
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

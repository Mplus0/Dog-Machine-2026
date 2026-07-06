cmake_minimum_required(VERSION 3.0.2)
project(dog_arm_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/dog_arm_bridge_node.py
  scripts/dog_arm_task_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

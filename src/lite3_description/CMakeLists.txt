cmake_minimum_required(VERSION 3.0.2)
project(lite3_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  Lite3
  Lite3L
  Lite3P
  Lite3V
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

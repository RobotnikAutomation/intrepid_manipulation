cmake_minimum_required(VERSION 2.8.3)
project(intrepid_bringup)


find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
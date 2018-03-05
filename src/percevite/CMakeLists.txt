cmake_minimum_required(VERSION 2.8.3)
project(percevite)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_install()
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
cs_export()
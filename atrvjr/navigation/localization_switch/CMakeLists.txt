cmake_minimum_required(VERSION 2.8.3)
project(localization_switch)

find_package(catkin REQUIRED
 COMPONENTS
 amcl
 robot_localization
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
)

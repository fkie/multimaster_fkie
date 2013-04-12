cmake_minimum_required(VERSION 2.8.3)
project(master_discovery_fkie)
find_package(catkin REQUIRED)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  Capability.msg
)

## Generate services in the 'srv' folder
add_service_files(
  ListDescription.srv
  ListNodes.srv
  LoadLaunch.srv
  Task.srv
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   master_discovery_fkie  # Or other packages containing msgs
#)

#install(PROGRAMS scripts/myscript DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
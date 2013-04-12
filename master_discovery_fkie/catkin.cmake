cmake_minimum_required(VERSION 2.8.3)
project(master_discovery_fkie)
find_package(catkin REQUIRED)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  LinkState.msg
  LinkStatesStamped.msg
  MasterState.msg
  ROSMaster.msg
  SyncMasterInfo.msg
  SyncTopicInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DiscoverMasters.srv
  GetSyncInfo.srv
)


#install(PROGRAMS scripts/myscript DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
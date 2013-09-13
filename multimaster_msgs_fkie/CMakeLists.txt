cmake_minimum_required(VERSION 2.8.3)
project(multimaster_msgs_fkie)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Capability.msg
  LinkState.msg
  LinkStatesStamped.msg
  MasterState.msg
  ROSMaster.msg
  SyncMasterInfo.msg
  SyncServiceInfo.msg
  SyncTopicInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  DiscoverMasters.srv
  GetSyncInfo.srv
  ListDescription.srv
  ListNodes.srv
  LoadLaunch.srv
  Task.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

cmake_minimum_required(VERSION 2.8.3)
project(master_discovery_fkie)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  LinkState.msg
  LinkStatesStamped.msg
  MasterState.msg
  ROSMaster.msg
  SyncMasterInfo.msg
  SyncTopicInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  DiscoverMasters.srv
  GetSyncInfo.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(
    PROGRAMS 
        nodes/master_discovery
        nodes/zeroconf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

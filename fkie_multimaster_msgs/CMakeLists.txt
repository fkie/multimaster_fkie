cmake_minimum_required(VERSION 3.0.2)
project(fkie_multimaster_msgs)

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
  SyncServiceInfo.msg
  SyncTopicInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  DiscoverMasters.srv
  GetSyncInfo.srv
  LoadLaunch.srv
  Task.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CFG_EXTRAS version.cmake.in
    CATKIN_DEPENDS message_runtime std_msgs
)

# generate code for gRPC protocols
include(cmake/grpc_protoc.cmake)
generate_grpc(PROTO_FILES file launch monitor screen settings version)

install(
    DIRECTORY
    ${GRPC_GENERATED_SRC_DIR}
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
    DIRECTORY
    grpc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

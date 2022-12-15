cmake_minimum_required(VERSION 3.5)
project(fkie_multimaster_msgs)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "FKIE multimaster msgs is being built using AMENT.")
    message(STATUS "------------------------------------------")

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg2/DiscoveredState.msg"
      "msg2/Endpoint.msg"
      "msg2/Gid.msg"
      "msg2/NodeEntitiesInfo.msg"
      "msg2/ParticipantEntitiesInfo.msg"
      "msg2/Qos.msg"
      "msg2/TopicEntity.msg"
      "srv/LoadLaunch.srv"
      "srv/Task.srv"
      DEPENDENCIES builtin_interfaces
    )

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME})

    ament_package()
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )

    message(STATUS "------------------------------------------")
    message(STATUS "FKIE multimaster msgs is being built using CATKIN.")
    message(STATUS "------------------------------------------")

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

    catkin_python_setup()
    generate_messages(DEPENDENCIES std_msgs)

    catkin_package(
        CFG_EXTRAS version.cmake.in
        CATKIN_DEPENDS message_runtime std_msgs rospy
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

    # # Add folders to be run by python nosetests
    if(CATKIN_ENABLE_TESTING)
        add_subdirectory(${PROJECT_NAME}/tests)
    endif()

endif()

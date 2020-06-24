cmake_minimum_required(VERSION 3.0.2)
project(fkie_node_manager_daemon)

# python -m grpc_tools.protoc -Iprotos --python_out=./src/fkie_node_manager_daemon/. --grpc_python_out=./src/fkie_node_manager_daemon/. protos/launch_manager.proto 
# python -m grpc_tools.protoc -Iprotos --python_out=./src/fkie_node_manager_daemon/. --grpc_python_out=./src/fkie_node_manager_daemon/. protos/launch_manager_client.proto

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  fkie_multimaster_msgs
  roslaunch
  rospy
)

catkin_python_setup()
catkin_package()
# generate version files; cmake from multimaster_msgs
generate_version(TARGETS version_TARGETS)


#############
## Install ##
#############

catkin_install_python(
    PROGRAMS 
        nodes/node_manager_daemon
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY
    tests/resources
    tests/resources_alt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

install(
    PROGRAMS
        tests/test_grpc_server.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

# install generated version files; cmake from multimaster_msgs
if (DEFINED version_TARGETS)
    install(
        FILES
            ${version_TARGETS}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif()

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(node_manager_daemon_fkie)

# python -m grpc_tools.protoc -Iprotos --python_out=./src/node_manager_daemon_fkie/. --grpc_python_out=./src/node_manager_daemon_fkie/. protos/launch_manager.proto 
# python -m grpc_tools.protoc -Iprotos --python_out=./src/node_manager_daemon_fkie/. --grpc_python_out=./src/node_manager_daemon_fkie/. protos/launch_manager_client.proto

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
)

catkin_python_setup()
catkin_package(
#  CFG_EXTRAS
#  INCLUDE_DIRS include
#  LIBRARIES node_manager_daemon_fkie
#  CATKIN_DEPENDS roslaunch rospy
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(
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

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

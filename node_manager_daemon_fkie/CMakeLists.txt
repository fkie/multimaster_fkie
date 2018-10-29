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

# generate code for gRPC protocols
include(cmake/grpc_protoc.cmake)
generate_grpc()

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
## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

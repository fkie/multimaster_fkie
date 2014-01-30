cmake_minimum_required(VERSION 2.8.3)
project(master_sync_fkie)
find_package(catkin REQUIRED COMPONENTS multimaster_msgs_fkie master_discovery_fkie)

catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie)
catkin_python_setup()

install(
    PROGRAMS 
        nodes/master_sync
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

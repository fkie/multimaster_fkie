cmake_minimum_required(VERSION 2.8.3)
project(master_discovery_fkie)

find_package(catkin REQUIRED COMPONENTS std_srvs multimaster_msgs_fkie)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie)

install(
    PROGRAMS 
        nodes/master_discovery
        nodes/zeroconf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

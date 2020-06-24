cmake_minimum_required(VERSION 3.0.2)
project(fkie_master_discovery)

find_package(catkin REQUIRED COMPONENTS fkie_multimaster_msgs std_srvs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS fkie_multimaster_msgs std_srvs)

catkin_install_python(
    PROGRAMS
        nodes/master_discovery
        nodes/zeroconf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED multimaster_msgs_fkie master_discovery_fkie)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie master_discovery_fkie)

install(
    PROGRAMS 
        nodes/node_manager
        scripts/remote_nm.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    DIRECTORY 
        images
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(
   PROGRAMS 
        ./README.rst
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
cmake_minimum_required(VERSION 2.8.3)
project(default_cfg_fkie)
find_package(catkin REQUIRED COMPONENTS multimaster_msgs_fkie)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie roslaunch roslib rospy)

install(
    PROGRAMS 
        nodes/default_cfg
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

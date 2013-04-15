cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
    PROGRAMS 
        nodes/node_manager
        nodes/remote_nm.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

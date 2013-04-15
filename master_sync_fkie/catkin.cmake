cmake_minimum_required(VERSION 2.8.3)
project(master_sync_fkie)
find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
    PROGRAMS 
        nodes/master_sync
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

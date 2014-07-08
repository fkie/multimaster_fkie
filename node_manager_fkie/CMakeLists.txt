cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED COMPONENTS multimaster_msgs_fkie master_discovery_fkie)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie master_discovery_fkie)

install(
    PROGRAMS 
        nodes/node_manager
        nodes/dynamic_reconfigure
        scripts/remote_nm.py
        scripts/respawn
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    DIRECTORY 
        images
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(
   FILES 
        ./README.rst
        ./plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(
   FILES
        ./src/node_manager_fkie/MasterTab.ui
        ./src/node_manager_fkie/LaunchFilesDockWidget.ui
        ./src/node_manager_fkie/LogDockWidget.ui
        ./src/node_manager_fkie/PasswordInput.ui
        ./src/node_manager_fkie/MainWindow.ui
        ./src/node_manager_fkie/SettingsDockWidget.ui
        ./src/node_manager_fkie/GUI.qrc
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

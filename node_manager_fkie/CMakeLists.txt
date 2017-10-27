cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED COMPONENTS multimaster_msgs_fkie master_discovery_fkie diagnostic_msgs)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS multimaster_msgs_fkie master_discovery_fkie diagnostic_msgs)

install(
    PROGRAMS 
        nodes/node_manager
        nodes/dynamic_reconfigure
        nodes/script_runner.py
        scripts/remote_nm.py
        scripts/respawn
        scripts/reduced_nm.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    DIRECTORY 
        images
        rqt_icons
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
        ./src/node_manager_fkie/EchoDialog.ui
        ./src/node_manager_fkie/MasterTab.ui
        ./src/node_manager_fkie/LaunchFilesDockWidget.ui
        ./src/node_manager_fkie/LogDockWidget.ui
        ./src/node_manager_fkie/PasswordInput.ui
        ./src/node_manager_fkie/ProfileWidget.ui
        ./src/node_manager_fkie/MainWindow.ui
        ./src/node_manager_fkie/SettingsDockWidget.ui
        ./src/node_manager_fkie/TimeInput.ui
        ./src/node_manager_fkie/GUI.qrc
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
   FILES
        ./src/node_manager_fkie/editor/GraphDockWidget.ui
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/editor/
)


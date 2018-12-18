cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    master_discovery_fkie
    multimaster_msgs_fkie
    node_manager_daemon_fkie
)

catkin_python_setup()
catkin_package(
#    CATKIN_DEPENDS
#        diagnostic_msgs
#        master_discovery_fkie
#        multimaster_msgs_fkie
#        node_manager_daemon_fkie
)

# generate code for gRPC protocols
include(cmake/version.cmake)
generate_version()
include(cmake/build_launcher.cmake)
build_launcher()

install(
    PROGRAMS 
        nodes/dynamic_reconfigure
        nodes/node_manager
        nodes/script_runner.py
        scripts/reduced_nm.py
        scripts/remote_nm.py
        scripts/respawn
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
        README.rst
        plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
   FILES
        src/${PROJECT_NAME}/EchoDialog.ui
        src/${PROJECT_NAME}/GUI.qrc
        src/${PROJECT_NAME}/LaunchFilesDockWidget.ui
        src/${PROJECT_NAME}/LogDockWidget.ui
        src/${PROJECT_NAME}/MainWindow.ui
        src/${PROJECT_NAME}/MasterTab.ui
        src/${PROJECT_NAME}/MessageFrame.ui
        src/${PROJECT_NAME}/PasswordInput.ui
        src/${PROJECT_NAME}/ProfileWidget.ui
        src/${PROJECT_NAME}/SettingsDockWidget.ui
        src/${PROJECT_NAME}/TimeInput.ui
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
   FILES
        src/${PROJECT_NAME}/editor/GraphDockWidget.ui
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/editor/
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (DEFINED VERSION_FILES)
    install(
        FILES
            ${VERSION_FILES}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif()

if (EXISTS ${NODE_MANAGER_LAUNCHER})
    # Install the launcher file.
    install(CODE "set(NODE_MANAGER_LAUNCHER \"${NODE_MANAGER_LAUNCHER}\")")
    install(SCRIPT cmake/install_launcher.cmake)
endif()

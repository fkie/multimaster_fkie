cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs master_discovery_fkie multimaster_msgs_fkie)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS diagnostic_msgs master_discovery_fkie multimaster_msgs_fkie)

# generate code for gRPC protocols
include(cmake/version.cmake)
generate_version()
include(cmake/build_launcher.cmake)
build_launcher()

install(
    PROGRAMS
        nodes/dynamic_reconfigure
        nodes/nm
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
        src/node_manager_fkie/EchoDialog.ui
        src/node_manager_fkie/GUI.qrc
        src/node_manager_fkie/LaunchFilesDockWidget.ui
        src/node_manager_fkie/LogDockWidget.ui
        src/node_manager_fkie/MainWindow.ui
        src/node_manager_fkie/MasterTab.ui
        src/node_manager_fkie/MessageFrame.ui
        src/node_manager_fkie/PasswordInput.ui
        src/node_manager_fkie/ProfileWidget.ui
        src/node_manager_fkie/SettingsDockWidget.ui
        src/node_manager_fkie/TimeInput.ui
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
   FILES
        src/node_manager_fkie/editor/GraphDockWidget.ui
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/editor/
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
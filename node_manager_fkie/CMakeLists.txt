cmake_minimum_required(VERSION 2.8.3)
project(node_manager_fkie)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs master_discovery_fkie multimaster_msgs_fkie)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS diagnostic_msgs master_discovery_fkie multimaster_msgs_fkie)

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

# On Ubuntu, create and install a desktop launcher for node_manager.
find_program(DFI desktop-file-install)
if (NOT DFI)
   message(STATUS "Skip installing Unity desktop launcher: desktop-file-install not found!")
endif()

if ((${CMAKE_SYSTEM_NAME} STREQUAL Linux) AND (EXISTS ${DFI}))
    # Define where to create the launcher file.
    set(NODE_MANAGER_LAUNCHER ${CMAKE_CURRENT_BINARY_DIR}/node_manager.desktop)

    # Create the launcher file.
    file(WRITE ${NODE_MANAGER_LAUNCHER}
        "[Desktop Entry]
Name=node_manager
Comment=GUI for managing running and configured ROS nodes on different hosts
Exec=/bin/sh -c \". ${CMAKE_INSTALL_PREFIX}/setup.sh; ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/node_manager\"
Icon=${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_icons/crystal_clear_prop_run.png
Terminal=false
Type=Application
Categories=Utility;Development;"
    )

    # Install the launcher file.
    install(CODE "set(NODE_MANAGER_LAUNCHER \"${NODE_MANAGER_LAUNCHER}\")")
    install(SCRIPT InstallLauncher.cmake)
endif()

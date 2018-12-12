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

# install a file with version tag
find_program(GIT git)
if (NOT GIT)
   message(STATUS "skip install version tag: git not found!")
endif()

if ((EXISTS ${GIT}) AND (EXISTS ${CMAKE_INSTALL_PREFIX}))
    # Define where to create the version file.
    set(VERSION_FILE "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/VERSION")
    execute_process (
        COMMAND bash -c "${GIT} describe --tags --dirty --always"
        OUTPUT_VARIABLE VERSION_TAG
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    # Create the version file.
    file(WRITE ${VERSION_FILE} ${VERSION_TAG})
    # Create the date file.
    set(DATE_FILE "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/DATE")
    execute_process (
        COMMAND bash -c "${GIT} show -s --format=%ci"
        OUTPUT_VARIABLE DATE_TAG
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    file(WRITE ${DATE_FILE} ${DATE_TAG})
endif()

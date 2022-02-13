cmake_minimum_required(VERSION 3.0.2)
project(fkie_node_manager)
find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    fkie_master_discovery
    fkie_multimaster_msgs
    fkie_node_manager_daemon
)

catkin_python_setup()
catkin_package(
#    CATKIN_DEPENDS
#        diagnostic_msgs
#        fkie_master_discovery
#        fkie_multimaster_msgs
#        fkie_node_manager_daemon
)

# generate version files; cmake from multimaster_msgs
generate_version(TARGETS version_TARGETS)

# generate launcher
include(cmake/build_launcher.cmake)
build_launcher()

catkin_install_python(
    PROGRAMS 
        nodes/dynamic_reconfigure
        nodes/node_manager
        nodes/script_runner.py
        scripts/remote_nm.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        scripts/respawn
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
   FILES
        README.rst
        plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY
        doc
        images
        launch
        rqt_icons
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY
        src/${PROJECT_NAME}/icons
        src/${PROJECT_NAME}/ui
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# install generated version files; cmake from multimaster_msgs
if (DEFINED version_TARGETS)
    install(
        FILES
            ${version_TARGETS}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif()

if (EXISTS "${NODE_MANAGER_LAUNCHER}")
    # Install the launcher file.
    install(CODE "set(NODE_MANAGER_LAUNCHER \"${NODE_MANAGER_LAUNCHER}\")")
    install(SCRIPT cmake/install_launcher.cmake)
endif()

cmake_minimum_required(VERSION 3.5)
project(fkie_node_manager)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "FKIE node_manager is being ignore using AMENT.")
    message(STATUS "------------------------------------------")
    # only to avoid warnings while build
    message(STATUS "install prefix: ${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
    message(STATUS "symlink install: ${CATKIN_SYMLINK_INSTALL}")

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )

    message(STATUS "------------------------------------------")
    message(STATUS "FKIE node_manager is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    find_package(catkin REQUIRED COMPONENTS
        diagnostic_msgs
        fkie_master_discovery
        fkie_multimaster_msgs
        fkie_multimaster_pylib
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
endif()
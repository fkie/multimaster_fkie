
macro(build_launcher)
    # On Ubuntu, create and install a desktop launcher for node_manager.
    find_program(DFI desktop-file-install)
    if (NOT DFI)
       message(STATUS "Skip installing Unity desktop launcher: desktop-file-install not found!")
    else()
        if (("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux") AND (EXISTS "${DFI}"))
            # Define where to create the launcher file.
            set(NODE_MANAGER_LAUNCHER ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/node_manager.desktop)

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
        endif()
    endif()

endmacro()
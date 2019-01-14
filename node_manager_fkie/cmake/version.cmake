include(CMakeParseArguments)

macro(generate_version)
    find_program(GIT git)
    if (NOT GIT)
        message(STATUS "git binary not found, VERSION and DATE files are not created")
    elseif (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
        message(STATUS "no .git repository found, VERSION and DATE files are not created")
    else(GIT)
        # install a file with version tag
        set(VERSION_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")
        set(VERSION_FILES "")
        set(VERSION_FILE "${VERSION_DIR}/VERSION")
        list(APPEND VERSION_FILES ${VERSION_FILE})
        # generate target for version file
        add_custom_command(
            OUTPUT ${VERSION_FILE}
            COMMAND bash -c "${GIT} describe --tags --dirty --always > ${VERSION_FILE}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "create version file ${VERSION_FILE}"
            VERBATIM
        )
        # generate target for date file
        set(DATE_FILE "${VERSION_DIR}/DATE")
        list(APPEND VERSION_FILES ${DATE_FILE})
        add_custom_command(
            OUTPUT ${DATE_FILE}
            COMMAND bash -c "${GIT} show -s --format=%ci > ${DATE_FILE}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "create date file ${DATE_FILE}"
            VERBATIM
        )

        # create ALL target
        add_custom_target(
            TARGET ALL
            DEPENDS ${VERSION_FILES}
            COMMENT "Generate version files"
        )
    endif()
endmacro()

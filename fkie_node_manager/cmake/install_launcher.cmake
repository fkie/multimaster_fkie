# Install and register the given launcher file.
message(STATUS "Installing Unity desktop launcher")
execute_process(COMMAND
        desktop-file-install --dir=$ENV{HOME}/.local/share/applications ${NODE_MANAGER_LAUNCHER}
    RESULT_VARIABLE 
        LAUNCHER_INSTALLATION_RESULT
)

# If an error occurred, print it.
if (NOT ${LAUNCHER_INSTALLATION_RESULT} EQUAL 0)
    message(AUTHOR_WARNING "Installing Unity desktop launcher failed")
endif (NOT ${LAUNCHER_INSTALLATION_RESULT} EQUAL 0)

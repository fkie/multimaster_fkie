# Install and register the given launcher file.
execute_process(COMMAND 
        desktop-file-install --dir=$ENV{HOME}/.local/share/applications ${NODE_MANAGER_LAUNCHER} 
    RESULT_VARIABLE 
        LAUNCHER_INSTALLATION_RESULT
)

# Print the result.
if (${LAUNCHER_INSTALLATION_RESULT} EQUAL 0)
    set(LAUNCHER_INSTALLATION_RESULT success)
endif (${LAUNCHER_INSTALLATION_RESULT} EQUAL 0)

if (${LAUNCHER_INSTALLATION_RESULT} EQUAL 1)
    set(LAUNCHER_INSTALLATION_RESULT failure)
endif (${LAUNCHER_INSTALLATION_RESULT} EQUAL 1)

message(STATUS "Installing Unity desktop launcher: ${LAUNCHER_INSTALLATION_RESULT}.")

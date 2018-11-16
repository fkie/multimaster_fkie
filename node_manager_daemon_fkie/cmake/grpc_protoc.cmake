include(CMakeParseArguments)

macro(generate_grpc)
    find_program(PYTHON python)
    # set(DEST_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/generated")
    set(DEST_DIR "${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/generated")
    set(PROTO_FILES file launch screen)
    set(GEN_PROTO_FILES "")
    set(ABS_PROTO_PATH "${PROJECT_SOURCE_DIR}/protos")
    # command to create generated directory
    add_custom_command(
        OUTPUT ${DEST_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_DIR}"
        COMMENT "create directory for generated gRPC code: ${CMAKE_CURRENT_SOURCE_DIR}"
    )
    message(STATUS "destination for gRPC generated code: ${DEST_DIR}")
    foreach(PROTO_FILE ${PROTO_FILES})
        set(ABS_PROTO_FILE "${ABS_PROTO_PATH}/${PROTO_FILE}.proto")
        list(APPEND GEN_PROTO_FILES ${DEST_DIR}/${PROTO_FILE}_pb2_grpc.py)
        message(STATUS "generate gRPC code from ${ABS_PROTO_FILE}")
        add_custom_command(
            OUTPUT ${DEST_DIR}/${PROTO_FILE}_pb2_grpc.py ${DEST_DIR}/${PROTO_FILE}_pb2.py
            COMMAND "${PYTHON}" -m grpc_tools.protoc -I${ABS_PROTO_PATH} --python_out=${DEST_DIR}/. --grpc_python_out=${DEST_DIR}/. ${ABS_PROTO_FILE}
            DEPENDS ${DEST_DIR} ${ABS_PROTO_FILE}
        )
    endforeach()
    # create init file
    add_custom_target(
        TARGET ALL
        COMMAND ${CMAKE_COMMAND} -E touch "${DEST_DIR}/__init__.py"
        DEPENDS ${GEN_PROTO_FILES}
        COMMENT "Create '__init__.py' for generated gRPC module"
    )
endmacro()

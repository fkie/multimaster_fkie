include(CMakeParseArguments)

macro(generate_grpc)
    # we need (for code generation) the root where the package lib goes to
    set(MM_ROOT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
    get_filename_component(DST_ROOT ${MM_ROOT} DIRECTORY)
    # and also the multimaster_fkie absolute path
    message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
    message(STATUS "DST_ROOT: ${DST_ROOT}")
    message(STATUS "MM_ROOT: ${MM_ROOT}")
    set(GRPC_GENERATED_SRC_DIR "${DST_ROOT}/${PROJECT_NAME}/grpc")
    cmake_parse_arguments(proto_arg "" "" "PROTO_FILES" ${ARGN})
    message(STATUS "gRPC proto files: ${proto_arg_PROTO_FILES}")
    set(GRPC_GENERATED_SOURCES "")
    set(SRC_PROTO_PATH "${PROJECT_SOURCE_DIR}/grpc")
    # command to create generated directory
    add_custom_command(
        OUTPUT ${GRPC_GENERATED_SRC_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${GRPC_GENERATED_SRC_DIR}"
        COMMENT "create directory for generated gRPC code: ${CMAKE_CURRENT_SOURCE_DIR}"
    )
    message(STATUS "destination for gRPC generated code: ${GRPC_GENERATED_SRC_DIR}")
    foreach(PROTO_FILE ${proto_arg_PROTO_FILES})
        set(ABS_PROTO_FILE "${SRC_PROTO_PATH}/${PROTO_FILE}.proto")
        set(ABS_BIN_PROTO_FILE "${MM_ROOT}/grpc/${PROTO_FILE}.proto")
        add_custom_command(
            OUTPUT ${ABS_BIN_PROTO_FILE}
            COMMAND ${CMAKE_COMMAND} -E copy
                ${ABS_PROTO_FILE}
                ${ABS_BIN_PROTO_FILE}
            DEPENDS ${ABS_PROTO_FILE}
        )
        list(APPEND GRPC_GENERATED_SOURCES ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2.py)
        list(APPEND GRPC_GENERATED_SOURCES ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2_grpc.py)
        message(STATUS "generate gRPC code from ${ABS_PROTO_FILE}")
        add_custom_command(
            OUTPUT ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2.py
            COMMAND "${PYTHON_EXECUTABLE}" -m grpc_tools.protoc -I${DST_ROOT} --python_out=${DST_ROOT} ${ABS_BIN_PROTO_FILE}
            DEPENDS ${GRPC_GENERATED_SRC_DIR} ${ABS_BIN_PROTO_FILE}
        )
        add_custom_command(
            OUTPUT ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2_grpc.py 
            COMMAND "${PYTHON_EXECUTABLE}" -m grpc_tools.protoc -I${DST_ROOT} --grpc_python_out=${DST_ROOT} ${ABS_BIN_PROTO_FILE}
            DEPENDS ${GRPC_GENERATED_SRC_DIR} ${ABS_BIN_PROTO_FILE} ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2.py
        )
    endforeach()
    # create init file
    add_custom_target(
        ${PROJECT_NAME}_GRPC ALL
        DEPENDS ${GRPC_GENERATED_SOURCES}
        COMMAND ${CMAKE_COMMAND} -E touch "${GRPC_GENERATED_SRC_DIR}/__init__.py"
        COMMENT "Create '__init__.py' for generated gRPC module"
    )
endmacro()

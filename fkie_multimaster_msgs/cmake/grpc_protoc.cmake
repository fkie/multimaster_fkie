include(CMakeParseArguments)

macro(generate_grpc)
    find_program(PYTHON python)
    set(GRPC_GENERATED_SRC_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/grpc")
    # set(GRPC_GENERATED_SRC_DIR "${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}")
    cmake_parse_arguments(proto_arg "" "" "PROTO_FILES" ${ARGN})
    message(STATUS "gRPC proto files: ${proto_arg_PROTO_FILES}")
    set(GRPC_GENERATED_SOURCES "")
    set(ABS_PROTO_PATH "${PROJECT_SOURCE_DIR}/protos")
    # command to create generated directory
    add_custom_command(
        OUTPUT ${GRPC_GENERATED_SRC_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${GRPC_GENERATED_SRC_DIR}"
        COMMENT "create directory for generated gRPC code: ${CMAKE_CURRENT_SOURCE_DIR}"
    )
    message(STATUS "destination for gRPC generated code: ${GRPC_GENERATED_SRC_DIR}")
    foreach(PROTO_FILE ${proto_arg_PROTO_FILES})
        set(ABS_PROTO_FILE "${ABS_PROTO_PATH}/${PROTO_FILE}.proto")
        list(APPEND GRPC_GENERATED_SOURCES ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2.py)
        list(APPEND GRPC_GENERATED_SOURCES ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2_grpc.py)
        message(STATUS "generate gRPC code from ${ABS_PROTO_FILE}")
        add_custom_command(
            OUTPUT ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2.py
            COMMAND "${PYTHON}" -m grpc_tools.protoc -I${ABS_PROTO_PATH} --python_out=${GRPC_GENERATED_SRC_DIR}/. ${ABS_PROTO_FILE}
            DEPENDS ${GRPC_GENERATED_SRC_DIR} ${ABS_PROTO_FILE}
        )
        add_custom_command(
            OUTPUT ${GRPC_GENERATED_SRC_DIR}/${PROTO_FILE}_pb2_grpc.py 
            COMMAND "${PYTHON}" -m grpc_tools.protoc -I${ABS_PROTO_PATH} --grpc_python_out=${GRPC_GENERATED_SRC_DIR}/. ${ABS_PROTO_FILE}
            DEPENDS ${GRPC_GENERATED_SRC_DIR} ${ABS_PROTO_FILE}
        )
    endforeach()
    # create init file
    add_custom_target(
        TARGET ALL
        DEPENDS ${GRPC_GENERATED_SOURCES}
        COMMAND ${CMAKE_COMMAND} -E touch "${GRPC_GENERATED_SRC_DIR}/__init__.py"
        COMMENT "Create '__init__.py' for generated gRPC module"
    )
endmacro()

cmake_minimum_required(VERSION 3.5)
project(fkie_master_discovery)

#---- Add the subdirectory cmake ----
#set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

#---- Enable C++14 ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-Wpedantic)
endif()

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)
find_package(fkie_multimaster_msgs REQUIRED)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "FKIE Master Discovery is being built using AMENT.")
    message(STATUS "------------------------------------------")

    set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

    # find dependencies
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(rmw_dds_common REQUIRED)
    find_package(rmw_fastrtps_cpp REQUIRED)
    find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
    find_package(rmw_fastrtps_shared_cpp REQUIRED)
    find_package(lifecycle_msgs REQUIRED)

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )
    find_package(catkin REQUIRED COMPONENTS fkie_multimaster_pylib rosgraph roslib rospy std_srvs)
    find_package(GTest)

    message(STATUS "------------------------------------------")
    message(STATUS "FKIE Master Discovery is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    catkin_python_setup()
    catkin_package( CATKIN_DEPENDS fkie_multimaster_msgs fkie_multimaster_pylib rosgraph roslib rospy std_srvs )

endif()

if ( ament_cmake_FOUND )
    set(DEPS
        fkie_multimaster_msgs
        rclcpp_lifecycle rclcpp
        rmw_fastrtps_shared_cpp
        lifecycle_msgs
    )
    list(APPEND DEPS rmw_dds_common)
    list(APPEND DEPS rmw_fastrtps_cpp)
    list(APPEND DEPS rosidl_typesupport_fastrtps_cpp)
    include_directories(${catkin_INCLUDE_DIRS})
    add_executable(master_discovery
        src/eprosima_pariticipant_listener.cpp
        src/message_type_support.cpp
    )
    ament_target_dependencies(master_discovery ${DEPS})


elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

endif()

#############################################################
if(ament_cmake_FOUND)
    if(MSVC)
    else()
        target_compile_options(master_discovery PRIVATE
            -Wall -Wextra -Werror=return-type)
    endif()
    
    #find_package(ament_index_cpp REQUIRED)
    #ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp)
    #ament_export_dependencies(ament_index_cpp)

    set( MASTER_DISCOVERY_LIB_DESTINATION   lib )
    set( MASTER_DISCOVERY_INC_DESTINATION   include )
    set( MASTER_DISCOVERY_BIN_DESTINATION   bin )
    set( MASTER_DISCOVERY_SHARE_DESTINATION share/${PROJECT_NAME} )

    ament_export_libraries(${PROJECT_NAME})
    ament_package()
    install(TARGETS
        master_discovery
        DESTINATION lib/${PROJECT_NAME})
elseif(catkin_FOUND)
    set( MASTER_DISCOVERY_LIB_DESTINATION   ${CATKIN_PACKAGE_LIB_DESTINATION} )
    set( MASTER_DISCOVERY_INC_DESTINATION   ${CATKIN_GLOBAL_INCLUDE_DESTINATION} )
    set( MASTER_DISCOVERY_BIN_DESTINATION   ${CATKIN_GLOBAL_BIN_DESTINATION} )
    set( MASTER_DISCOVERY_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
    catkin_install_python(
        PROGRAMS
            nodes/master_discovery
            nodes/zeroconf
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else()
    set( MASTER_DISCOVERY_LIB_DESTINATION   lib )
    set( MASTER_DISCOVERY_INC_DESTINATION   include )
    set( MASTER_DISCOVERY_BIN_DESTINATION   bin )
    set( MASTER_DISCOVERY_SHARE_DESTINATION share )

endif()

message( STATUS "MASTER_DISCOVERY_LIB_DESTINATION:   ${MASTER_DISCOVERY_LIB_DESTINATION} " )
message( STATUS "MASTER_DISCOVERY_BIN_DESTINATION:   ${MASTER_DISCOVERY_BIN_DESTINATION} " )
message( STATUS "MASTER_DISCOVERY_SHARE_DESTINATION: ${MASTER_DISCOVERY_SHARE_DESTINATION} " )

install(
    DIRECTORY
    launch
    DESTINATION ${MASTER_DISCOVERY_SHARE_DESTINATION}
)

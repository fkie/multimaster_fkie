cmake_minimum_required(VERSION 3.5)
project(fkie_multimaster)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "FKIE multimaster is being built using AMENT.")
    message(STATUS "------------------------------------------")
    # only to avoid warnings while build
    message(STATUS "install prefix: ${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
    message(STATUS "symlink install: ${CATKIN_SYMLINK_INSTALL}")
    # find dependencies
    find_package(ament_cmake REQUIRED)
    ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )
    find_package(catkin REQUIRED )

    message(STATUS "------------------------------------------")
    message(STATUS "FKIE multimaster is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    catkin_package()

endif()


# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and

import os
import re
from fkie_multimaster_msgs.system.host import ros_host_suffix


SEP = '/'
PRIV_NAME = '~'
NM_DISCOVERY_NAMESPACE = '/_node_manager'
NM_DISCOVERY_NAME = f'discovery_{os.environ["ROS_DISTRO"]}_{ros_host_suffix()}'
EMPTY_PATTERN = re.compile('\b', re.I)
SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg',
                 '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']

PACKAGE_FILE = 'package.xml'

try:
    import rospkg
    LOG_PATH = rospkg.get_log_dir()
except ImportError:
    LOG_PATH = ''.join([os.environ.get('ROS_LOG_DIR'), os.path.sep]) if os.environ.get(
        'ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
''':var LOG_PATH: logging path where all screen configuration and log files are stored.'''

SETTINGS_PATH = os.path.expanduser('~/.config/ros.fkie/')

SCREEN = "/usr/bin/screen"
''':var SCREEN: Defines the path to screen binary.'''

try:
    import rospy
    SCREEN_SLASH_SEP = '_'
    '''this character is used to replace the slashes in ROS-Names for ROS1 nodes.'''
    RESPAWN_SCRIPT = 'rosrun fkie_node_manager_daemon respawn'
    ''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''
except ImportError:
    import rospkg
    SCREEN_SLASH_SEP = '_'
    '''this character is used to replace the slashes in ROS-Names for ROS2 nodes.'''
    RESPAWN_SCRIPT = 'ros2 run fkie_node_manager_daemon respawn'
    ''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''

GRPC_TIMEOUT = 15.0
''':var GRPC_TIMEOUT: timeout for connection to remote gRPC-server'''

GRPC_SERVER_PORT_OFFSET = 1010
''':var GRPC_SERVER_PORT_OFFSET: offset to the ROS-Master port.'''

NMD_DEFAULT_PORT = 11811
''':var NMD_DEFAULT_PORT: default port of node manager daemon.'''

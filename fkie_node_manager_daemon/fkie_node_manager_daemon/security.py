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
# limitations under the License.

from typing import Dict
from typing import List
from typing import Text
from typing import Tuple

from datetime import datetime
import os
import re
import sys

from ament_index_python import get_packages_with_prefixes
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from rclpy.node import Node
from xml.dom import minidom


DEFAULT_COMMON_NAME = '/node_manager'
_INITIALIZED = False
_STRICT_MODE_ENABLED = False
_CA_CERT = None
_NODE_MANAGER_PRIVATE_KEY = None
_NODE_MANAGER_CERT = None


# TODO: Support for both permissive and strict enforcement of security
# Participants with the security features enabled will not communicate with participants that don’t, but what should RCL do if one tries to launch a participant that has no discernable enclave with keys/permissions/etc.? It has two options:

# Permissive mode: Try to find security files, and if they can’t be found, launch the participant without enabling any security features. This is the default behavior.
# Strict mode: Try to find security files, and if they can’t be found, fail to run the participant.
# The type of mode desired can be specified by setting the ROS_SECURITY_STRATEGY environment variable to “Enforce” (case-sensitive) for strict mode, and anything else for permissive mode.

def init_keys(rosnode: [Node, None] = None) -> None:
    global _INITIALIZED
    if _INITIALIZED:
        return
    if 'ROS_SECURITY_ENABLE' in os.environ:
        if os.environ['ROS_SECURITY_ENABLE'] in ['true', 'True']:
            if 'ROS_SECURITY_KEYSTORE' in os.environ:
                global _CA_CERT
                global _NODE_MANAGER_PRIVATE_KEY
                global _NODE_MANAGER_CERT
                ros_security_keystore = os.environ['ROS_SECURITY_KEYSTORE']
                public_cert = os.path.join(
                    ros_security_keystore, 'ca.cert.pem')
                enclave_path = ros_security_keystore
                if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] > 'eloquent':
                    public_cert = os.path.join(
                        ros_security_keystore, 'public', 'ca.cert.pem')
                    enclave_path = os.path.join(
                        ros_security_keystore, 'enclaves')
                if os.path.exists(public_cert):
                    nm_cert = os.path.join(
                        enclave_path, 'node_manager', 'cert.pem')
                    nm_private_key = os.path.join(
                        enclave_path, 'node_manager', 'key.pem')
                    if os.path.exists(nm_private_key) and os.path.exists(nm_cert):
                        f = open(public_cert, 'rb')
                        _CA_CERT = f.read()
                        f = open(nm_cert, 'rb')
                        _NODE_MANAGER_CERT = f.read()
                        f = open(nm_private_key, 'rb')
                        _NODE_MANAGER_PRIVATE_KEY = f.read()
                    else:
                        if 'ROS_SECURITY_STRATEGY' in os.environ and os.environ['ROS_SECURITY_STRATEGY'] == 'Enforce':
                            global _STRICT_MODE_ENABLED
                            _STRICT_MODE_ENABLED = True
                        if rosnode is not None:
                            rosnode.get_logger().warn('Security keys (cert.pem, key.pem) for node manager in %s not found!' %
                                                      (os.path.dirname(nm_cert)))
                            if _STRICT_MODE_ENABLED:
                                rosnode.get_logger().warn('Security strict mode enabled!')
            elif rosnode is not None:
                rosnode.get_logger().warn(
                    'ROS_SECURITY_ENABLE is True, but ROS_SECURITY_KEYSTORE is not set!')
    _INITIALIZED = True


def has_keys() -> bool:
    global _CA_CERT
    global _NODE_MANAGER_PRIVATE_KEY
    global _NODE_MANAGER_CERT
    return all((_CA_CERT, _NODE_MANAGER_PRIVATE_KEY, _NODE_MANAGER_CERT))


def is_strict_mode() -> bool:
    global _STRICT_MODE_ENABLED
    return _STRICT_MODE_ENABLED


def get_ca_cert() -> [Text, None]:
    global _CA_CERT
    return _CA_CERT


def get_keys() -> Tuple[Text, Text]:
    global _NODE_MANAGER_PRIVATE_KEY
    global _NODE_MANAGER_CERT
    return (_NODE_MANAGER_PRIVATE_KEY, _NODE_MANAGER_CERT)


def get_node_manager_cert() -> [Text, None]:
    global _NODE_MANAGER_CERT
    return _NODE_MANAGER_CERT


def get_node_manager_private_key() -> [Text, None]:
    global _NODE_MANAGER_PRIVATE_KEY
    return _NODE_MANAGER_PRIVATE_KEY

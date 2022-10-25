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

import os

import roslib
import rospy
from urllib.parse import urlparse
import xmlrpc.client as xmlrpcclient
from fkie_multimaster_msgs.defines import GRPC_SERVER_PORT_OFFSET


MASTERURI = None


def from_ros() -> str:
    '''
    Returns the master URI depending on ROS distribution API.

    :return: ROS master URI
    :rtype: str
    :see: rosgraph.rosenv.get_master_uri() (fuerte)
    :see: roslib.rosenv.get_master_uri() (prior)
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            return roslib.rosenv.get_master_uri()
        else:
            import rosgraph
            return rosgraph.rosenv.get_master_uri()
    except Exception:
        return os.environ['ROS_MASTER_URI']


def from_master(from_env_on_error:bool = False) -> str:
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and
    returns it. The 'materuri' attribute will be set to the requested value.

    :return: ROS master URI
    :rtype: str or None
    '''
    global MASTERURI
    result = MASTERURI
    try:
        if MASTERURI is None:
            masteruri = from_ros()
            result = masteruri
            master = xmlrpcclient.ServerProxy(masteruri)
            code, _, MASTERURI = master.getUri(rospy.get_name())
            if code == 1:
                result = MASTERURI
    except Exception as err:
        if from_env_on_error:
            result = from_ros()
        else:
            raise err
    return result


def from_grpc(grpc_path: str) -> str:
    '''
    Determine ROS-Master uri from gRPC-URI by replacing the scheme and reducing the
    port by :const:`GRPC_SERVER_PORT_OFFSET`.

    :param str grpc_path: an URI with `grpc://` scheme.
    :return: ROS-Master URI
    :rtype: str
    :raise ValueError: if uri is not empty and does not start with 'grpc://'.
    '''
    if not grpc_path:
        return from_master(True)
    if not grpc_path.startswith('grpc://'):
        raise ValueError(
            "Invalid grpc path to get masteruri: %s; `grpc` scheme missed!" % grpc_path)
    o = urlparse(grpc_path)
    port = o.port
    if o.scheme == 'grpc':
        port -= GRPC_SERVER_PORT_OFFSET
    return "http://%s:%d/" % (o.hostname, port)
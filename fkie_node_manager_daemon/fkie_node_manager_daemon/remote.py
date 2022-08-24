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
import grpc
from rclpy.node import Node
import time

import fkie_node_manager_daemon.security as security


try:
    del os.environ['https_proxy']
except Exception:
    pass
try:
    del os.environ['http_proxy']
except Exception:
    pass


def open_channel(url, *, rosnode: [Node, None] = None):
    '''
    :param str url: the url to parse or hostname. If hostname the channel should be added before.
    :return: returns gRPC channel for given url.
    :rtype: grpc.Channel or None
    '''
    starttime = time.time()
    # create credentials
    security.init_keys(rosnode)
    if security.has_keys():
        (private_key, certificate_chain) = security.get_keys()
        credentials = grpc.ssl_channel_credentials(root_certificates=security.get_ca_cert(
        ), private_key=private_key, certificate_chain=certificate_chain)
        if rosnode is not None:
            rosnode.get_logger().debug("create secure channel to %s" % url)
        options = (('grpc.ssl_target_name_override',
                    security.DEFAULT_COMMON_NAME,),)
        channel = grpc.secure_channel(url, credentials, options=options)
    else:
        if rosnode is not None:
            rosnode.get_logger().debug("create insecure channel to %s" % url)
        channel = grpc.insecure_channel(url)
    if time.time() - starttime > 5.0:
        if rosnode is not None:
            rosnode.get_logger().warn("Open insecure gRPC channel took too long (%.3f sec)! Fix your network configuration!" %
                                      (time.time() - starttime))
    return channel

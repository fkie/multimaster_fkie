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

import fkie_node_manager_daemon.grpc_proto.version_pb2_grpc as vgrpc
import fkie_node_manager_daemon.grpc_proto.version_pb2 as vmsg
import fkie_node_manager_daemon as nmd
from . import version


class VersionServicer(vgrpc.VersionServiceServicer):

    def __init__(self):
        nmd.rosnode.get_logger().info("Create version servicer")
        vgrpc.VersionServiceServicer.__init__(self)
        self._version, self._date = version.detect_version(nmd.rosnode, 'fkie_node_manager_daemon')

    def GetVersion(self, request, context):
        reply = vmsg.Version()
        reply.version = self._version
        reply.date = self._date
        return reply

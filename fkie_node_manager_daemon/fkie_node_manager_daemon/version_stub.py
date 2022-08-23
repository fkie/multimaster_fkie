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
from . import settings


class VersionStub(object):

    def __init__(self, channel):
        self.vm_stub = vgrpc.VersionServiceStub(channel)

    def get_version(self):
        '''
        :return: a tuple of version and date
        :rtype: (str, str)
        '''
        request = vmsg.Empty()
        response = self.vm_stub.GetVersion(request, timeout=settings.GRPC_TIMEOUT)
        return (response.version, response.date)

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

import fkie_node_manager_daemon.grpc_proto.settings_pb2_grpc as sgrpc
import fkie_node_manager_daemon.grpc_proto.settings_pb2 as smsg
from . import settings


class SettingsStub(object):

    def __init__(self, channel):
        self.ss_stub = sgrpc.SettingsServiceStub(channel)

    def get_config(self, nslist=[]):
        '''
        :return: String representation for YAML
        :rtype: str
        '''
        request = smsg.Filter()
        request.nslist.extend(nslist)
        response = self.ss_stub.GetConfig(request, timeout=settings.GRPC_TIMEOUT)
        return response.data

    def set_config(self, data):
        '''
        :param str data: string representation of YAML file
        '''
        request = smsg.Yaml()
        request.data = data
        self.ss_stub.SetConfig(request, timeout=settings.GRPC_TIMEOUT)

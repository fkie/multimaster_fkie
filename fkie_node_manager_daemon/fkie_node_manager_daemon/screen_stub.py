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

import fkie_node_manager_daemon.grpc_proto.screen_pb2_grpc as sgrpc
import fkie_node_manager_daemon.grpc_proto.screen_pb2 as smsg
from . import settings


class ScreenStub(object):

    def __init__(self, channel):
        self.sm_stub = sgrpc.ScreenServiceStub(channel)

    def screens(self, nodename):
        '''
        :return: dictionary of screen name and corresponding ROS node name
        :rtype: {str: str}
        '''
        request = smsg.Screen(name='', node=nodename)
        response = self.sm_stub.GetScreens(request, timeout=settings.GRPC_TIMEOUT)
        return {screen.name: screen.node for screen in response.screens}

    def all_screens(self):
        '''
        :return: dictionary of screen name and corresponding ROS node name
        :rtype: {str: str}
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetAllScreens(request, timeout=settings.GRPC_TIMEOUT)
        return {screen.name: screen.node for screen in response.screens}

    def multiple_screens(self):
        '''
        :return: dictionary of corresponding ROS node name and list of screen sessions
        :rtype: {str: [str]}
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetMultipleScreens(request, timeout=settings.GRPC_TIMEOUT)
        result = {}
        for screen in response.screens:
            if screen.node not in result:
                result[screen.node] = []
            result[screen.node].append(screen.name)
        return result

    def rosclean(self):
        '''
        Removes the content of the log directory.
        '''
        request = smsg.Empty()
        _empty_response = self.sm_stub.RosClean(request, timeout=settings.GRPC_TIMEOUT)

    def delete_log(self, nodes):
        '''
        Removes log files for given nodes.
        :param [str] nodes: a list with names of nodes to remove log files
        '''
        request = smsg.Nodes()
        for node in nodes:
            request.nodes.append(node)
        _empty_response = self.sm_stub.DeleteLog(request, timeout=settings.GRPC_TIMEOUT)

    def log_dir_size(self):
        '''
        Determine the size of the ROS log directory.
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetLogDiskSize(request, timeout=settings.GRPC_TIMEOUT)
        return response.size

    def wipe_screens(self):
        request = smsg.Empty()
        _response = self.sm_stub.WipeScreens(request, timeout=settings.GRPC_TIMEOUT)

    def kill_nodes(self, nodes):
        '''
        Searches for screens of the given nodes and try to kill them by their PID
        :param [str] nodes: a list with names of nodes
        '''
        request = smsg.Nodes()
        for node in nodes:
            request.nodes.append(node)
        response = self.sm_stub.KillNodes(request, timeout=settings.GRPC_TIMEOUT)
        return response

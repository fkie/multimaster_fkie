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

from __future__ import division, absolute_import, print_function, unicode_literals

from fkie_node_manager_daemon.monitor import ros_msg
import fkie_node_manager_daemon.grpc_proto.monitor_pb2_grpc as mgrpc
import fkie_node_manager_daemon.grpc_proto.monitor_pb2 as mmsg
from . import settings


class MonitorStub(object):

    def __init__(self, channel):
        self.mm_stub = mgrpc.MonitorServiceStub(channel)

    def get_system_diagnostics(self, *, filter_level: list = [], filter_timestamp: float = 0):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Filter(timestamp=filter_timestamp, level=filter_level)
        response = self.mm_stub.GetSystemDiagnostics(
            request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_system_warnings(self):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Empty()
        response = self.mm_stub.GetSystemWarnings(
            request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_diagnostics(self, *, filter_level: list = [], filter_timestamp: float = 0):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Filter(timestamp=filter_timestamp, level=filter_level)
        response = self.mm_stub.GetDiagnostics(
            request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_warnings(self):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Empty()
        response = self.mm_stub.GetWarnings(
            request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def kill_process(self, pid):
        '''
        '''
        request = mmsg.Pid()
        request.pid = pid
        self.mm_stub.KillProcess(request, timeout=settings.GRPC_TIMEOUT)

    def set_time(self, timestamp: float):
        '''
        '''
        request = mmsg.Timestamp()
        request.timestamp = timestamp
        response = self.mm_stub.SetTime(request, timeout=settings.GRPC_TIMEOUT)
        return response.timestamp

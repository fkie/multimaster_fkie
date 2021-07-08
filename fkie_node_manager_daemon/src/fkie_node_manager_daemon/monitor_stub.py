# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



from fkie_node_manager_daemon.monitor import ros_msg
import fkie_multimaster_msgs.grpc.monitor_pb2_grpc as mgrpc
import fkie_multimaster_msgs.grpc.monitor_pb2 as mmsg
from . import settings


class MonitorStub(object):

    def __init__(self, channel):
        self.mm_stub = mgrpc.MonitorServiceStub(channel)

    def get_system_diagnostics(self, filter_level=0, filter_timestamp=0):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Filter(timestamp=filter_timestamp, level=filter_level)
        response = self.mm_stub.GetSystemDiagnostics(request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_system_warnings(self):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Empty()
        response = self.mm_stub.GetSystemWarnings(request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_diagnostics(self, filter_level=0, filter_timestamp=0):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Filter(timestamp=filter_timestamp, level=filter_level)
        response = self.mm_stub.GetDiagnostics(request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def get_warnings(self):
        '''
        :return: DiagnosticArray
        :rtype: DiagnosticArray
        '''
        request = mmsg.Empty()
        response = self.mm_stub.GetWarnings(request, timeout=settings.GRPC_TIMEOUT)
        return ros_msg(response)

    def kill_process(self, pid):
        '''
        '''
        request = mmsg.Pid()
        request.pid = pid
        self.mm_stub.KillProcess(request, timeout=settings.GRPC_TIMEOUT)

    def get_user(self):
        '''
        '''
        request = mmsg.Empty()
        response = self.mm_stub.GetUser(request, timeout=settings.GRPC_TIMEOUT)
        return response.user

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



import getpass
import os
import rospy
import signal
from fkie_node_manager_daemon.monitor import Service, grpc_msg
import fkie_multimaster_msgs.grpc.monitor_pb2_grpc as mgrpc
import fkie_multimaster_msgs.grpc.monitor_pb2 as mmsg


class MonitorServicer(mgrpc.MonitorServiceServicer):

    def __init__(self, settings):
        rospy.loginfo("Create monitor servicer")
        mgrpc.MonitorServiceServicer.__init__(self)
        self._monitor = Service(settings)

    def stop(self):
        self._monitor.stop()

    def GetSystemDiagnostics(self, request, context):
        rosmsg = self._monitor.get_system_diagnostics(request.level, request.timestamp)
        return grpc_msg(rosmsg)

    def GetSystemWarnings(self, request, context):
        rosmsg = self._monitor.get_system_diagnostics(2, 0)
        return grpc_msg(rosmsg)

    def GetDiagnostics(self, request, context):
        rosmsg = self._monitor.get_diagnostics(request.level, request.timestamp)
        return grpc_msg(rosmsg)

    def GetWarnings(self, request, context):
        rosmsg = self._monitor.get_diagnostics(2, 0)
        return grpc_msg(rosmsg)

    def KillProcess(self, request, context):
        os.kill(request.pid, signal.SIGKILL)
        reply = mmsg.Empty()
        return reply

    def GetUser(self, request, context):
        reply = mmsg.User()
        reply.user = getpass.getuser()
        return reply

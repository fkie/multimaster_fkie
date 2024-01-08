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
import signal
import json
import asyncio
from autobahn import wamp

from fkie_node_manager_daemon.monitor import Service, grpc_msg
import fkie_multimaster_msgs.grpc.monitor_pb2_grpc as mgrpc
import fkie_multimaster_msgs.grpc.monitor_pb2 as mmsg
from fkie_multimaster_pylib.crossbar.runtime_interface import DiagnosticArray
from fkie_multimaster_pylib.crossbar.runtime_interface import DiagnosticStatus
from fkie_multimaster_pylib.crossbar.runtime_interface import SystemEnvironment
from fkie_multimaster_pylib.crossbar.runtime_interface import SystemInformation
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.system.screen import ros_clean


class MonitorServicer(mgrpc.MonitorServiceServicer, CrossbarBaseSession):
    def __init__(
        self,
        settings,
        loop: asyncio.AbstractEventLoop,
        realm: str = "ros",
        port: int = 11911,
        test_env=False,
    ):
        Log.info("Create monitor servicer")
        mgrpc.MonitorServiceServicer.__init__(self)
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._monitor = Service(settings, self.diagnosticsCbPublisher)

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

    def _toCrossbarDiagnostics(self, rosmsg):
        cbMsg = DiagnosticArray(
            float(rosmsg.header.stamp.secs)
            + float(rosmsg.header.stamp.nsecs) / 1000000000.0, []
        )
        for sensor in rosmsg.status:
            values = []
            for v in sensor.values:
                values.append(DiagnosticStatus.KeyValue(v.key, v.value))
            status = DiagnosticStatus(
                sensor.level, sensor.name, sensor.message, sensor.hardware_id, values
            )
            cbMsg.status.append(status)
        return cbMsg

    def diagnosticsCbPublisher(self, rosmsg):
        self.publish_to(
            "ros.provider.diagnostics",
            json.dumps(self._toCrossbarDiagnostics(rosmsg), cls=SelfEncoder),
        )

    @wamp.register("ros.provider.get_diagnostics")
    def getDiagnostics(self) -> DiagnosticArray:
        Log.info("crossbar: get diagnostics")
        rosmsg = self._monitor.get_diagnostics(0, 0)
        # copy message to the crossbar structure
        return json.dumps(self._toCrossbarDiagnostics(rosmsg), cls=SelfEncoder)

    @wamp.register("ros.provider.get_system_info")
    def getSystemInfo(self) -> SystemInformation:
        Log.info("crossbar: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    @wamp.register("ros.provider.get_system_env")
    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("crossbar: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    @wamp.register("ros.provider.ros_clean_purge")
    def rosCleanPurge(self) -> {bool, str}:
        Log.info("crossbar: ros_clean_purge")
        result = False
        message = ''
        try:
            ros_clean()
            result = True
        except Exception as error:
            message = str(error)
        return json.dumps({result: result, message: message}, cls=SelfEncoder)

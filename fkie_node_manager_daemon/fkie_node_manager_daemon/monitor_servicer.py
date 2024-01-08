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

import asyncio
from autobahn import wamp
import json

from fkie_node_manager_daemon.monitor import Service

from fkie_multimaster_pylib.crossbar.runtime_interface import DiagnosticArray
from fkie_multimaster_pylib.crossbar.runtime_interface import DiagnosticStatus
from fkie_multimaster_pylib.crossbar.runtime_interface import SystemEnvironment
from fkie_multimaster_pylib.crossbar.runtime_interface import SystemInformation
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.logging.logging import Log


class MonitorServicer(CrossbarBaseSession):
    def __init__(
        self, settings, loop: asyncio.AbstractEventLoop, realm: str = "ros", port: int = 11911
    ):
        Log.info("Create monitor servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        self._monitor = Service(settings, self.diagnosticsCbPublisher)

    def stop(self):
        self._monitor.stop()
        self.shutdown()

    @wamp.register("ros.provider.get_system_info")
    def getSystemInfo(self) -> SystemInformation:
        Log.info("crossbar: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    @wamp.register("ros.provider.get_system_env")
    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("crossbar: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

    def _toCrossbarDiagnostics(self, rosmsg):
        cbMsg = DiagnosticArray(
            timestamp=float(rosmsg.header.stamp.sec)
            + float(rosmsg.header.stamp.nanosec) / 1000000000.0, status=[]
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

    @wamp.register("ros.provider.ros_clean_purge")
    def rosCleanPurge(self) -> {bool, str}:
        Log.info("crossbar: ros_clean_purge")
        result = False
        message = 'Not implemented'
        Log.warn("Not implemented: ros.provider.ros_clean_purge")
        return json.dumps({result: result, message: message}, cls=SelfEncoder)

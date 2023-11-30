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
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.crossbar.runtime_interface import DaemonVersion
from fkie_multimaster_pylib.logging.logging import Log
import fkie_node_manager_daemon as nmd
from . import version


class VersionServicer(CrossbarBaseSession):
    def __init__(
        self, loop: asyncio.AbstractEventLoop, realm: str = "ros", port: int = 11911
    ):
        Log.info("Create ROS2 version servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        self._version, self._date = version.detect_version(
            nmd.ros_node, "fkie_node_manager_daemon"
        )

    def stop(self):
        self.shutdown()

    @wamp.register("ros.daemon.get_version")
    def get_version(self) -> DaemonVersion:
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(f"{self._version}", f"{self._date}")
        return json.dumps(reply, cls=SelfEncoder)

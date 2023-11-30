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

from fkie_multimaster_pylib.crossbar.runtime_interface import SystemEnvironment
from fkie_multimaster_pylib.crossbar.runtime_interface import SystemInformation
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.logging.logging import Log


class MonitorServicer(CrossbarBaseSession):
    def __init__(
        self, loop: asyncio.AbstractEventLoop, realm: str = "ros", port: int = 11911
    ):
        Log.info("Create monitor servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)

    @wamp.register("ros.provider.get_system_info")
    def getSystemInfo(self) -> SystemInformation:
        Log.info("crossbar: get system info")
        return json.dumps(SystemInformation(), cls=SelfEncoder)

    @wamp.register("ros.provider.get_system_env")
    def getSystemEnv(self) -> SystemEnvironment:
        Log.info("crossbar: get system env")
        return json.dumps(SystemEnvironment(), cls=SelfEncoder)

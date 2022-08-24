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

import fkie_node_manager_daemon as nmd

from . import version
from .settings import Settings


class SettingsServicer():

    def __init__(self):
        nmd.rosnode.get_logger().info("Create settings servicer")
        self.settings = Settings(version=version.detect_version(
            nmd.rosnode, 'fkie_node_manager_daemon')[0])

    def GetConfig(self, request, context):
        pass
        # msg = smsg.Yaml()
        # msg.data = self.settings.yaml(request.nslist)
        # return msg

    def SetConfig(self, request, context):
        pass
        # self.settings.apply(request.data)
        # return smsg.Empty()

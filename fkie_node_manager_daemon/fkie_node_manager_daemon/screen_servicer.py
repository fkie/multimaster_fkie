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

import os
import signal
import fkie_node_manager_daemon.grpc_proto.screen_pb2_grpc as sgrpc
import fkie_node_manager_daemon.grpc_proto.screen_pb2 as smsg
import fkie_node_manager_daemon as nmd
from . import screen


class ScreenServicer(sgrpc.ScreenServiceServicer):

    def __init__(self):
        nmd.rosnode.get_logger().info("Create screen servicer")
        sgrpc.ScreenServiceServicer.__init__(self)
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)

    def stop(self):
        global IS_RUNNING
        IS_RUNNING = False

    def GetScreens(self, request, context):
        screens = screen.get_active_screens(request.node)
        reply = smsg.Screens()
        screen_objs = []
        for session_name, node_name in screens.items():
            screen_objs.append(smsg.Screen(name=session_name, node=node_name))
        reply.screens.extend(screen_objs)
        return reply

    def GetAllScreens(self, request, context):
        screens = screen.get_active_screens()
        reply = smsg.Screens()
        screen_objs = []
        for session_name, node_name in screens.items():
            screen_objs.append(smsg.Screen(name=session_name, node=node_name))
        reply.screens.extend(screen_objs)
        return reply

    def GetMultipleScreens(self, request, context):
        screens = screen.get_active_screens()
        reply = smsg.Screens()
        screen_objs = []
        node_names = []
        added_node_names = []
        for session_name, node_name in screens.items():
            if node_name not in node_names:
                node_names.append(node_name)
            elif node_name not in added_node_names:
                screen_objs.append(smsg.Screen(
                    name=session_name, node=node_name))
                added_node_names.append(node_name)
        # TODO currently we add only one session name. Add all!
        reply.screens.extend(screen_objs)
        return reply

    def RosClean(self, request, context):
        screen.rosclean()
        reply = smsg.Empty()
        return reply

    def DeleteLog(self, request, context):
        for nodename in request.nodes:
            screen.delete_log(nodename)
        reply = smsg.Empty()
        return reply

    def GetLogDiskSize(self, request, context):
        reply = smsg.DirSize()
        reply.size = screen.log_dir_size()
        return reply

    def WipeScreens(self, request, context):
        screen.wipe()
        reply = smsg.Empty()
        return reply

    def KillNodes(self, request, context):
        screens = screen.get_active_screens()
        reply = smsg.Screens()
        screen_objs = []
        for session_name, node_name in screens.items():
            if node_name in request.nodes:
                try:
                    ss = session_name.split('.')
                    os.kill(int(ss[0]), signal.SIGKILL)
                    screen_objs.append(smsg.Screen(
                        name=session_name, node=node_name))
                except Exception:
                    import traceback
                    print(traceback.format_exc())
        reply.screens.extend(screen_objs)
        screen.wipe()
        return reply

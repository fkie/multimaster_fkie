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



import rospy
import fkie_multimaster_msgs.grpc.screen_pb2_grpc as sgrpc
import fkie_multimaster_msgs.grpc.screen_pb2 as smsg
from . import screen


class ScreenServicer(sgrpc.ScreenServiceServicer):

    def __init__(self):
        rospy.loginfo("Create screen servicer")
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
                screen_objs.append(smsg.Screen(name=session_name, node=node_name))
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

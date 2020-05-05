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



import fkie_multimaster_msgs.grpc.screen_pb2_grpc as sgrpc
import fkie_multimaster_msgs.grpc.screen_pb2 as smsg
from . import settings


class ScreenStub(object):

    def __init__(self, channel):
        self.sm_stub = sgrpc.ScreenServiceStub(channel)

    def screens(self, nodename):
        '''
        :return: dictionary of screen name and corresponding ROS node name
        :rtype: {str: str}
        '''
        request = smsg.Screen(name='', node=nodename)
        response = self.sm_stub.GetScreens(request, timeout=settings.GRPC_TIMEOUT)
        return {screen.name: screen.node for screen in response.screens}

    def all_screens(self):
        '''
        :return: dictionary of screen name and corresponding ROS node name
        :rtype: {str: str}
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetAllScreens(request, timeout=settings.GRPC_TIMEOUT)
        return {screen.name: screen.node for screen in response.screens}

    def multiple_screens(self):
        '''
        :return: dictionary of corresponding ROS node name and list of screen sessions
        :rtype: {str: [str]}
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetMultipleScreens(request, timeout=settings.GRPC_TIMEOUT)
        result = {}
        for screen in response.screens:
            if screen.node not in result:
                result[screen.node] = []
            result[screen.node].append(screen.name)
        return result

    def rosclean(self):
        '''
        Removes the content of the log directory.
        '''
        request = smsg.Empty()
        _empty_response = self.sm_stub.RosClean(request, timeout=settings.GRPC_TIMEOUT)

    def delete_log(self, nodes):
        '''
        Removes log files for given nodes.
        :param [str] nodes: a list with names of nodes to remove log files
        '''
        request = smsg.Nodes()
        for node in nodes:
            request.nodes.append(node)
        _empty_response = self.sm_stub.DeleteLog(request, timeout=settings.GRPC_TIMEOUT)

    def log_dir_size(self):
        '''
        Determine the size of the ROS log directory.
        '''
        request = smsg.Empty()
        response = self.sm_stub.GetLogDiskSize(request, timeout=settings.GRPC_TIMEOUT)
        return response.size

    def wipe_screens(self):
        request = smsg.Empty()
        _response = self.sm_stub.WipeScreens(request, timeout=settings.GRPC_TIMEOUT)

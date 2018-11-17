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

from __future__ import print_function

import settings
import multimaster_msgs_fkie.grpc.screen_pb2_grpc as sgrpc
import multimaster_msgs_fkie.grpc.screen_pb2 as smsg


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

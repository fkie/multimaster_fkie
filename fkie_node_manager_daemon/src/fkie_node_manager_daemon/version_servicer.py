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

import asyncio
from autobahn import wamp
import json
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.crossbar.runtime_interface import DaemonVersion
import fkie_multimaster_msgs.grpc.version_pb2_grpc as vgrpc
import fkie_multimaster_msgs.grpc.version_pb2 as vmsg
from . import version
from fkie_multimaster_pylib.logging.logging import Log


class VersionServicer(vgrpc.VersionServiceServicer, CrossbarBaseSession):
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        realm: str = "ros",
        port: int = 11911,
        test_env=False,
    ):
        Log.info("Create version servicer")
        vgrpc.VersionServiceServicer.__init__(self)
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._version, self._date = version.detect_version("fkie_node_manager_daemon")

    def stop(self):
        self.shutdown()

    def GetVersion(self, request, context):
        reply = vmsg.Version()
        reply.version = self._version
        reply.date = self._date
        return reply

    @wamp.register("ros.daemon.get_version")
    def get_version(self) -> DaemonVersion:
        Log.info(f"{self.__class__.__name__}: get daemon version ")
        reply = DaemonVersion(self._version, self._date)
        return json.dumps(reply, cls=SelfEncoder)

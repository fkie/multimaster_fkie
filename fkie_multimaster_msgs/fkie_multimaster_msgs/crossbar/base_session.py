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


import time

from json import JSONEncoder
import json

# crossbar-io dependencies
import asyncio
from autobahn.wamp.types import ComponentConfig
from autobahn.asyncio.wamp import ApplicationSession, ApplicationRunner
from asyncio import coroutine

# ros
#import rospy

from .server import crossbar_start_server, CROSSBAR_PATH


class SelfEncoder(JSONEncoder):
    def default(self, obj):
        return obj.__dict__


class CrossbarBaseSession(ApplicationSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, test_env=False) -> None:
        self.port = port
        self.crossbar_loop = loop
        if test_env:
            return
        ApplicationSession.__init__(self, ComponentConfig(realm, {}))
        self.crossbar_connected = False
        self.crossbar_connecting = False
        self.uri = f"ws://localhost:{self.port}/ws"
        self.crossbar_runner = ApplicationRunner(self.uri, self.config.realm)
        task = asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)

    def onConnect(self):
        #rospy.loginfo("%s: autobahn connected" % self.__class__.__name__)
        print(f"{self.__class__.__name__}: autobahn connected")
        self.join(self.config.realm)

    def onDisconnect(self):
        #rospy.loginfo('%s: autobahn disconnected' % self.__class__.__name__)
        print(f"{self.__class__.__name__}: autobahn disconnected")
        self.crossbar_connected = False
        self.crossbar_connecting = False

    @coroutine
    def onJoin(self, details):
        res = yield from self.register(self)
        print(f"{self.__class__.__name__}: {len(res)} crossbar procedures registered!")

        # notify node changes to remote GUIs
        self.publish('ros.system.changed', "")

    async def crossbar_connect_async(self):
        self.crossbar_connected = False
        while not self.crossbar_connected:
            try:
                # try to connect to the crossbar server
                self.crossbar_connecting = True
                coro = await self.crossbar_runner.run(self, start_loop=False)
                (self.__crossbar_transport, self.__crossbar_protocol) = coro
                self.crossbar_connected = True
                self.crossbar_connecting = False
            except Exception as err:
                print(err)

                # try to start the crossbar server
                try:
                    config_path = crossbar_start_server(self.port)
                    print(
                        f"start crossbar server @ {self.uri} realm: {self.config.realm}, config: {config_path}")
                except:
                    import traceback
                    print(traceback.format_exc())

                self.crossbar_connecting = False
                self.crossbar_connected = False
                time.sleep(2.0)

    async def crossbar_connect(self) -> None:
        current_task = asyncio.current_task()
        if not self.crossbar_connecting:
            task = asyncio.create_task(self.crossbar_connect_async())
        else:
            task = current_task
        await asyncio.gather(task)

    def crossbar_reconnect(self):
        asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)

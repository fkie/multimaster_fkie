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

from typing import Any
from typing import Callable
from typing import Union

import time

import json

# crossbar-io dependencies
import asyncio
from autobahn.wamp.types import ComponentConfig
from autobahn.asyncio.wamp import ApplicationSession, ApplicationRunner
from autobahn.wamp.exception import TransportLost
from asyncio import coroutine


from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system.host import get_host_name
from .server import crossbar_start_server, CROSSBAR_PATH


class SelfEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        for key, value in vars(obj).items():
            if key[0] != '_':
                result[key] = value
        return result


class CrossbarBaseSession(ApplicationSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, test_env=False) -> None:
        self.port = port
        self.crossbar_loop = loop
        self._on_shutdown = False
        self._crossbar_subscriptions = []  # list of tuples (topic, handler)
        self._crossbar_failed_publications = {}  # (topic, message)
        self.crossbar_connected = False
        self.crossbar_connecting = False
        self.crossbar_registered = False
        self.uri = f"ws://localhost:{self.port}/ws"
        if test_env:
            return
        ApplicationSession.__init__(self, ComponentConfig(realm, {}))
        self.crossbar_runner = ApplicationRunner(self.uri, self.config.realm)
        self.uri = self.uri.replace('localhost', get_host_name())
        task = asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)

    '''
    Subscribes to a crossbar topic in async way.
    Subscribed topics in this way are subscribed after reconnection.
    '''

    def subscribe_to(self, topic: str, handler: Callable):
        self._crossbar_subscriptions.append((topic, handler))
        if self.crossbar_connected:
            asyncio.run_coroutine_threadsafe(
                self.subcribe_async(topic, handler), self.crossbar_loop)

    async def subcribe_async(self, topic: str, handler: Callable):
        try:
            await self.subscribe(handler, topic)
            Log.info(
                f"{self.__class__.__name__}: subscribed to crossbar topic 'ros.nodes.abort'")
        except Exception as e:
            Log.warn(
                f"{self.__class__.__name__}: could not subscribe to 'ros.nodes.abort': {0}".format(e))

    '''
    Publishes message to given topic without throw an exception on connection problems.
    Last failed message will be send on connect.
    '''

    def publish_to(self, topic: str, msg: Union[str, Any]):
        encoded_msg = msg
        if not isinstance(msg, str):
            encoded_msg = json.dumps(msg, cls=SelfEncoder)
        try:
            Log.debug(f"Publish '{topic}': {encoded_msg}")
            self.publish(topic, encoded_msg)
        except TransportLost as e:
            Log.info(f"add {topic}: {encoded_msg}")
            self._crossbar_failed_publications[topic] = encoded_msg
        except Exception:
            import traceback
            Log.warn(
                f"Error while publish to {topic}: {traceback.format_exc()}")

    def shutdown(self):
        self._on_shutdown = True
        self.disconnect()

    def onConnect(self):
        Log.info(f"{self.__class__.__name__}: autobahn connected")
        self.join(self.config.realm)
        for (topic, handler) in self._crossbar_subscriptions:
            asyncio.run_coroutine_threadsafe(
                self.subcribe_async(topic, handler), self.crossbar_loop)
        for topic, msg in self._crossbar_failed_publications.items():
            self.publish_to(topic, msg)
        self._crossbar_failed_publications.clear()

    def onDisconnect(self):
        Log.info(f"{self.__class__.__name__}: autobahn disconnected")
        self.crossbar_connected = False
        self.crossbar_connecting = False
        self.crossbar_registered = False
        if not self._on_shutdown:
            self.crossbar_reconnect()

    def onLeave(self, details):
        ApplicationSession.onLeave(self, details)
        Log.debug(f"{self.__class__.__name__}.onLeave: {details}")

    @coroutine
    def onJoin(self, details):
        res = yield from self.register(self)
        Log.info(
            f"{self.__class__.__name__}: {len(self._registrations)} crossbar procedures registered!")
        Log.info(f"{self.__class__.__name__}: list of registered uri's:")
        for _session_id, reg in self._registrations.items():
            Log.info(f"{self.__class__.__name__}:   {reg.procedure}")
        self.crossbar_registered = True

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
                Log.debug(f"{err}")

                # try to start the crossbar server
                try:
                    config_path = crossbar_start_server(self.port)
                    Log.info(
                        f"start crossbar server @ {self.uri} realm: {self.config.realm}, config: {config_path}")
                except:
                    import traceback
                    Log.debug(traceback.format_exc())

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
        Log.info(f"reconnect to crossbar @ {self.uri}")
        asyncio.run_coroutine_threadsafe(
            self.crossbar_connect(), self.crossbar_loop)

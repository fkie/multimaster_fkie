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
import os
import signal
import threading
import time
from typing import Dict, List
from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.runtime_interface import ScreenRepetitions
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system import screen
import fkie_node_manager_daemon as nmd


class ScreenServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911):
        Log.info("Create ROS2 screen servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        self._is_running = True
        self._multiple_screen_check_force_after = 10
        self._multiple_screen_do_check = True
        self._multiple_screen_thread = threading.Thread(
            target=self._check_multiple_screens, daemon=True)
        self._multiple_screen_thread.start()

    def stop(self):
        self._is_running = False
        self.shutdown()

    def _check_multiple_screens(self):
        last_check = 0
        notified_about_multiple = True
        while self._is_running:
            if self._multiple_screen_do_check or last_check >= self._multiple_screen_check_force_after:
                screen.wipe()
                self._multiple_screen_do_check = False
                screens = screen.get_active_screens()
                screen_dict: Dict[str, ScreenRepetitions] = {}
                for session_name, node_name in screens.items():
                    if node_name in screen_dict:
                        screen_dict[node_name].screens.append(session_name)
                    else:
                        screen_dict[node_name] = ScreenRepetitions(
                            name=node_name, screens=[session_name])
                crossbar_msg: List(ScreenRepetitions) = []
                for node_name, msg in screen_dict.items():
                    if len(msg.screens) > 1:
                        crossbar_msg.append(msg)
                if crossbar_msg or notified_about_multiple:
                    Log.debug(f"ros.screen.multiple with {len(crossbar_msg)} nodes with multiple screens.")
                    self.publish_to('ros.screen.multiple', crossbar_msg)
                    notified_about_multiple = len(crossbar_msg) > 0
                last_check = 0
            else:
                last_check += 1
            time.sleep(1.0)

    @wamp.register('ros.screen.kill_node')
    def kill_node(self, name: str) -> bool:
        Log.info(f"Kill node '{name}'")
        self._multiple_screen_do_check = True
        success = False
        screens = screen.get_active_screens(name)
        if len(screens.items()) == 0:
            return json.dumps({'result': success, 'message': 'Node does not have an active screen'}, cls=SelfEncoder)

        for session_name, node_name in screens.items():
            pid, session_name = screen.split_session_name(session_name)
            os.kill(pid, signal.SIGKILL)
            success = True
        return json.dumps({'result': success, 'message': ''}, cls=SelfEncoder)

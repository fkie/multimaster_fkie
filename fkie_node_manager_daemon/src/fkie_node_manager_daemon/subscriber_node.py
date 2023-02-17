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
from typing import Tuple

import argparse
import asyncio
import json
import threading

import rospy
from roslib import message
from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.runtime_interface import SubscriberEvent
from fkie_multimaster_msgs.logging.logging import Log


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


class MsgEncoder(json.JSONEncoder):
    def __init__(self, *, skipkeys: bool = False, ensure_ascii: bool = True, check_circular: bool = True, allow_nan: bool = True, sort_keys: bool = False, indent: Union[int, str, None] = None, separators: Union[Tuple[str, str], None] = None, default: Union[ Callable[..., Any], None] = None,
                 no_arr: bool = True,
                 no_str: bool = True) -> None:
        super().__init__(skipkeys=skipkeys, ensure_ascii=ensure_ascii, check_circular=check_circular, allow_nan=allow_nan, sort_keys=sort_keys, indent=indent, separators=separators, default=default)
        self.no_arr = no_arr
        self.no_str = no_str

    def default(self, obj):
        result = {}
        for key in obj.__slots__:
            result[key] = getattr(obj, key)
        return result


class SubscriberNode(CrossbarBaseSession):

    def __init__(self, node_name: str, log_level: int = rospy.INFO, test_env: bool = False):
        self.parser = self._init_arg_parser()
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None
        rospy.init_node(node_name, log_level=log_level)
        self._topic = parsed_args.topic
        self._message_type = parsed_args.message_type
        self._no_data = parsed_args.no_data
        self._no_arr = parsed_args.no_arr
        self._no_str = parsed_args.no_str
        self._hz = parsed_args.hz
        self._window = parsed_args.window
        self._tcp_no_delay = parsed_args.tcp_no_delay
        self._crossbar_port = parsed_args.crossbar_port
        self._crossbar_realm = parsed_args.crossbar_realm

        self._latched_messages = []

        Log.info(f"start publisher for {self._topic}[{self._message_type}]")
        self.__msg_class = message.get_message_class(self._message_type)
        if self.__msg_class:
            self.crossbar_loop = asyncio.get_event_loop()
            CrossbarBaseSession.__init__(
                self, self.crossbar_loop, self._crossbar_realm, self._crossbar_port, test_env)
            self._crossbarThread = threading.Thread(
                target=self.run_crossbar_forever, args=(self.crossbar_loop,), daemon=True)
            self._crossbarThread.start()
            self.sub = rospy.Subscriber(
                self._topic, self.__msg_class, self._msg_handle)
        else:
            raise Exception(
                f"Cannot load message class for [{self._message_type}]. Did you build messages?")

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, 'crossbar_loop'):
            self.crossbar_loop.stop()

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _init_arg_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser()
        parser.add_argument('--crossbar_port', nargs='?', type=int,
                            required=True,  help='port for crossbar server')
        parser.add_argument('--crossbar_realm', nargs='?', type=str,
                            default='ros',  help='realm crossbar server')
        parser.add_argument('-t', '--topic', nargs='?', required=True,
                            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        parser.add_argument("-m", "--message_type", nargs='?', required=True,
                            help="Type of the ROS message (e.g. 'std_msgs/msg/String')")
        parser.add_argument('--no_data', action='store_true',
                            help='Report only statistics without message content.')
        parser.add_argument('--no_arr', action='store_true',
                            help='Exclude arrays.')
        parser.add_argument('--no_str', action='store_true',
                            help='Exclude string fields.')
        parser.add_argument('--hz', nargs='?', type=int, default=1,
                            help='Rate to forward messages. Ignored on latched topics. Disabled by 0.')
        parser.add_argument('--window', nargs='?', type=int, default=1,
                            help='window size, in # of messages, for calculating rate.')
        parser.add_argument('--tcp_no_delay', action='store_true',
                            help='use the TCP_NODELAY transport hint when subscribing to topics (Only ROS1).')
        # parser.add_argument('--use_sim_time', type=str2bool, nargs='?', const=True, default=False, help='Enable ROS simulation time (Only ROS2).')
        parser.set_defaults(no_data=False)
        parser.set_defaults(no_arr=False)
        parser.set_defaults(no_str=False)
        parser.set_defaults(tcp_no_delay=False)
        parser.set_defaults(help=False)
        return parser

    def _msg_handle(self, data):
        self._latched = data._connection_header['latching'] != '0'
        if self.crossbar_registered:
            print(f"LATCHEND: {data._connection_header['latching'] != '0'}")
            event = SubscriberEvent(self._topic, self._message_type)
            event.data = json.loads(json.dumps(data, cls=MsgEncoder, **{"no_arr": self._no_arr}))
            self.publish_to('ros.subscriber.event', event)
        elif self._latched:
            self._latched_messages.append(data)

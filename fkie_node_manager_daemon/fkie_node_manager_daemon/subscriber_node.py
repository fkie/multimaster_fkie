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


import argparse
import asyncio
import json
import os
import signal
import sys
import threading
import time
import traceback
from importlib import import_module
from types import SimpleNamespace
from typing import Any
from typing import Callable
from typing import Union
from typing import Tuple

import rclpy
from rclpy.client import SrvType
from rclpy.client import SrvTypeRequest
from rclpy.client import SrvTypeResponse
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from fkie_node_manager_daemon.server import Server
from fkie_multimaster_pylib.crossbar import server
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.runtime_interface import SubscriberEvent
from fkie_multimaster_pylib.crossbar.runtime_interface import SubscriberFilter
from fkie_multimaster_pylib.defines import NM_NAMESPACE
from fkie_multimaster_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_multimaster_pylib.system.host import ros_host_suffix
from fkie_multimaster_pylib.system.screen import test_screen

import fkie_node_manager_daemon as nmd
from fkie_multimaster_pylib.logging.logging import Log


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
    def __init__(self, *, skipkeys: bool = False, ensure_ascii: bool = True, check_circular: bool = True, allow_nan: bool = True, sort_keys: bool = False, indent: Union[int, str, None] = None, separators: Union[Tuple[str, str], None] = None, default: Union[Callable[..., Any], None] = None,
                 no_arr: bool = True,
                 no_str: bool = True) -> None:
        super().__init__(skipkeys=skipkeys, ensure_ascii=ensure_ascii, check_circular=check_circular,
                         allow_nan=allow_nan, sort_keys=sort_keys, indent=indent, separators=separators, default=default)
        self.no_arr = no_arr
        self.no_str = no_str

    def default(self, obj):
        result = {}
        fields = obj.__slots__
        if hasattr(obj, 'get_fields_and_field_types'):
            fields = obj.get_fields_and_field_types()
        for key in fields:
            skip = False
            if self.no_arr and isinstance(getattr(obj, key), (list, dict)):
                skip = True
            if self.no_str and isinstance(getattr(obj, key), str):
                skip = True
            if not skip:
                result[key] = getattr(obj, key)
        return result


class RosSubscriberLauncher(CrossbarBaseSession):
    '''
    Launches the ROS node to forward a topic subscription.
    '''

    DEFAULT_WINDOWS_SIZE = 5000

    def __init__(self, test_env=False):
        self.ros_domain_id = 0
        self.parser = self._init_arg_parser()
        # change terminal name
        parsed_args, remaining_args = self.parser.parse_known_args()
        if parsed_args.help:
            return None
        self.namespace, self.name = ros2_subscriber_nodename_tuple(parsed_args.topic)
        print('\33]0;%s\a' % (self.name), end='', flush=True)
        # self._displayed_name = parsed_args.name
        # self._port = parsed_args.port
        # self._load = parsed_args.load
        if 'ROS_DOMAIN_ID' in os.environ:
            self.ros_domain_id = int(os.environ['ROS_DOMAIN_ID'])
            # TODO: switch domain id
            # os.environ.pop('ROS_DOMAIN_ID')
        rclpy.init(args=remaining_args)
        #NM_NAMESPACE
        self.rosnode = rclpy.create_node(self.name, namespace=self.namespace)

        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.rosnode)

        self._topic = parsed_args.topic
        self._message_type = parsed_args.message_type
        self._count_received = 0
        self._no_data = parsed_args.no_data
        self._no_arr = parsed_args.no_arr
        self._no_str = parsed_args.no_str
        self._hz = parsed_args.hz
        self._window = parsed_args.window
        if self._window == 0:
            self._window = self.DEFAULT_WINDOWS_SIZE
        self._tcp_no_delay = parsed_args.tcp_no_delay
        self._crossbar_port = parsed_args.crossbar_port
        self._crossbar_realm = parsed_args.crossbar_realm

        self._send_ts = 0
        self._latched_messages = []
        # stats parameter
        self._last_received_ts = 0
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._times = []
        self._bytes = []
        self._bws = []

        nmd.ros_node = self.rosnode
        # set loglevel to DEBUG
        nmd.ros_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # get a reference to the global node for logging
        Log.set_ros2_logging_node(self.rosnode)

        Log.info(f"start subscriber for {self._topic}[{self._message_type}]")
        splitted_type = self._message_type.replace('/', '.').rsplit('.', 1)
        splitted_type.reverse()
        module = import_module(splitted_type.pop())
        sub_class = getattr(module, splitted_type.pop())
        while splitted_type:
            sub_class = getattr(sub_class, splitted_type.pop())
        if sub_class is None:
            raise ImportError(f"invalid message type: '{self._message_type}'. If this is a valid message type, perhaps you need to run 'colcon build'")

        self.__msg_class = sub_class
        self.crossbar_loop = asyncio.get_event_loop()
        CrossbarBaseSession.__init__(
            self, self.crossbar_loop, self._crossbar_realm, self._crossbar_port, test_env=test_env)
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(self.crossbar_loop,), daemon=True)
        self._crossbarThread.start()
        qos_state_profile = QoSProfile(depth=100,
                                    # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    # history=QoSHistoryPolicy.KEEP_LAST,
                                    # reliability=QoSReliabilityPolicy.RELIABLE)
                                    )
        self.sub = nmd.ros_node.create_subscription(
            self.__msg_class, self._topic, self._msg_handle, qos_profile=qos_state_profile)
        self.subscribe_to(
            f"ros.subscriber.filter.{self._topic.replace('/', '_')}", self._clb_update_filter)

    def __del__(self):
        self.stop()

    def spin(self):
        try:
            self.executor.spin()
                # rclpy.spin(self.rosnode)
        except KeyboardInterrupt:
            pass
        except Exception:
            # on load error the process will be killed to notify user
            # in node_manager about error
            self.rosnode.get_logger().warning('Start failed: %s' %
                                              traceback.format_exc())
            sys.stdout.write(traceback.format_exc())
            sys.stdout.flush()
            # TODO: how to notify user in node manager about start errors
            # os.kill(os.getpid(), signal.SIGKILL)
        self.sub.destroy()
        print('shutdown rclpy')
        self.executor.shutdown()
        # rclpy.shutdown()
        print('bye!')

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

    def stop(self):
        if hasattr(self, 'crossbar_loop'):
            self.crossbar_loop.stop()

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _msg_handle(self, data):
        self._count_received += 1
        self._latched = False
        #self._latched = data._connection_header['latching'] != '0'
        # print(data._connection_header)
        # print(dir(data))
        # print("SIZE", data.__sizeof__())
        #print(f"LATCHEND: {data._connection_header['latching'] != '0'}")
        event = SubscriberEvent(self._topic, self._message_type)
        event.latched = self._latched
        if not self._no_data:
            event.data = json.loads(json.dumps(
                data, cls=MsgEncoder, **{"no_arr": self._no_arr, "no_str": self._no_str}))
        event.count = self._count_received
        self._calc_stats(data, event)
        print(f"publish_to: ", f"ros.subscriber.event.{self._topic.replace('/', '_')}")
        timeouted = self._hz == 0
        if self._hz != 0:
            now = time.time()
            if now - self._send_ts > 1.0 / self._hz:
                self._send_ts = now
                timeouted = True
        if event.latched or timeouted:
            self.publish_to(
                f"ros.subscriber.event.{self._topic.replace('/', '_')}", event, resend_after_connect=self._latched)

    def _get_message_size(self, msg):
        # print("size:", msg.__sizeof__())
        # print("dir:", dir(msg))
        return msg.__sizeof__()
        buff = None
        from io import BytesIO  # Python 3.x
        buff = BytesIO()
        print(dir(msg))
        #msg.serialize(buff)
        #return buff.getbuffer().nbytes
        return 0

    def _calc_stats(self, msg, event):
        current_time = time.time()
        if current_time - self._last_received_ts > 1:
            pass
        msg_len = self._get_message_size(msg)
        if msg_len > -1:
            event.size = msg_len
            event.size_min = msg_len
            event.size_max = msg_len
        if self._msg_t0 < 0 or self._msg_t0 > current_time:
            self._msg_t0 = current_time
            self._msg_tn = current_time
            self._times = []
            self._bytes = []
        else:
            self._times.append(current_time - self._msg_tn)
            self._msg_tn = current_time

            if msg_len > -1:
                self._bytes.append(msg_len)
            if len(self._bytes) > self._window:
                self._bytes.pop(0)
            if len(self._times) > self._window:
                self._times.pop(0)

            sum_times = sum(self._times)

            if self._bytes:
                sum_bytes = sum(self._bytes)
                if sum_times > 3:
                    event.bw = float(sum_bytes) / float(sum_times)
                    self._bws.append(event.bw)
                    if len(self._bws) > self._window:
                        self._bws.pop(0)
                    event.bw_min = min(self._bws)
                    event.bw_max = max(self._bws)
                event.size_min = min(self._bytes)
                event.size_max = max(self._bytes)

        # the code from ROS rostopic
        n = len(self._times)
        if n > 1:
            avg = sum_times / n
            event.rate = 1. / avg if avg > 0. else 0

        # # min and max
        # if self.SHOW_JITTER or self.show_only_rate:
        #     max_delta = max(self.times)
        #     min_delta = min(self.times)
        #     message_jitter = "jitter[ min: %.3fs   max: %.3fs ]" % (
        #         min_delta, max_delta)
        # # std dev
        # self.last_printed_count = self.message_count
        # if self.SHOW_STD_DEV or self.show_only_rate:
        #     std_dev = math.sqrt(
        #         sum((x - mean) ** 2 for x in self.times) / n)
        #     message_std_dev = "std dev: %.5fs" % (std_dev)
        # if self.SHOW_WINDOW_SIZE or self.show_only_rate:
        #     message_window = "window: %s" % (n + 1)

        self._last_received_ts = current_time

    def _clb_update_filter(self, json_filter: SubscriberFilter):
        # Convert filter settings into a proper python object
        request = json.loads(json.dumps(json_filter),
                             object_hook=lambda d: SimpleNamespace(**d))
        Log.info(f"update filter for {self._topic}[{self._message_type}]")
        self._no_data = request.no_data
        self._no_arr = request.no_arr
        self._no_str = request.no_str
        self._hz = request.hz
        if self._window != request.window:
            self._window = request.window
            if self._window == 0:
                self._window = self.DEFAULT_WINDOWS_SIZE
            while len(self._bytes) > self._window:
                self._bytes.pop(0)
            while len(self._times) > self._window:
                self._times.pop(0)
            while len(self._bws) > self._window:
                self._bws.pop(0)

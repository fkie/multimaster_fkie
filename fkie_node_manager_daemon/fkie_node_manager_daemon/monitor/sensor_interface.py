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

import abc
import rclpy
import threading

from typing import Callable

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_multimaster_pylib import formats
from fkie_multimaster_pylib.settings import Settings
import fkie_node_manager_daemon as nmd


class SensorInterface(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, hostname: str = '', sensorname: str = 'noname', interval: float = 1.0):
        nmd.ros_node.get_logger().info("Loading monitor service: %s" % type(self).__name__)
        self.hostname = hostname
        self.mutex = threading.RLock()
        self._interval = interval
        self._timer = None
        self._stat_msg = DiagnosticStatus()
        self._stat_msg.name = '%s' % sensorname
        self._stat_msg.level = DiagnosticStatus.STALE
        self._stat_msg.hardware_id = hostname
        self._stat_msg.message = 'No Data'
        self._stat_msg.values = []
        self._ts_last = 0
        self._start_check_sensor()

    @abc.abstractmethod
    def check_sensor(self):
        pass

    @abc.abstractmethod
    def reload_parameter(self, settings: Settings):
        pass

    def _start_check_sensor(self):
        if not self.is_active():
            return
        self.check_sensor()
        if self.is_active() and self._interval > 0:
            self.start_timer(self._interval, self._start_check_sensor)

    def last_state(self, ts_now: float = 0, filter_level: list = [], filter_ts: float = 0):
        '''
        :param float ts_now: current timestamp
        :param int filter_level: minimal level
        :param float filter_ts: only message after this timestamp
        :return: last state if data is available. In other case it should be None
        :rtype: diagnostic_msgs.msg.DiagnosticStatus
        '''
        with self.mutex:
            if self._ts_last > 0:
                if self._ts_last > filter_ts and self._stat_msg.level in filter_level:
                    self.update_value_last_ts(
                        self._stat_msg, ts_now, self._ts_last)
                    return self._stat_msg
        return None

    def update_value_last_ts(self, msg, nowts: float = 0, ts: float = 0):
        if msg.values and msg.values[-1].key == 'Timestamp':
            del msg.values[-1]
        msg.values.append(
            KeyValue(key='Timestamp', value=formats.timestamp_fmt(ts, False, False)))

    def is_active(self):
        if not rclpy.ok():
            with self.mutex:
                self.cancel_timer()
            return False
        return True

    def start_timer(self, interval: float, callback: Callable):
        self._timer = threading.Timer(interval, callback)
        self._timer.start()

    def cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

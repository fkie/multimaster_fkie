# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Fraunhofer FKIE/US, Alexander Tiderko
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

import rclpy
import threading
import time

from rclpy.clock import Clock

import fkie_node_manager_daemon as nmd
from fkie_multimaster_pylib.settings import Settings
from fkie_multimaster_pylib.system.host import get_host_name
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fkie_node_manager_daemon.time_helpers import rostime2float
from .cpu_load import CpuLoad
from .cpu_temperatur import CpuTemperatur
from .hdd_usage import HddUsage
from .mem_usage import MemUsage
from .net_load import NetLoad


class DiagnosticObj(DiagnosticStatus):

    def __init__(self, msg=DiagnosticStatus(), timestamp=0):
        self.msg = msg
        self.timestamp = timestamp

    def __eq__(self, item):
        if isinstance(item, DiagnosticObj):
            return self.msg.name == item.msg.name
        if isinstance(item, DiagnosticStatus):
            return self.msg.name == item.name

    def __ne__(self, item):
        return not (self == item)

    def __gt__(self, item):
        if isinstance(item, DiagnosticObj):
            return self.msg.name > item.msg.name
        if isinstance(item, DiagnosticStatus):
            return self.msg.name > item.name


class Service:

    def __init__(self, settings: Settings, callbackDiagnostics=None):
        self._clock = Clock()
        self._settings = settings
        self._mutex = threading.RLock()
        self._diagnostics = []  # DiagnosticObj
        self._callbackDiagnostics = callbackDiagnostics
        self.use_diagnostics_agg = settings.param(
            'global/use_diagnostics_agg', True)
        self._sub_diag = None
        if self.use_diagnostics_agg:
            self._sub_diag = nmd.ros_node.create_subscription(
                DiagnosticArray, '/diagnostics_agg', self._callback_diagnostics, 100)
        else:
            self._sub_diag = nmd.ros_node.create_subscription(
                DiagnosticArray, '/diagnostics', self._callback_diagnostics, 100)
        hostname = get_host_name()

        self.sensors = []
        self.sensors.append(CpuLoad(hostname, 1.0, 0.9))
        self.sensors.append(CpuTemperatur(hostname, 1.0, 85.0))
        self.sensors.append(HddUsage(hostname, 30.0, 0.95))
        self.sensors.append(MemUsage(hostname, 1.0, 0.95))
        self.sensors.append(NetLoad(hostname, 1.0, 0.9))
        for sensor in self.sensors:
            self._settings.add_reload_listener(sensor.reload_parameter)
        self._settings.add_reload_listener(self.reload_parameter)

    def reload_parameter(self, settings: Settings):
        value = settings.param('global/use_diagnostics_agg', False)
        if value != self.use_diagnostics_agg:
            if self._sub_diag is not None:
                nmd.ros_node.destroy_subscription(self._sub_diag)
                self._sub_diag = None
            if value:
                self._sub_diag = nmd.ros_node.create_subscription(
                    DiagnosticArray, '/diagnostics_agg', self._callback_diagnostics, 100)
            else:
                self._sub_diag = nmd.ros_node.create_subscription(
                    DiagnosticArray, '/diagnostics', self._callback_diagnostics, 100)
            self.use_diagnostics_agg = value

    def _callback_diagnostics(self, msg: DiagnosticArray):
        # TODO: update diagnostics
        with self._mutex:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000.0
            for status in msg.status:
                try:
                    idx = self._diagnostics.index(status)
                    diag_obj = self._diagnostics[idx]
                    diag_obj.msg = status
                    diag_obj.timestamp = stamp
                except Exception:
                    diag_obj = DiagnosticObj(status, stamp)
                    diag_obj.timestamp = stamp
                    self._diagnostics.append(diag_obj)
            if self._callbackDiagnostics:
                self._callbackDiagnostics(msg)

    def get_system_diagnostics(self, filter_level: list = [], filter_ts: float = 0):
        result = DiagnosticArray()
        with self._mutex:
            now = self._clock.now()
            result.header.stamp = self._clock.now().to_msg()  # rospy.Time.from_sec(nowsec)
            for sensor in self.sensors:
                diag_msg = sensor.last_state(
                    rostime2float(now), filter_level, filter_ts)
                if diag_msg is not None:
                    result.status.append(diag_msg)
        return result

    def get_diagnostics(self, filter_level: list = [], filter_ts: float = 0):
        result = DiagnosticArray()
        # rospy.Time.from_sec(time.time())
        result.header.stamp = self._clock.now().to_msg()
        with self._mutex:
            for diag_obj in self._diagnostics:
                if diag_obj.timestamp >= filter_ts:
                    #                    if int.from_bytes(diag_obj.msg.level, byteorder='big') >= filter_level:
                    if diag_obj.msg.level in filter_level:
                        result.status.append(diag_obj.msg)
        return result

    def stop(self):
        with self._mutex:
            for sensor in self.sensors:
                sensor.cancel_timer()
            # destroy subscription to avoid runtime warnings
            if self._sub_diag is not None:
                nmd.ros_node.destroy_subscription(self._sub_diag)
                self._sub_diag = None

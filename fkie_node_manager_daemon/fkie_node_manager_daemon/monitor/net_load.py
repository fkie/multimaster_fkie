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

import psutil
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from .sensor_interface import SensorInterface


class NetLoad(SensorInterface):

    def __init__(self, hostname='', interval=3.0, warn_level=0.9):
        self._net_load_warn = warn_level
        self._net_speed = 6
        self._net_stat_last = {}  # {iface: (sent, recv)}
        self._interface = ''
        self.settings = None
        SensorInterface.__init__(
            self, hostname, sensorname='Network Load', interval=interval)

    def reload_parameter(self, settings):
        self._net_load_warn = settings.param(
            'sysmon/Network/load_warn_level', self._net_load_warn)
        self._net_speed = settings.param(
            'sysmon/Network/speed', self._net_speed)
        # TODO: support more than one interface
        self._interface = settings.param('sysmon/Network/interface', '')
        self.settings = settings

    def check_sensor(self):
        net_stats = psutil.net_if_stats()
        net = psutil.net_io_counters(pernic=True)
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        diag_msg = 'warn at >%.2f%% at %.0fMBit' % (
            self._net_load_warn * 100.0, self._net_speed)
        now = time.time()
        warn_level = self._net_load_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        interfaces = []
        for net_if, net_if_stats in net_stats.items():
            interfaces.append(net_if)
            do_parse = net_if in net
            if not self._interface:
                do_parse = do_parse and net_if_stats.isup and net_if_stats.speed > 0
            else:
                do_parse = do_parse and net_if == self._interface
            if do_parse:
                if self.settings is not None and not self._interface:
                    # TODO: support more than one interface
                    self._interface = net_if
                    self.settings.set_param('sysmon/Network/interface', net_if)
                net_values = net[net_if]
                # in psutil versions below 5.3.0 there is no 'nowrap' argument. We need to calculate current rate itself.
                if net_if not in self._net_stat_last:
                    self._net_stat_last[net_if] = (0, 0)
                duration = now - self._ts_last
                bytes_sent_1s = (net_values.bytes_sent -
                                 self._net_stat_last[net_if][0]) / duration
                bytes_recv_1s = (net_values.bytes_recv -
                                 self._net_stat_last[net_if][1]) / duration
                # store current overall stats for next rate calculation
                self._net_stat_last[net_if] = (
                    net_values.bytes_sent, net_values.bytes_recv)
                # diag_vals.append(KeyValue(key='%s: bytes sent (total)' % net_if, value=net_values.bytes_sent))
                # diag_vals.append(KeyValue(key='%s: bytes recv (total)' % net_if, value=net_values.bytes_recv))
                # diag_vals.append(KeyValue(key='%s: packets sent (total)' % net_if, value=net_values.packets_sent))
                # diag_vals.append(KeyValue(key='%s: packets recv (total)' % net_if, value=net_values.packets_recv))
                diag_vals.append(
                    KeyValue(key='%s: sent [1s]' % net_if, value='%.2f' % bytes_sent_1s))
                diag_vals.append(
                    KeyValue(key='%s: recv [1s]' % net_if, value='%.2f' % bytes_recv_1s))
                percent_sent = bytes_sent_1s / \
                    (self._net_speed * 1024 * 1024 / 8)
                # diag_vals.append(KeyValue(key='%s: sent [%%]' % net_if, value='%.2f' % (percent_sent * 100)))
                percent_recv = bytes_recv_1s / \
                    (self._net_speed * 1024 * 1024 / 8)
                # diag_vals.append(KeyValue(key='%s: recv [%%]' % net_if, value='%.2f' % (percent_recv * 100)))
                if percent_sent >= warn_level or percent_recv >= warn_level:
                    diag_level = DiagnosticStatus.WARN
                    if percent_sent >= warn_level:
                        diag_msg = 'Net load for sent is %.0f%% (warn >%.0f%% [%dMBit])' % (
                            percent_sent * 100, self._net_load_warn * 100, self._net_speed)
                    if percent_recv >= warn_level:
                        if diag_msg:
                            diag_msg = 'Net load for sent is %.0f%% and recv %.0f%% (warn >%.0f%% [%dMBit])' % (
                                percent_sent * 100, percent_recv * 100, self._net_load_warn * 100, self._net_speed)
                        else:
                            diag_msg = 'Net load for recv is %.0f%% (warn >%.0f%% [%dMBit])' % (
                                percent_recv * 100, self._net_load_warn * 100, self._net_speed)
                # print '%s: percent %.2f, %.2f' % (net_if, percent_sent, percent_recv), ", level:", diag_level
        if self.settings is not None:
            self.settings.set_param(
                'sysmon/Network/interface', interfaces, tag=':alt')
        # Update status
        with self.mutex:
            self._ts_last = now
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

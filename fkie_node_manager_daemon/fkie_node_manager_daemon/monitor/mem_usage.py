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
from fkie_multimaster_msgs import formats
from .sensor_interface import SensorInterface


class MemUsage(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.95):
        self._mem_usage_warn = warn_level
        SensorInterface.__init__(
            self, hostname, sensorname='Memory Usage', interval=interval)

    def reload_parameter(self, settings):
        self._mem_usage_warn = settings.param(
            'sysmon/Memory/usage_warn_level', self._mem_usage_warn)

    def check_sensor(self):
        mem = psutil.virtual_memory()
        diag_level = DiagnosticStatus.OK
        diag_vals = []
        warn_on_mem = mem.total * (1.0 - self._mem_usage_warn)
        diag_msg = 'warn at >%s%% (<%s)' % (
            self._mem_usage_warn * 100., formats.sizeof_fmt(warn_on_mem))
        warn_level = warn_on_mem
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 1.1
        if mem.total - mem.used <= warn_on_mem:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'Memory available %s (warn <%s)' % (
                formats.sizeof_fmt(mem.total - mem.used), formats.sizeof_fmt(warn_on_mem))
        # print "MEM available ", mem.available, diag_level
        diag_vals.append(KeyValue(key='Free', value='%d' %
                                  (mem.total - mem.used)))
        diag_vals.append(KeyValue(key='Free [%]', value='%.2f' % (
            float(mem.total - mem.used) * 100.0 / float(mem.total))))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

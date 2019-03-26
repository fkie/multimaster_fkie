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

import psutil
import rospy
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from .sensor_interface import SensorInterface


class CpuLoad(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.9):
        self._cpu_load_warn = rospy.get_param('~cpu_load_warn', warn_level)
        SensorInterface.__init__(self, hostname, sensorname='CPU Load', interval=interval)

    def check_sensor(self):
        cpu_percent_total = psutil.cpu_percent(interval=None)
        cpu_percents = psutil.cpu_percent(interval=None, percpu=True)
        diag_level = 0
        diag_vals = []
        diag_msg = ''
        warn_level = self._cpu_load_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        diag_vals.append(KeyValue(key='CPU percent (total)', value=cpu_percent_total))
        count_warn_cpu = 0
        for cpu_idx in range(len(cpu_percents)):
            diag_vals.append(KeyValue(key='CPU percent (%d)' % cpu_idx, value=cpu_percents[cpu_idx]))
            if cpu_percents[cpu_idx] / 100.0 >= warn_level:
                count_warn_cpu += 1
        if count_warn_cpu > 1:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'CPU load of %d cores is >%.0f%%)' % (count_warn_cpu, self._cpu_load_warn * 100)
        cpu_count = psutil.cpu_count(logical=True)
        diag_vals.append(KeyValue(key='CPU count', value=cpu_count))

        # Update status
        with self.mutex:
            diag_vals.append(KeyValue(key='Update Status', value='OK'))
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

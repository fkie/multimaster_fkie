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
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from .sensor_interface import SensorInterface


class CpuLoad(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.9):
        self._cpu_load_warn = warn_level
        self._count_processes = 3
        SensorInterface.__init__(self, hostname, sensorname='CPU Load', interval=interval)

    def reload_parameter(self, settings):
        self._cpu_load_warn = settings.param('sysmon/CPU/load_warn_level', self._cpu_load_warn)
        self._count_processes = settings.param('sysmon/CPU/count_processes', 3)

    def check_sensor(self):
        cpu_percents = psutil.cpu_percent(interval=None, percpu=True)
        diag_level = 0
        diag_vals = []
        diag_msg = 'warn at >%.2f%%' % (self._cpu_load_warn * 100.0)
        warn_level = self._cpu_load_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        count_warn_cpu = 0
        cpu_max_percent = 0
        cpu_percent_total = 0
        for cpu_idx in range(len(cpu_percents)):
            cpu_percent = cpu_percents[cpu_idx]
            if cpu_percent > cpu_max_percent:
                cpu_max_percent = cpu_percent
            if cpu_percents[cpu_idx] / 100.0 >= warn_level:
                count_warn_cpu += 1
            cpu_percent_total += cpu_percent
        diag_vals.append(KeyValue(key='Max [%]', value=cpu_max_percent))
        diag_vals.append(KeyValue(key='Avg [%]', value='%.2f' % (cpu_percent_total / len(cpu_percents))))
        if count_warn_cpu > 1:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'CPU load of %d cores is >%.0f%%' % (count_warn_cpu, self._cpu_load_warn * 100)
            try:
                # determine processes with high load
                processes = []
                for pi in sorted(psutil.process_iter(attrs=['name', 'cpu_percent']), key=lambda pi: pi.info['cpu_percent'], reverse=True):
                    if pi.info['cpu_percent'] / 100.0 >= warn_level:
                        phlmsg = '%.2f%% %s [%d]' % (pi.info['cpu_percent'], pi.info['name'], pi.pid)
                        processes.append(phlmsg)
                    if len(processes) >= self._count_processes:
                        break
                for msg in processes:
                    diag_vals.append(KeyValue(key='Process load', value=msg))
            except Exception:
                pass
        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

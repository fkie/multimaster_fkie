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
from fkie_node_manager_daemon.common import sizeof_fmt
from .sensor_interface import SensorInterface


class MemUsage(SensorInterface):

    def __init__(self, hostname='', interval=5.0, warn_level=0.95):
        self._mem_usage_warn = warn_level
        SensorInterface.__init__(self, hostname, sensorname='Memory Usage', interval=interval)

    def reload_parameter(self, settings):
        self._mem_usage_warn = settings.param('sysmon/Memory/usage_warn_level', self._mem_usage_warn)

    def check_sensor(self):
        mem = psutil.virtual_memory()
        diag_level = 0
        diag_vals = []
        warn_on_mem = mem.total * (1.0 - self._mem_usage_warn)
        diag_msg = 'warn at >%s%% (<%s)' % (self._mem_usage_warn * 100., sizeof_fmt(warn_on_mem))
        warn_level = warn_on_mem
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 1.1
        if mem.total - mem.used <= warn_on_mem:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'Memory available %s (warn <%s)' % (sizeof_fmt(mem.total - mem.used), sizeof_fmt(warn_on_mem))
        # print "MEM available ", mem.available, diag_level
        diag_vals.append(KeyValue(key='Free', value=mem.total - mem.used))
        diag_vals.append(KeyValue(key='Free [%]', value='%.2f' % (float(mem.total - mem.used) * 100.0 / float(mem.total))))

        # Update status
        with self.mutex:
            self._ts_last = time.time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

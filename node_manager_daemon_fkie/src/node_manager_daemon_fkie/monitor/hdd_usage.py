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

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from node_manager_daemon_fkie.common import sizeof_fmt
from node_manager_daemon_fkie.screen import LOG_PATH
from .sensor_interface import SensorInterface


class HddUsage(SensorInterface):

    def __init__(self, hostname='', interval=30.0, warn_level=100.0):
        self._hdd_usage_warn = rospy.get_param('~hdd_usage_warn', warn_level)
        SensorInterface.__init__(self, hostname, sensorname='HDD Usage', interval=interval)

    def check_sensor(self):
        hdd = psutil.disk_usage(LOG_PATH)
        diag_level = 0
        diag_vals = []
        diag_msg = 'disk size for %s' % LOG_PATH
        warn_level = self._hdd_usage_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        if hdd.free <= 1024 * 1024 * warn_level:
            diag_level = DiagnosticStatus.WARN
            diag_msg = 'Free disk space on log path only %s (warn on <%s)' % (sizeof_fmt(hdd.free), sizeof_fmt(self._hdd_usage_warn * 1024 * 1024))
        # print "Percent Disk %.2f" % (hdd.percent), diag_level
        diag_vals.append(KeyValue(key='Total Disk', value=hdd.total))
        diag_vals.append(KeyValue(key='Used Disk', value=hdd.used))
        diag_vals.append(KeyValue(key='Free Disk', value=hdd.free))
        diag_vals.append(KeyValue(key='Percent Disk', value=hdd.percent))

        # Update status
        with self.mutex:
            diag_vals.append(KeyValue(key='Update Status', value='OK'))
            self._ts_last = rospy.get_time()
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

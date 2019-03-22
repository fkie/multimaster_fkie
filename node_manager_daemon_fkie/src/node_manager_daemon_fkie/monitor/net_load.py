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
from .sensor_interface import SensorInterface


class NetLoad(SensorInterface):

    def __init__(self, hostname='', interval=3.0, warn_level=0.9):
        self._net_load_warn = rospy.get_param('~net_load_warn', warn_level)
        self._net_stat_last = {}  # {iface: (sent, recv)}
        SensorInterface.__init__(self, hostname, sensorname='Network Load', interval=interval)

    def check_sensor(self):
        net_stats = psutil.net_if_stats()
        net = psutil.net_io_counters(pernic=True)
        diag_level = 0
        diag_vals = []
        diag_msg = ''
        now = rospy.get_time()
        warn_level = self._net_load_warn
        if diag_level == DiagnosticStatus.WARN:
            warn_level = warn_level * 0.9
        for net_if, net_if_stats in net_stats.items():
            if net_if_stats.isup and net_if_stats.speed > 0 and net_if in net:
                net_values = net[net_if]
                # in psutil versions below 5.3.0 there is no 'nowrap' argument. We need to calculate current rate itself.
                if net_if not in self._net_stat_last:
                    self._net_stat_last[net_if] = (0, 0)
                duration = now - self._ts_last
                bytes_sent_1st = (net_values.bytes_sent - self._net_stat_last[net_if][0]) / duration
                bytes_recv_1s = (net_values.bytes_recv - self._net_stat_last[net_if][1]) / duration
                # store current overall stats for next rate calculation
                self._net_stat_last[net_if] = (net_values.bytes_sent, net_values.bytes_recv)
                diag_vals.append(KeyValue(key='%s: bytes sent (total)' % net_if, value=net_values.bytes_sent))
                diag_vals.append(KeyValue(key='%s: bytes recv (total)' % net_if, value=net_values.bytes_recv))
                diag_vals.append(KeyValue(key='%s: packets sent (total)' % net_if, value=net_values.packets_sent))
                diag_vals.append(KeyValue(key='%s: packets recv (total)' % net_if, value=net_values.packets_recv))
                diag_vals.append(KeyValue(key='%s: bytes sent (1/s)' % net_if, value=bytes_sent_1st))
                diag_vals.append(KeyValue(key='%s: bytes recv (1/s)' % net_if, value=bytes_recv_1s))
                percent_sent = (bytes_sent_1st / 1024 / 1024) / (net_if_stats.speed / 8)
                diag_vals.append(KeyValue(key='%s: percent sent' % net_if, value='%.2f' % percent_sent))
                percent_recv = (bytes_recv_1s / 1024 / 1024) / (net_if_stats.speed / 8)
                diag_vals.append(KeyValue(key='%s: percent recv' % net_if, value='%.2f' % percent_recv))
                if percent_sent >= warn_level or percent_recv >= warn_level:
                    diag_level = DiagnosticStatus.WARN
                    if percent_sent >= warn_level:
                        diag_msg = 'Net load for sent is %.0f%% (warn >%.0f%%)' % (percent_sent * 100, self._net_load_warn * 100)
                    if percent_recv >= warn_level:
                        if diag_msg:
                            diag_msg = 'Net load for sent is %.0f%% and recv %.0f%% (warn >%.0f%%)' % (percent_sent * 100, percent_recv * 100, self._net_load_warn * 100)
                        else:
                            diag_msg = 'Net load for recv is %.0f%% (warn >%.0f%%)' % (percent_recv * 100, self._net_load_warn * 100)
                # print '%s: percent %.2f, %.2f' % (net_if, percent_sent, percent_recv), ", level:", diag_level
        # Update status
        with self.mutex:
            diag_vals.append(KeyValue(key='Update Status', value='OK'))
            self._ts_last = now
            self._stat_msg.level = diag_level
            self._stat_msg.values = diag_vals
            self._stat_msg.message = diag_msg

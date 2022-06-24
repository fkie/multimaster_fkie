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



import rospy
import threading

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fkie_node_manager_daemon.common import formated_ts


class SensorInterface(threading.Thread):

    def __init__(self, hostname='', sensorname='noname', interval=1.0):
        threading.Thread.__init__(self, daemon=True)
        self.hostname = hostname
        self.mutex = threading.RLock()
        self._interval = interval
        self._stat_msg = DiagnosticStatus()
        self._stat_msg.name = '%s' % (sensorname)
        self._stat_msg.level = 3
        self._stat_msg.hardware_id = hostname
        self._stat_msg.message = 'No Data'
        self._stat_msg.values = []
        self._ts_last = 0
        self._stop_event = threading.Event()
        self.start()

    # @abc.abstractmethod
    def check_sensor(self):
        pass

    # @abc.abstractmethod
    def reload_parameter(self, settings):
        pass

    def run(self):
        while not self._stop_event.wait(self._interval):
            self.check_sensor()

    def stop(self):
        self._stop_event.set()

    def last_state(self, ts_now, filter_level=0, filter_ts=0):
        '''
        :param float ts_now: current timestamp
        :param int filter_level: minimal level
        :param float filter_ts: only message after this timestamp
        :return: last state if data is available. In other case it should be None
        :rtype: diagnostic_msgs.msg.DiagnosticStatus
        '''
        with self.mutex:
            if self._ts_last > 0:
                if self._ts_last > filter_ts and self._stat_msg.level >= filter_level:
                    self.update_value_last_ts(self._stat_msg, ts_now, self._ts_last)
                    return self._stat_msg
        return None

    def update_value_last_ts(self, msg, nowts, ts):
        if msg.values and msg.values[-1].key == 'Timestamp':
            del msg.values[-1]
        msg.values.append(KeyValue(key='Timestamp', value=formated_ts(ts, False, False)))

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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



from python_qt_binding.QtCore import QObject, Signal
import random
import socket
import threading
import time
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import rospy

from fkie_master_discovery.master_info import MasterInfo
from fkie_node_manager_daemon.common import utf8


class UpdateThread(QObject, threading.Thread):
    '''
    A thread to retrieve the state about ROS master from remote discovery node and
    publish it be sending a QT signal.
    '''
    update_signal = Signal(MasterInfo)
    '''
  :ivar: update_signal is a signal, which is emitted, if a new
  U{fkie_master_discovery.MasterInfo<http://docs.ros.org/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>} is retrieved.
  '''

    master_errors_signal = Signal(str, list)
    '''
  :ivar: master_errors_signal is a signal (masteruri, list of errors),
  which is emitted, if we get a list with errors from remote master_discovery.
  '''

    error_signal = Signal(str, str)
    '''
  :ivar: error_signal is a signal (masteruri, error message), which is emitted,
  if an error while retrieving a master info was occurred.
  '''

    timediff_signal = Signal(str, float)
    '''
  :ivar: timediff_signal is a signal (masteruri, time difference), which is emitted
  after the difference of time to the remote host is determined.
  '''

    username_signal = Signal(str, str)
    '''
  :ivar: username_signal is a signal (masteruri, username), which is emitted
  after the name was retrieved from host.
  '''

    def __init__(self, monitoruri, masteruri, delayed_exec=0., parent=None):
        '''
        :param str masteruri: the URI of the remote ROS master
        :param str monitoruri: the URI of the monitor RPC interface of the master_discovery node
        :param float delayed_exec: Delay the execution of the request for given seconds.
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._monitoruri = monitoruri
        self._masteruri = masteruri
        self._delayed_exec = delayed_exec
        self.setDaemon(True)

    def run(self):
        '''
        '''
        try:
            delay = self._delayed_exec + 0.5 + random.random()
            # 'print "wait request update", self._monitoruri, delay
            time.sleep(delay)
            # 'print "request update", self._monitoruri
            socket.setdefaulttimeout(25)
            remote_monitor = xmlrpcclient.ServerProxy(self._monitoruri)
            # get first master errors
            try:
                muri, errors = remote_monitor.masterErrors()
                self.master_errors_signal.emit(muri, errors)
            except xmlrpcclient.Fault as _err:
                rospy.logwarn("Older master_discovery on %s detected. It does not support master error reports!" % self._masteruri)
            # get the time difference
            try:
                myts = time.time()
                muri, remote_ts = remote_monitor.getCurrentTime()
                self.timediff_signal.emit(muri, remote_ts - myts - (time.time() - myts) / 2.0)
            except xmlrpcclient.Fault as _errts:
                rospy.logwarn("Older master_discovery on %s detected. It does not support getCurrentTime!" % self._masteruri)
            # get the user name
            try:
                muri, username = remote_monitor.getUser()
                self.username_signal.emit(muri, username)
            except xmlrpcclient.Fault as _errts:
                rospy.logwarn("Older master_discovery on %s detected. It does not support getUser!" % self._masteruri)
            # now get master info from master discovery
            remote_info = remote_monitor.masterInfo()
            master_info = MasterInfo.from_list(remote_info)
            master_info.check_ts = time.time()
            # 'print "request success", self._monitoruri
            self.update_signal.emit(master_info)
        except Exception:
            import traceback
#      print traceback.print_exc()
            formatted_lines = traceback.format_exc(1).splitlines()
            rospy.logwarn("Cannot update ROS state, connection to %s failed:\n\t%s", utf8(self._monitoruri), formatted_lines[-1])
            # 'print "request failed", self._monitoruri
            self.error_signal.emit(self._masteruri, formatted_lines[-1])
        finally:
            if socket is not None:
                socket.setdefaulttimeout(None)

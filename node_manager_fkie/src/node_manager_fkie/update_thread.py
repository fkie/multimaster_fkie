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

import time
import socket
import threading
import xmlrpclib
import random
from python_qt_binding import QtCore

#import roslib; roslib.load_manifest('node_manager_fkie')
import rospy

from master_discovery_fkie.master_info import MasterInfo

class UpdateThread(QtCore.QObject, threading.Thread):
  '''
  A thread to retrieve the state about ROS master from remote discovery node and 
  publish it be sending a QT signal.
  '''
  update_signal = QtCore.Signal(MasterInfo)
  '''
  @ivar: update_signal is a signal, which is emitted, if a new 
  L{aster_discovery_fkie.MasterInfo} is retrieved.
  '''

  master_errors_signal = QtCore.Signal(str, list)
  '''
  @ivar: master_errors_signal is a signal (masteruri, list of errors), 
  which is emitted, if we get a list with errors from remote master_discovery.
  '''

  error_signal = QtCore.Signal(str, str)
  '''
  @ivar: error_signal is a signal (masteruri, error message), which is emitted, 
  if an error while retrieving a master info was occurred.
  '''

  def __init__(self, monitoruri, masteruri, delayed_exec=0., parent=None):
    '''
    @param masteruri: the URI of the remote ROS master
    @type masteruri: C{str}
    @param monitoruri: the URI of the monitor RPC interface of the master_discovery node
    @type monitoruri: C{str}
    @param delayed_exec: Delay the execution of the request for given seconds.
    @type delayed_exec: C{float}
    '''
    QtCore.QObject.__init__(self)
    threading.Thread.__init__(self)
    self._monitoruri = monitoruri
    self._masteruri = masteruri
    self._delayed_exec = delayed_exec
    self.setDaemon(True)

  def run(self):
    '''
    '''
    try:
      delay = self._delayed_exec+0.5+random.random()
      #'print "wait request update", self._monitoruri, delay
      time.sleep(delay)
      #'print "request update", self._monitoruri
      socket.setdefaulttimeout(25)
      remote_monitor = xmlrpclib.ServerProxy(self._monitoruri)
      # get first master errors
      try:
        muri, errors = remote_monitor.masterErrors()
        self.master_errors_signal.emit(muri, errors)
      except xmlrpclib.Fault as err:
        rospy.logwarn("Older master_discovery on %s detected. It does not support master error reports!"%self._masteruri)
      # now get master info from master discovery
      remote_info = remote_monitor.masterInfo()
      master_info = MasterInfo.from_list(remote_info)
      master_info.check_ts = time.time()
      #'print "request success", self._monitoruri
      self.update_signal.emit(master_info)
    except:
      import traceback
#      print traceback.print_exc()
      formatted_lines = traceback.format_exc(1).splitlines()
      rospy.logwarn("Connection to %s failed:\n\t%s", str(self._monitoruri), formatted_lines[-1])
      #'print "request failed", self._monitoruri
      self.error_signal.emit(self._masteruri, formatted_lines[-1])
    finally:
      if not socket is None:
        socket.setdefaulttimeout(None)
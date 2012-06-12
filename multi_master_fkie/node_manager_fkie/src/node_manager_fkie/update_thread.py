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
#  * Neither the name of I Heart Engineering nor the names of its
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
import threading
import xmlrpclib
from PySide import QtCore

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

  def __init__(self, monitoruri, parent=None):
    QtCore.QObject.__init__(self)
    threading.Thread.__init__(self)
    self._monitoruri = monitoruri
    self.setDaemon(True)

  def run(self):
    '''
    '''
    try:
      remote_monitor = xmlrpclib.ServerProxy(self._monitoruri)
      remote_info = remote_monitor.masterInfo()
      master_info = MasterInfo.from_list(remote_info)
      master_info.check_ts = time.time()
      self.update_signal.emit(master_info)
    except:
      import traceback
#      print traceback.print_exc()
      formatted_lines = traceback.format_exc().splitlines()
      rospy.logwarn("Connection to %s failed:\n\t%s", str(self._monitoruri), formatted_lines[-1])
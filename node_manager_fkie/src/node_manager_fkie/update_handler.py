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

import threading
from python_qt_binding import QtCore

from master_discovery_fkie.master_info import MasterInfo
from update_thread import UpdateThread

class UpdateHandler(QtCore.QObject):
  '''
  A class to retrieve the state about ROS master from remote discovery node and 
  publish it be sending a QT signal. To retrieve the state a new thread will be
  created.
  '''
  master_info_signal = QtCore.Signal(MasterInfo)
  '''
  @ivar: master_info_signal is a signal, which is emitted, if a new 
  L{aster_discovery_fkie.MasterInfo} is retrieved.
  '''
  master_errors_signal = QtCore.Signal(str, list)
  '''
  @ivar: master_errors_signal is a signal (masteruri, error list) with errors which
  are occured on remote master_discovery.
  '''

  error_signal = QtCore.Signal(str, str)
  '''
  @ivar: error_signal is a signal (masteruri, error message), which is emitted, 
  if an error while retrieving a master info was occurred.
  '''

  def __init__(self):
    QtCore.QObject.__init__(self)
    self.__updateThreads = {}
    self.__requestedUpdates = {}
    self._lock = threading.RLock()
    
  def stop(self):
    print "  Shutdown update threads..."
    self.__requestedUpdates.clear()
    for _, thread in self.__updateThreads.iteritems():
      thread.join(3)
    print "  Update threads are off!"

  def requestMasterInfo(self, masteruri, monitoruri, delayed_exec=0.0):
    '''
    This method starts a thread to get the informations about the ROS master by
    the given RCP uri of the master_discovery node. If all informations are
    retrieved, a C{master_info_signal} of this class will be emitted. If for given
    masteruri a thread is already running, it will be inserted to the requested
    updates. For the same masteruri only one requested update can be stored. 
    On update error the requested update will be ignored.
    This method is thread safe. 
    
    @param masteruri: the URI of the remote ROS master
    @type masteruri: C{str}
    @param monitoruri: the URI of the monitor RPC interface of the master_discovery node
    @type monitoruri: C{str}
    @param delayed_exec: Delay the execution of the request for given seconds.
    @type delayed_exec: C{float}
    '''
    with self._lock:
      try:
        if (self.__updateThreads.has_key(masteruri)):
          self.__requestedUpdates[masteruri] = (monitoruri, delayed_exec)
        else:
          self.__create_update_thread(monitoruri, masteruri, delayed_exec)
  #        from urlparse import urlparse
  #        om = urlparse(masteruri)
      except:
        pass

  def _on_master_info(self, minfo):
    self.master_info_signal.emit(minfo)
    self.__handle_requests(minfo.masteruri)

  def _on_master_errors(self, masteruri, error_list):
    self.master_errors_signal.emit(masteruri, error_list)

  def _on_error(self, masteruri, error):
    self.error_signal.emit(masteruri, error)
    self.__handle_requests(masteruri)

  def __handle_requests(self, masteruri):
    with self._lock:
      try:
        thread = self.__updateThreads.pop(masteruri)
        del thread
        monitoruri, delayed_exec = self.__requestedUpdates.pop(masteruri)
        self.__create_update_thread(monitoruri, masteruri, delayed_exec)
      except KeyError:
  #      import traceback
  #      print traceback.format_exc(1)
        pass
      except:
        import traceback
        print traceback.format_exc(1)

  def __create_update_thread(self, monitoruri, masteruri, delayed_exec):
    upthread = UpdateThread(monitoruri, masteruri, delayed_exec)
    self.__updateThreads[masteruri] = upthread
    upthread.update_signal.connect(self._on_master_info)
    upthread.master_errors_signal.connect(self._on_master_errors)
    upthread.error_signal.connect(self._on_error)
    upthread.start()

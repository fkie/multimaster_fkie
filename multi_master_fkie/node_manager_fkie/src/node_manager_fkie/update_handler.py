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

import threading
from PySide import QtCore

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

  def __init__(self):
    QtCore.QObject.__init__(self)
    self.__updateThreads = {}
    self.__requestedUpdates = {}
    self._lock = threading.RLock()

  def requestMasterInfo(self, masteruri, monitoruri):
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
    '''
    try:
      self._lock.acquire(True)
      if (self.__updateThreads.has_key(masteruri)):
        self.__requestedUpdates[masteruri] = monitoruri
      else:
        self.__create_update_thread(monitoruri, masteruri)
#        from urlparse import urlparse
#        om = urlparse(masteruri)
    except:
      pass
    finally:
      self._lock.release()

  def _on_master_info(self, minfo):
    self.master_info_signal.emit(minfo)
    self.__handle_requests(minfo.masteruri)

  def _on_error(self, masteruri, error):
    self.__handle_requests(masteruri)
    
  def __handle_requests(self, masteruri):
    self._lock.acquire(True)
    try:
      thread = self.__updateThreads.pop(masteruri)
      del thread
      monitoruri = self.__requestedUpdates.pop(masteruri)
    except KeyError:
#      import traceback
#      print traceback.format_exc()
      pass
    except:
      import traceback
      print traceback.format_exc()
    else:
      self.__create_update_thread(monitoruri, masteruri)
    finally:
      self._lock.release()

  def __create_update_thread(self, monitoruri, masteruri):
    upthread = UpdateThread(monitoruri, masteruri)
    self.__updateThreads[masteruri] = upthread
    upthread.update_signal.connect(self._on_master_info)
    upthread.error_signal.connect(self._on_error)
    upthread.start()

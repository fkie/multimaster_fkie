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
import threading

from fkie_master_discovery.master_info import MasterInfo
from fkie_node_manager.update_thread import UpdateThread


class UpdateHandler(QObject):
    '''
    A class to retrieve the state about ROS master from remote discovery node and
    publish it be sending a QT signal. To retrieve the state a new thread will be
    created.
    '''
    master_info_signal = Signal(MasterInfo)
    '''
  :ivar: master_info_signal is a signal, which is emitted, if a new
  U{fkie_master_discovery.MasterInfo<http://docs.ros.org/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>} is retrieved.
  '''
    master_errors_signal = Signal(str, list)
    '''
  :ivar: master_errors_signal is a signal (masteruri, error list) with errors which
  are occured on remote master_discovery.
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

    def __init__(self):
        QObject.__init__(self)
        self.__updateThreads = {}
        self.__requestedUpdates = {}
        self._lock = threading.RLock()

    def stop(self):
        with self._lock:
            if len(self.__updateThreads) > 0:
                print("  Shutdown update threads...")
                self.__requestedUpdates.clear()
                for _, thread in self.__updateThreads.items():
                    thread.join(3)
                print("  Update threads are off!")

    def requestMasterInfo(self, masteruri, monitoruri, delayed_exec=0.0):
        '''
        This method starts a thread to get the informations about the ROS master by
        the given RCP uri of the master_discovery node. If all informations are
        retrieved, a C{master_info_signal} of this class will be emitted. If for given
        masteruri a thread is already running, it will be inserted to the requested
        updates. For the same masteruri only one requested update can be stored.
        On update error the requested update will be ignored.
        This method is thread safe.

        :param str masteruri: the URI of the remote ROS master
        :param str monitoruri: the URI of the monitor RPC interface of the master_discovery node
        :param float delayed_exec: Delay the execution of the request for given seconds.
        '''
        with self._lock:
            try:
                if masteruri in self.__updateThreads:
                    self.__requestedUpdates[masteruri] = (monitoruri, delayed_exec)
                else:
                    self.__create_update_thread(monitoruri, masteruri, delayed_exec)
            except Exception:
                pass

    def _on_master_info(self, minfo):
        self.master_info_signal.emit(minfo)
        self.__handle_requests(minfo.masteruri)

    def _on_master_errors(self, masteruri, error_list):
        self.master_errors_signal.emit(masteruri, error_list)

    def _on_timediff(self, masteruri, timediff):
        self.timediff_signal.emit(masteruri, timediff)

    def _on_username(self, masteruri, username):
        self.username_signal.emit(masteruri, username)

    def _on_error(self, masteruri, error):
        self.error_signal.emit(masteruri, error)
        self.__handle_requests(masteruri)

    def __handle_requests(self, masteruri):
        with self._lock:
            try:
                thread = self.__updateThreads.pop(masteruri)
                thread.update_signal.disconnect(self._on_master_info)
                thread.master_errors_signal.disconnect(self._on_master_errors)
                thread.error_signal.disconnect(self._on_error)
                thread.timediff_signal.disconnect(self._on_timediff)
                thread.username_signal.disconnect(self._on_username)
                del thread
                monitoruri, delayed_exec = self.__requestedUpdates.pop(masteruri)
                self.__create_update_thread(monitoruri, masteruri, delayed_exec)
            except KeyError:
                pass
            except Exception:
                import traceback
                print(traceback.format_exc(1))

    def __create_update_thread(self, monitoruri, masteruri, delayed_exec):
        upthread = UpdateThread(monitoruri, masteruri, delayed_exec)
        self.__updateThreads[masteruri] = upthread
        upthread.update_signal.connect(self._on_master_info)
        upthread.master_errors_signal.connect(self._on_master_errors)
        upthread.error_signal.connect(self._on_error)
        upthread.timediff_signal.connect(self._on_timediff)
        upthread.username_signal.connect(self._on_username)
        upthread.start()

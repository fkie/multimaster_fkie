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


class LaunchServerHandler(QObject):
    '''
    A class to retrieve the state of launch servers. To retrieve the state a new
    thread will be created.
    '''
    launch_server_signal = Signal(str, int, list)
    '''
  @ivar: launch_server_signal is a signal (serveruri, pid, nodes), which is emitted, if a info from
  launch server was successful retrieved.
  '''
    error_signal = Signal(str, str)
    '''
  @ivar: error_signal is a signal (serveruri, error message), which is emitted,
  if an error while retrieving a launch server info was occurred.
  '''

    def __init__(self):
        QObject.__init__(self)
        self.__updateThreads = {}
        self.__requestedUpdates = {}
        self._lock = threading.RLock()

    def stop(self):
        if len(self.__updateThreads) > 0:
            print("  Shutdown launch update threads...")
            self.__requestedUpdates.clear()
            with self._lock:
                for _, thread in self.__updateThreads.items():
                    thread.launch_server_signal.disconnect()
                    thread.error_signal.disconnect()
            print("  Launch update threads are off!")

    def updateLaunchServerInfo(self, serveruri, delayed_exec=0.0):
        '''
        This method starts a thread to get the informations about the launch server by
        the given RCP uri of the launch server. If all informations are
        retrieved, a C{launch_server_signal} of this class will be emitted. If for given
        serveruri a thread is already running, it will be inserted to the requested
        updates. For the same serveruri only one requested update can be stored.
        On update error the requested update will be ignored.
        This method is thread safe.

        @param serveruri: the URI of the remote launch server
        @type serveruri: C{str}
        @param delayed_exec: Delay the execution of the request for given seconds.
        @type delayed_exec: C{float}
        '''
        with self._lock:
            try:
                if serveruri in self.__updateThreads:
                    self.__requestedUpdates[serveruri] = delayed_exec
                else:
                    self.__create_update_thread(serveruri, delayed_exec)
            except Exception:
                pass

    def _on_launch_server_info(self, serveruri, pid, nodes):
        self.launch_server_signal.emit(serveruri, pid, nodes)
        self.__handle_requests(serveruri)

    def _on_error(self, serveruri, error):
        self.error_signal.emit(serveruri, error)
        self.__handle_requests(serveruri)

    def __handle_requests(self, serveruri):
        with self._lock:
            try:
                thread = self.__updateThreads.pop(serveruri)
                del thread
                delayed_exec = self.__requestedUpdates.pop(serveruri)
                self.__create_update_thread(serveruri, delayed_exec)
            except KeyError:
                pass
            except Exception:
                import traceback
                print(traceback.format_exc(2))

    def __create_update_thread(self, serveruri, delayed_exec):
        upthread = LaunchServerUpdateThread(serveruri, delayed_exec)
        self.__updateThreads[serveruri] = upthread
        upthread.launch_server_signal.connect(self._on_launch_server_info)
        upthread.error_signal.connect(self._on_error)
        upthread.start()


class LaunchServerUpdateThread(QObject, threading.Thread):
    '''
    A thread to retrieve the list of pid and nodes from launch server and publish
    it by sending a QT signal.
    '''
    launch_server_signal = Signal(str, int, list)
    error_signal = Signal(str, str)

    def __init__(self, launch_serveruri, delayed_exec=0.0, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._launch_serveruri = launch_serveruri
        self._delayed_exec = delayed_exec
        self.setDaemon(True)

    def run(self):
        '''
        '''
        try:
            delay = self._delayed_exec + 0.5 + random.random()
            time.sleep(delay)
            socket.setdefaulttimeout(25)
            server = xmlrpcclient.ServerProxy(self._launch_serveruri)
            _, _, pid = server.get_pid()  # _:=code, msg
            _, _, nodes = server.get_node_names()  # _:=code, msg
            self.launch_server_signal.emit(self._launch_serveruri, pid, nodes)
        except Exception:
            import traceback
#      print traceback.print_exc()
            formatted_lines = traceback.format_exc(1).splitlines()
            rospy.logwarn("Connection to launch server @ %s failed:\n\t%s", str(self._launch_serveruri), formatted_lines[-1])
            # 'print "request failed", self._monitoruri
            self.error_signal.emit(self._launch_serveruri, formatted_lines[-1])
        finally:
            if socket is not None:
                socket.setdefaulttimeout(None)

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
import time

import rospy

import node_manager_fkie as nm
from node_manager_fkie.common import grpc_join, grpc_url_from_path, utf8


class LauncherThreaded(QObject):
    '''
    A class to retrieve the list of launch files and their nodes loaded on node manager daemon.
    The received list will be published by sending a QT signal. To retrieve
    the loaded launches a new thread will be created.
    '''
    update_signal = Signal(str, list)
    '''
      update_signal is a signal, which is emitted, if a new list with launch files is
      retrieved. The signal has the URI of the node manager daemon and
      a list with node_manager_daemon_fkie.launch_description.LaunchDescription.
    '''
    err_signal = Signal(str, Exception)
    '''
      err_signal is a signal, which is emitted on errors. The signal has the URI of the
      node manager daemon and Exception.
    '''

    def __init__(self):
        QObject.__init__(self)
        self.__get_info_threads = {}
        self._lock = threading.RLock()

    def stop(self):
        print "    Shutdown default config update threads..."
        for _, service in self.__get_info_threads.iteritems():
            service.join(3)
        print "    Default config update threads are off!"

    def update_info(self, url, delay_exec=0.0):
        '''
        This method starts a thread to get the informations about loaded configurations on node_manager_daemon.
        If all informations are retrieved, a C{node_list_signal} of
        this class will be emitted. If a thread for update info is
        already running, the request will be ignored.
        This method is thread safe.

        :param str url: the URI of the node manager daemon
        :param float delay_exec: delayd the execution
        '''
        with self._lock:
            clean_url = grpc_url_from_path(url)
            if clean_url not in self.__get_info_threads:
                upthread = GetInfoThread(clean_url, delay_exec)
                upthread.update_signal.connect(self._on_info)
                upthread.err_signal.connect(self._on_err)
                self.__get_info_threads[clean_url] = upthread
                upthread.start()

    def _on_info(self, url, descriptions):
        with self._lock:
            try:
                thread = self.__get_info_threads.pop(url)
                del thread
            except KeyError:
                pass
        self.update_signal.emit(url, descriptions)

    def _on_err(self, url, msg):
        with self._lock:
            try:
                thread = self.__get_info_threads.pop(url)
                del thread
            except KeyError:
                pass
        self.err_signal.emit(url, msg)


class GetInfoThread(QObject, threading.Thread):
    '''
    A thread to to retrieve the list of nodes from the default configuration
    service and publish it by sending a QT signal.
    '''
    update_signal = Signal(str, list)
    err_signal = Signal(str, Exception)

    def __init__(self, url, delay_exec=0.0, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._url = url
        self._delay_exec = delay_exec
        self.setDaemon(True)

    def run(self):
        if self._url:
            try:
                if self._delay_exec > 0:
                    time.sleep(self._delay_exec)
                launch_descriptions = nm.nmd().get_nodes(self._url)
                for ld in launch_descriptions:
                    ld.path = grpc_join(self._url, ld.path)
                self.update_signal.emit(self._url, launch_descriptions)
            except Exception as err:
                # import traceback
                # lines = traceback.format_exc(1).splitlines()
                # rospy.logwarn("Error while retrieve launch info from %s: %s", utf8(self._url), utf8(lines[-1]))
                self.err_signal.emit(self._url, err)

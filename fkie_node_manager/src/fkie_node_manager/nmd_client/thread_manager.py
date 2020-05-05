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
from python_qt_binding.QtCore import QObject


class ThreadManager(QObject):
    '''
    Class to manage threads with request to remote grpc-server. This can avoid multiple same requests.
    '''

    def __init__(self):
        QObject.__init__(self)
        self._lock = threading.RLock()
        self._threads = {}

    def __del__(self):
        self._threads.clear()

    def has_thread(self, thread_id):
        try:
            with self._lock:
                return self._threads[thread_id].is_alive()
        except Exception:
            pass
        return False

    def start_thread(self, thread_id, target, args=()):
        '''
        Starts a new thread to execute a callable object given by target.
        Avoids the start of new thread if one with same thread_id currently active.

        :param str thread_id: thread identification string determine by caller.
        :param object target: callable object to be invoked by start of new thread
        :param tule args: the argument tuple for the target invocation. Defaults to ().
        '''
        if not rospy.is_shutdown() and not self.has_thread(thread_id):
            with self._lock:
                thread = threading.Thread(target=target, args=args)
                thread.setDaemon(True)
                self._threads[thread_id] = thread
                thread.start()
            return True
        return False

    def finished(self, thread_id):
        '''
        Removes a thread with given thread_id from the managed list.

        :param str thread_id: thread identification string used while start_thread().
        :raise KeyError: if thread_id does not exists.
        '''
        with self._lock:
            if thread_id in self._threads:
                del self._threads[thread_id]

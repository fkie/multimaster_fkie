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


from python_qt_binding.QtCore import QObject, Signal
import rospy

from .thread_manager import ThreadManager
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.grpc_helper import remote


class ChannelInterface(QObject):

    error = Signal(str, str, str, Exception)
    '''
    :ivar  str,str,str,Exception error: error is a signal, which is emitted on errors {method, url, path, Exception}.
    '''

    def __init__(self):
        QObject.__init__(self)
        self._threads = ThreadManager()

    def open_channel(self, uri):
        channel = remote.open_channel(uri)
        if channel is None:
            raise Exception("Node manager daemon '%s' not reachable" % uri)
        return channel

    def close_channel(self, channel, uri):
        if channel is not None:
            Log.debug("close channel to %s" % uri)
            channel.close()

    def clear_cache(self, grpc_path=''):
        pass

    def stop(self):
        self.clear_cache()
        del self._threads

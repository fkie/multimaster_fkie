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

import fkie_node_manager_daemon.remote as remote
from .file_channel import FileChannel
from .launch_channel import LaunchChannel
from .monitor_channel import MonitorChannel
from .screen_channel import ScreenChannel
from .settings_channel import SettingsChannel
from .version_channel import VersionChannel


class NmdClient(QObject):

    error = Signal(str, str, str, Exception)
    '''
    :ivar  str,str,str,Exception error: error is a signal, which is emitted on errors {method, url, path, Exception}.
    '''

    def __init__(self):
        QObject.__init__(self)
        self._channels = []
        self.file = FileChannel()
        self.file.error.connect(self.on_error)
        self._channels.append(self.file)
        self.launch = LaunchChannel()
        self.launch.error.connect(self.on_error)
        self._channels.append(self.launch)
        self.monitor = MonitorChannel()
        self.monitor.error.connect(self.on_error)
        self._channels.append(self.monitor)
        self.screen = ScreenChannel()
        self.screen.error.connect(self.on_error)
        self._channels.append(self.screen)
        self.settings = SettingsChannel()
        self.settings.error.connect(self.on_error)
        self._channels.append(self.settings)
        self.version = VersionChannel()
        self.version.error.connect(self.on_error)
        self._channels.append(self.version)

    def stop(self):
        print("clear grpc channels...")
        for channel in self._channels:
            channel.stop()
        remote.clear_channels()
        print("clear grpc channels...ok")
        self.clear_cache()
        del self._channels[:]

    def clear_cache(self, grpc_path=''):
        for channel in self._channels:
            channel.clear_cache(grpc_path)

    def on_error(self, method, url, path, exception):
        self.error.emit(method, url, path, exception)

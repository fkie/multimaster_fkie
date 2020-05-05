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
from python_qt_binding.QtCore import Signal

import fkie_node_manager_daemon.settings_stub as scstub
from fkie_node_manager_daemon import url as nmdurl

from .channel_interface import ChannelInterface


class SettingsChannel(ChannelInterface):

    yaml_config_signal = Signal(str, str)
    '''
      :ivar str,str yaml_config_signal: signal emit YAML configuration from daemon {YAML string, grpc_url}.
    '''

    def clear_cache(self, grpc_url=''):
        pass

    def get_settings_manager(self, uri='localhost:12321'):
        channel = self.get_insecure_channel(uri)
        return scstub.SettingsStub(channel), channel

    def get_config_threaded(self, grpc_url='grpc://localhost:12321'):
        self._threads.start_thread("gcfgt_%s" % grpc_url, target=self.get_config, args=(grpc_url, True))

    def get_config(self, grpc_url='grpc://localhost:12321', threaded=False):
        rospy.logdebug("get config from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_settings_manager(uri)
        try:
            yaml_cfg = sm.get_config()
            if threaded:
                self.yaml_config_signal.emit(yaml_cfg, grpc_url)
                self._threads.finished("gcfgt_%s" % grpc_url)
            return yaml_cfg
        except Exception as e:
            self.error.emit("get_config", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)

    def set_config(self, grpc_url='grpc://localhost:12321', data=''):
        rospy.logdebug("set config to %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_settings_manager(uri)
        sm.set_config(data)
        self.close_channel(channel, uri)

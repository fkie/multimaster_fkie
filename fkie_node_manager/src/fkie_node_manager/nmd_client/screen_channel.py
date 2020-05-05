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

import fkie_node_manager_daemon.screen_stub as sstub
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon.common import sizeof_fmt

from .channel_interface import ChannelInterface


class ScreenChannel(ChannelInterface):

    multiple_screens = Signal(str, dict)
    '''
    :ivar str,dict multiple_screens: this signal is emitted if new multiple screen are detected {grpc_url, {node_name: [screen_session]}}.
    '''
    log_dir_size_signal = Signal(str, float)
    '''
      :ivar str,int log_dir_size_signal: signal emitted on log_dir size was {grpc_url, log_dir size}.
    '''

    def clear_cache(self, grpc_url=''):
        pass

    def get_screen_manager(self, uri='localhost:12321'):
        channel = self.get_insecure_channel(uri)
        return sstub.ScreenStub(channel), channel

    def get_all_screens(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get all screens from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_screen_manager(uri)
        screens = sm.all_screens()
        self.close_channel(channel, uri)
        return screens

    def get_screens(self, grpc_url='grpc://localhost:12321', node=''):
        rospy.logdebug("get screen from %s for %s" % (grpc_url, node))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_screen_manager(uri)
        screens = sm.screens(node)
        self.close_channel(channel, uri)
        return screens

    def multiple_screens_threaded(self, grpc_url='grpc://localhost:12321'):
        '''
        The existence of multiple screens for one node can lead to failures.
        This method starts a thread to request all nodes with multiple screens.
        On finish this method emit a qt-signal of type NmdClient.multiple_screens.

        :param str grpc_url: the url for grpc-server
        '''
        self._threads.start_thread("mst_%s" % grpc_url, target=self._multiple_screens, args=(grpc_url,))

    def _multiple_screens(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get multiple screens from %s" % (grpc_url))
        try:
            uri, _ = nmdurl.split(grpc_url)
            sm, channel = self.get_screen_manager(uri)
            result = sm.multiple_screens()
            self.multiple_screens.emit(grpc_url, result)
        except Exception as e:
            self.error.emit("get_multiple_screens", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("mst_%s" % grpc_url)

    def rosclean(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("clear log directory on %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_screen_manager(uri)
        result = sm.rosclean()
        self.close_channel(channel, uri)
        return result

    def wipe_screens(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("wipe screens on %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_screen_manager(uri)
        sm.wipe_screens()
        self.close_channel(channel, uri)

    def log_dir_size_threaded(self, grpc_url='grpc://localhost:12321'):
        '''
        Determine the size of ROS log_dir.

        :param str grpc_url: the url for grpc-server
        '''
        self._threads.start_thread("lds_%s" % grpc_url, target=self._log_dir_size, args=(grpc_url,))

    def _log_dir_size(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get log_dir size on %s" % (grpc_url))
        try:
            uri, _ = nmdurl.split(grpc_url)
            sm, channel = self.get_screen_manager(uri)
            log_dir_size = sm.log_dir_size()
            rospy.logdebug("log_dir size on %s: %s" % (grpc_url, sizeof_fmt(log_dir_size)))
            self.log_dir_size_signal.emit(grpc_url, log_dir_size)
        except Exception as e:
            self.error.emit("log_dir_size", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("lds_%s" % grpc_url)

    def delete_log(self, grpc_url='grpc://localhost:12321', nodes=[]):
        rospy.logdebug("delete logs on %s for %s" % (grpc_url, nodes))
        uri, _ = nmdurl.split(grpc_url)
        sm, channel = self.get_screen_manager(uri)
        result = sm.delete_log(nodes)
        self.close_channel(channel, uri)
        return result

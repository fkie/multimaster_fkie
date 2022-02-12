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
from diagnostic_msgs.msg import DiagnosticArray
from python_qt_binding.QtCore import Signal

import fkie_node_manager_daemon.monitor_stub as mstub
from fkie_node_manager_daemon import url as nmdurl

from .channel_interface import ChannelInterface


class MonitorChannel(ChannelInterface):

    system_diagnostics_signal = Signal(DiagnosticArray, str)
    '''
      :ivar DiagnosticArray,str system_diagnostics_signal: signal emit system (nmd) diagnostic messages {DiagnosticArray, grpc_url}.
    '''
    remote_diagnostics_signal = Signal(DiagnosticArray, str)
    '''
      :ivar DiagnosticArray,str remote_diagnostics_signal: signal emit diagnostic messages from remote system {DiagnosticArray, grpc_url}.
    '''
    username_signal = Signal(str, str)
    '''
      :ivar str,str user_signal: signal emit current user for remote system {user name, grpc_url}.
    '''


    def clear_cache(self, grpc_url=''):
        pass

    def get_monitor_manager(self, uri='localhost:12321'):
        channel = self.get_insecure_channel(uri)
        return mstub.MonitorStub(channel), channel

    def get_system_diagnostics_threaded(self, grpc_url='grpc://localhost:12321'):
        self._threads.start_thread("gmsdt_%s" % grpc_url, target=self.get_system_diagnostics, args=(grpc_url, True))

    def get_system_diagnostics(self, grpc_url='grpc://localhost:12321', threaded=False):
        rospy.logdebug("get system diagnostics from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        vm, channel = self.get_monitor_manager(uri)
        try:
            diagnostic_array = vm.get_system_diagnostics()
            if threaded:
                self.system_diagnostics_signal.emit(diagnostic_array, grpc_url)
                self._threads.finished("gmsdt_%s" % grpc_url)
            return diagnostic_array
        except Exception as e:
            self.error.emit("get_system_diagnostics", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)

    def get_diagnostics_threaded(self, grpc_url='grpc://localhost:12321'):
        self._threads.start_thread("gmdt_%s" % grpc_url, target=self.get_diagnostics, args=(grpc_url, True))

    def get_diagnostics(self, grpc_url='grpc://localhost:12321', threaded=False):
        rospy.logdebug("get diagnostics from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        vm, channel = self.get_monitor_manager(uri)
        try:
            diagnostic_array = vm.get_diagnostics()
            if threaded:
                self.remote_diagnostics_signal.emit(diagnostic_array, grpc_url)
                self._threads.finished("gmdt_%s" % grpc_url)
            return diagnostic_array
        except Exception as e:
            self.error.emit("get_diagnostics", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)

    def kill_process(self, pid, grpc_url='grpc://localhost:12321'):
        if pid is not None:
            rospy.logdebug("kill process %d on %s" % (pid, grpc_url))
            uri, _ = nmdurl.split(grpc_url)
            vm, channel = self.get_monitor_manager(uri)
            try:
                vm.kill_process(pid)
            except Exception as e:
                self.error.emit("kill_process", "grpc://%s" % uri, "", e)
            finally:
                self.close_channel(channel, uri)

    def get_user_threaded(self, grpc_url='grpc://localhost:12321'):
        self._threads.start_thread("gut_%s" % grpc_url, target=self.get_user, args=(grpc_url,))

    def get_user(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get user from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        vm, channel = self.get_monitor_manager(uri)
        user = ''
        try:
            user = vm.get_user()
            self.username_signal.emit(user, grpc_url)
        except Exception as e:
            self.error.emit("get_user", "grpc://%s" % uri, "", e)
        finally:
            self.close_channel(channel, uri)
        return user

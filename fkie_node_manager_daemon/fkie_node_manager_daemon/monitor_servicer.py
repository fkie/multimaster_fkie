# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from datetime import datetime
import os
import rclpy
import signal
import subprocess
import time
from fkie_node_manager_daemon.monitor import Service# , grpc_msg
import fkie_node_manager_daemon as nmd


class MonitorServicer():

    def __init__(self, settings):
        nmd.rosnode.get_logger().info("Create monitor servicer")
        #mgrpc.MonitorServiceServicer.__init__(self)
        self._monitor = Service(settings)

    def stop(self):
        self._monitor.stop()

    def level2bytes(self, levels:list):
        return [int.to_bytes(lvl, length=1, byteorder='big') for lvl in levels]

    def GetSystemDiagnostics(self, request, context):
        rosmsg = self._monitor.get_system_diagnostics(self.level2bytes(request.level), request.timestamp)
        #return grpc_msg(rosmsg)

    def GetSystemWarnings(self, request, context):
        rosmsg = self._monitor.get_system_diagnostics(2, 0)
        #return grpc_msg(rosmsg)

    def GetDiagnostics(self, request, context):
        rosmsg = self._monitor.get_diagnostics(self.level2bytes(request.level), request.timestamp)
        #return grpc_msg(rosmsg)

    def GetWarnings(self, request, context):
        rosmsg = self._monitor.get_diagnostics(2, 0)
        #return grpc_msg(rosmsg)

    def KillProcess(self, request, context):
        os.kill(request.pid, signal.SIGKILL)
        #reply = mmsg.Empty()
        #return reply

    def SetTime(self, request, context):
        dtime = datetime.fromtimestamp(request.timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        nmd.rosnode.get_logger().info('Set time: %s' % args)
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            raise Exception(result_err)
        #reply = mmsg.Timestamp()
        #reply.timestamp = time.time()
        #return reply

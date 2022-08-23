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


from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fkie_node_manager_daemon.time_helpers import rostime2float
from fkie_node_manager_daemon.time_helpers import float2rostime

from .service import Service


def grpc_msg(rosmsg):
    try:
        pass
        # result = mmsg.DiagnosticArray()
        # result.timestamp = rostime2float(Time.from_msg(rosmsg.header.stamp))
        # stats = []
        # for stat in rosmsg.status:
        #     ds = mmsg.DiagnosticStatus()
        #     ds.level = int.from_bytes(stat.level, byteorder='big')
        #     ds.name = stat.name
        #     ds.message = stat.message
        #     ds.hardware_id = stat.hardware_id
        #     values = []
        #     for val in stat.values:
        #         dv = mmsg.KeyValue()
        #         dv.key = val.key
        #         dv.value = str(val.value)
        #         values.append(dv)
        #     ds.values.extend(values)
        #     stats.append(ds)
        # result.status.extend(stats)
        # return result
    except Exception as _err:
        import traceback
        raise Exception(traceback.format_exc())


def ros_msg(grpcmsg):
    try:
        result = DiagnosticArray()
        result.header.stamp = float2rostime(grpcmsg.timestamp).to_msg()
        for stat in grpcmsg.status:
            ds = DiagnosticStatus()
            ds.level = int.to_bytes(stat.level, length=1, byteorder='big')
            ds.name = stat.name
            ds.message = stat.message
            ds.hardware_id = stat.hardware_id
            for val in stat.values:
                dv = KeyValue()
                dv.key = val.key
                dv.value = val.value
                ds.values.append(dv)
            result.status.append(ds)
        return result
    except Exception as _err:
        import traceback
        raise Exception(traceback.format_exc())

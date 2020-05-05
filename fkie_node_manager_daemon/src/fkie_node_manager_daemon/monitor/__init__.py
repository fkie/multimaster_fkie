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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import fkie_multimaster_msgs.grpc.monitor_pb2 as mmsg
from fkie_node_manager_daemon.common import utf8

from .service import Service


def grpc_msg(rosmsg):
    try:
        result = mmsg.DiagnosticArray()
        result.timestamp = rosmsg.header.stamp.secs
        stats = []
        for stat in rosmsg.status:
            ds = mmsg.DiagnosticStatus()
            ds.level = stat.level
            ds.name = stat.name
            ds.message = stat.message
            ds.hardware_id = stat.hardware_id
            values = []
            for val in stat.values:
                dv = mmsg.KeyValue()
                dv.key = val.key
                dv.value = utf8(val.value)
                values.append(dv)
            ds.values.extend(values)
            stats.append(ds)
        result.status.extend(stats)
        return result
    except Exception as _err:
        import traceback
        raise Exception(traceback.format_exc())


def ros_msg(grpcmsg):
    try:
        result = DiagnosticArray()
        result.header.stamp = rospy.Time(grpcmsg.timestamp)
        for stat in grpcmsg.status:
            ds = DiagnosticStatus()
            ds.level = stat.level
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

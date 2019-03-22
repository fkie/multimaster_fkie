import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import multimaster_msgs_fkie.grpc.monitor_pb2 as mmsg
from node_manager_daemon_fkie.common import utf8

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

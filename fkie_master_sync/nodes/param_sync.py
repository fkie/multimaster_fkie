#!/usr/bin/env python
# Original code from https://github.com/jhu-lcsr-forks/fkie_multimaster/tree/param-sync
# adapt to change only local ROS Parameter Server

import rospy

from fkie_master_discovery.common import masteruri_from_master
from fkie_multimaster_msgs.msg import MasterState

def master_changed(msg, cb_args):
    param_cache, local_master, __add_ns = cb_args
    local_name = ''
    if local_master:
        local_name = local_master[0]
    if msg.master.uri != masteruri_from_master() and local_name in param_cache:
        master_to = rospy.MasterProxy(masteruri_from_master())
        master_from = rospy.MasterProxy(msg.master.uri)
        rospy.logdebug("Getting params from {}...".format(msg.master.uri))
        params_from = master_from['/']
        rospy.logdebug("Got {} params.".format(len(msg.master.uri)))
        if local_name in params_from:
            del params_from[local_name]
        if '/'+local_name in params_from:
            del params_from['/'+local_name]
        rospy.logdebug("Syncing params from {} to {}...".format(msg.master.name, local_name))
        if __add_ns:
            _ns = msg.master.name
        else:
            _ns = ''
        if param_cache.get(_ns, None) != params_from:
            param_cache[_ns] = params_from
            master_to['/'+_ns] = params_from
            rospy.logdebug("Done syncing params from {} to {}.".format(msg.master.name, local_name))
        else:
            rospy.logdebug("Params have not changed from {} to {}.".format(msg.master.name, local_name))
    else:
        local_name = msg.master.name
        local_master.append(local_name)
        master_from = rospy.MasterProxy(msg.master.uri)
        rospy.logdebug("Getting params from local {}...".format(msg.master.uri))
        param_cache[local_name] = master_from['/']
        rospy.logdebug("Got {} params.".format(len(msg.master.uri)))


def main():
    rospy.init_node('param_sync', log_level=rospy.DEBUG)

    param_cache = dict()
    local_master = list()
    masteruri_from_master()

    __add_ns = rospy.get_param('~add_ns', True)
    sub = rospy.Subscriber('master_discovery/changes', MasterState, master_changed, callback_args=(param_cache, local_master, __add_ns))

    rospy.spin()

if __name__ == '__main__':
    main()

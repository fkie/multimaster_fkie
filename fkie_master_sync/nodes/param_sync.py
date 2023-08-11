#!/usr/bin/env python
# Original code from https://github.com/jhu-lcsr-forks/fkie_multimaster/tree/param-sync
# adapt to change only local ROS Parameter Server

import rospy

from fkie_multimaster_msgs.msg import MasterState
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.system import ros1_masteruri


def master_changed(msg, cb_args):
    param_cache, local_master, __add_ns, __ignore, __only = cb_args
    local_name = ''
    if local_master:
        local_name = local_master[0]
    if msg.master.uri != ros1_masteruri.from_master() and local_name in param_cache:
        master_to = rospy.MasterProxy(ros1_masteruri.from_master())
        master_from = rospy.MasterProxy(msg.master.uri)
        Log.debug("Getting params from {}...".format(msg.master.uri))
        params_from = master_from.getParam('/')[2]
        if not __add_ns:
            for key in ['run_id', 'rosversion', 'roslaunch', 'rosdistro', 'master_sync', 'master_discovery', 'capabilities', 'mastername', 'robots']:
                try:
                    del params_from[key]
                except Exception:
                    pass
        for key in __ignore + [local_name, '/'+local_name]:
            try:
                del params_from[key]
            except Exception:
                pass
        if __only:
            for key in params_from.keys():
                if key not in __only:
                    del params_from[key]
        Log.debug("Syncing params from {} to {}...".format(
            msg.master.name, local_name))
        if __add_ns:
            _ns = msg.master.name
        else:
            _ns = ''
        Log.debug("Got {} params.".format(len(params_from)))
        if param_cache.get(_ns, None) != params_from:
            param_cache[_ns] = params_from
            for key, value in params_from.items():
                master_to.setParam('/'+_ns+key, value)
            Log.debug("Done syncing params from {} to {}.".format(
                msg.master.name, local_name))
        else:
            Log.debug("Params have not changed from {} to {}.".format(
                msg.master.name, local_name))
    else:
        local_name = msg.master.name
        local_master.append(local_name)
        master_from = rospy.MasterProxy(msg.master.uri)
        Log.debug(
            "Getting params from local {}...".format(msg.master.uri))
        param_cache[local_name] = master_from.getParam('/')[2]
        Log.debug("Got {} local params.".format(
            len(param_cache[local_name])))


def main():
    rospy.init_node('param_sync', log_level=rospy.DEBUG)

    param_cache = dict()
    local_master = list()
    ros1_masteruri.from_master()

    __add_ns = rospy.get_param('~add_ns', True)
    __ignore = rospy.get_param('~ignore', [])
    __only = rospy.get_param('~only', [])
    sub = rospy.Subscriber('master_discovery/changes', MasterState, master_changed,
                           callback_args=(param_cache, local_master, __add_ns, __ignore, __only))

    rospy.spin()


if __name__ == '__main__':
    main()

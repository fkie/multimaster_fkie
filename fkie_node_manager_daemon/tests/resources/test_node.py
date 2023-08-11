#!/usr/bin/env python

import os
import signal
import sys
import traceback

import rospy
from fkie_multimaster_pylib.logging.logging import Log


rospy.init_node("launch_test_node", log_level=rospy.DEBUG)
try:
    print("os.environ:", os.environ)
    if 'ROS_NAMESPACE' not in os.environ:
        raise Exception("'ROS_NAMESPACE' not in environment of the node")
except Exception:
    # on load error the process will be killed to notify user in node_manager
    # about error
    Log.warn("%s", traceback.format_exc())
    sys.stdout.write(traceback.format_exc())
    sys.stdout.flush()
    os.kill(os.getpid(), signal.SIGKILL)

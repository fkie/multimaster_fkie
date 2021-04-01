#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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

import os
import signal
import sys

import roslib
import rospy
import time

try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse


# MCAST_GROUP = "ff02::1"# ipv6 multicast group
MCAST_GROUP = "226.0.0.0"  # ipv4 multicast group
MCAST_PORT = 11511
PROCESS_NAME = "master_discovery"


def get_default_rtcp_port(zeroconf=False):
    try:
        from fkie_master_discovery.common import masteruri_from_ros
        masteruri = masteruri_from_ros()
        # rospy.loginfo("ROS Master URI: %s", masteruri)
        return urlparse(masteruri).port + (600 if zeroconf else 300)
    except:
        import traceback
        print(traceback.format_exc())
        return 11911 if zeroconf else 11611


def set_terminal_name(name):
    '''
    Change the terminal name.
    @param name: New name of the terminal
    @type name:  str
    '''
    sys.stdout.write("\x1b]2;%s\x07" % name)


def set_process_name(name):
    '''
    Change the process name.
    @param name: New process name
    @type name:  str
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except Exception:
        try:
            import setproctitle
            setproctitle.setproctitle(name)
        except Exception:
            pass


def is_port_in_use(port):
    import socket, errno
    result = False
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('localhost', port))
    except socket.error as e:
        if e.errno == errno.EADDRINUSE:
            result = True
        else:
            # something else raised the socket.error exception
            print(e)
    s.close()
    return result


def wait_for_free_port():
    wait_index = 0
    rpc_port = get_default_rtcp_port()
    while wait_index < 12 and is_port_in_use(rpc_port):
        wait_index += 1
        if wait_index == 1:
            print('RPC port %d is already in use, is there another instance of master_discovery running?' % rpc_port)
        time.sleep(1)
    if wait_index > 1:
        # give time for shutdown other node
        time.sleep(3)


def main():
    '''
    Creates and runs the ROS node using multicast messages for discovering
    '''
    import fkie_master_discovery.master_discovery as master_discovery
    wait_for_free_port()
    # setup the loglevel
    try:
        log_level = getattr(rospy, rospy.get_param('/%s/log_level' % PROCESS_NAME, "INFO"))
    except Exception as e:
        print("Error while set the log level: %s\n->INFO level will be used!" % e)
        log_level = rospy.INFO
    rospy.init_node(PROCESS_NAME, log_level=log_level)
    set_terminal_name(PROCESS_NAME)
    set_process_name(PROCESS_NAME)
    mcast_group = rospy.get_param('~mcast_group', MCAST_GROUP)
    mcast_port = rospy.get_param('~mcast_port', MCAST_PORT)
    rpc_port = rospy.get_param('~rpc_port', get_default_rtcp_port())
    rpc_addr = rospy.get_param('~rpc_addr', '')
    try:
        discoverer = master_discovery.Discoverer(mcast_port, mcast_group, rpc_port, rpc_addr=rpc_addr)
        discoverer.start()
        rospy.spin()
        discoverer.finish()
    except Exception as e:
        import traceback
        rospy.logerr("%s\nError while start master_discovery: %s" % (traceback.format_exc(), str(e)))
        os.kill(os.getpid(), signal.SIGKILL)
        time.sleep(10)


def main_zeroconf():
    '''
    Creates and runs the ROS node using zeroconf/avahi for discovering
    '''
    import fkie_master_discovery.zeroconf as zeroconf
    PROCESS_NAME = "zeroconf"
    wait_for_free_port()
    # setup the loglevel
    try:
        log_level = getattr(rospy, rospy.get_param('/%s/log_level' % PROCESS_NAME, "INFO"))
    except Exception as e:
        print("Error while set the log level: %s\n->INFO level will be used!" % e)
        log_level = rospy.INFO
    rospy.init_node(PROCESS_NAME, log_level=log_level)
    set_terminal_name(rospy.get_name())
    set_process_name(rospy.get_name())
    mcast_port = rospy.get_param('~mcast_port', MCAST_PORT)
    rpc_port = rospy.get_param('~rpc_port', get_default_rtcp_port(True))
    discoverer = zeroconf.Discoverer(rpc_port, mcast_port - MCAST_PORT)
    discoverer.start()
    rospy.spin()

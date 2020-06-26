#!/usr/bin/env python

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
import shlex
import socket
import subprocess
import time
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import rospy
from rosgraph.network import get_local_addresses

from fkie_master_discovery.common import masteruri_from_ros
from fkie_master_discovery.common import get_hostname, get_port
from fkie_node_manager_daemon import host as nmdhost
from fkie_node_manager_daemon import screen


class StartException(Exception):
    pass


def get_ros_home():
    '''
    Returns the ROS HOME depending on ROS distribution API.

    :return: ROS HOME path
    :rtype: str
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            import roslib.rosenv
            return roslib.rosenv.get_ros_home()
        else:
            from rospkg import get_ros_home
            return get_ros_home()
    except Exception:
        from roslib import rosenv
        return rosenv.get_ros_home()


class Settings(object):
    LOG_VIEWER = "/usr/bin/less -fKLnQrSU"
    STARTER_SCRIPT = 'rosrun fkie_node_manager remote_nm.py'
    RESPAWN_SCRIPT = 'rosrun fkie_node_manager respawn'


class StartHandler(object):

    @classmethod
    def is_local(cls, hostname, wait=False):
        '''
        Test whether the given host name is the name of the local host or not.

        :param str hostname: the name or IP of the host
        :return: True if the hostname is local or None
        :rtype: bool
        :raise Exception: on errors while resolving host
        '''
        if (hostname is None):
            return True
        try:
            socket.inet_aton(hostname)
            local_addresses = ['localhost'] + get_local_addresses()
            # check 127/8 and local addresses
            result = hostname.startswith('127.') or hostname in local_addresses
            return result
        except socket.error:
            # the hostname must be resolved => do it in a thread
            if wait:
                result = cls.__is_local(hostname)
                return result
        return False

    @classmethod
    def __is_local(cls, hostname):
        try:
            machine_addr = socket.gethostbyname(hostname)
        except socket.gaierror:
            import traceback
            print(traceback.format_exc())
            return False
        local_addresses = ['localhost'] + get_local_addresses()
        # check 127/8 and local addresses
        result = machine_addr.startswith('127.') or machine_addr in local_addresses
        return result

    @classmethod
    def _prepareROSMaster(cls, masteruri):
        if not masteruri:
            masteruri = masteruri_from_ros()
        # start roscore, if needed
        try:
            if not os.path.isdir(screen.LOG_PATH):
                os.makedirs(screen.LOG_PATH)
            socket.setdefaulttimeout(3)
            master = xmlrpcclient.ServerProxy(masteruri)
            master.getUri(rospy.get_name())
        except Exception:
            # run a roscore
            master_host = get_hostname(masteruri)
            if cls.is_local(master_host, True):
                print("Start ROS-Master with %s ..." % masteruri)
                master_port = get_port(masteruri)
                new_env = dict(os.environ)
                new_env['ROS_MASTER_URI'] = masteruri
                ros_hostname = nmdhost.get_ros_hostname(masteruri)
                if ros_hostname:
                    new_env['ROS_HOSTNAME'] = ros_hostname
                cmd_args = '%s roscore --port %d' % (screen.get_cmd('/roscore--%d' % master_port), master_port)
                try:
                    subprocess.Popen(shlex.split(cmd_args), env=new_env)
                    # wait for roscore to avoid connection problems while init_node
                    result = -1
                    count = 1
                    while result == -1 and count < 11:
                        try:
                            print("  retry connect to ROS master %d/10" % count)
                            master = xmlrpcclient.ServerProxy(masteruri)
                            result, _, _ = master.getUri(rospy.get_name())  # _:=uri, msg
                        except Exception:
                            time.sleep(1)
                            count += 1
                    if count >= 11:
                        raise StartException('Cannot connect to the ROS-Master: ' + str(masteruri))
                except Exception as e:
                    import sys
                    sys.stderr.write("%s\n" % e)
                    raise
            else:
                raise Exception("ROS master '%s' is not reachable" % masteruri)
        finally:
            socket.setdefaulttimeout(None)

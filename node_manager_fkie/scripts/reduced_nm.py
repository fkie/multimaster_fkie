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
import xmlrpclib

from roslib.network import get_local_addresses
import rospy

from master_discovery_fkie.common import get_hostname, get_port


class StartException(Exception):
    pass


def get_ros_home():
    '''
    Returns the ROS HOME depending on ROS distribution API.
    @return: ROS HOME path
    @rtype: C{str}
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
    except:
        from roslib import rosenv
        return rosenv.get_ros_home()


def masteruri_from_ros():
    '''
    Returns the master URI depending on ROS distribution API.
    @return: ROS master URI
    @rtype: C{str}
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            import roslib.rosenv
            return roslib.rosenv.get_master_uri()
        else:
            import rosgraph
            return rosgraph.rosenv.get_master_uri()
    except:
        return os.environ['ROS_MASTER_URI']


def get_ros_hostname(url):
    '''
    Returns the host name used in a url, if it is a name. If it is an IP an
    empty string will be returned.

    @return: host or '' if url is an IP or invalid
    @rtype:  C{str}
    '''
    hostname = get_hostname(url)
    if hostname is not None:
        if hostname != 'localhost':
            if '.' not in hostname and ':' not in hostname:
                # ROS resolves the 'localhost' to local hostname
                local_hostname = 'localhost'
                try:
                    local_hostname = socket.gethostname()
                except:
                    pass
                if hostname != local_hostname:
                    return hostname
    return ''


class Settings(object):
    LOG_PATH = os.environ.get('ROS_LOG_DIR') if os.environ.get('ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
    LOG_VIEWER = "/usr/bin/less -fKLnQrSU"
    STARTER_SCRIPT = 'rosrun node_manager_fkie remote_nm.py'
    RESPAWN_SCRIPT = 'rosrun node_manager_fkie respawn'


class ScreenHandler(object):
    '''
    The class to handle the running screen sessions and create new sessions on
    start of the ROS nodes.
    '''

    LOG_PATH = os.environ.get('ROS_LOG_DIR') if os.environ.get('ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
    SCREEN = "/usr/bin/screen"
    SLASH_SEP = '_'

    @classmethod
    def createSessionName(cls, node=None):
        '''
        Creates a name for the screen session. All slash separators are replaced by
        L{SLASH_SEP}
        @param node: the name of the node
        @type node: C{str}
        @return: name for the screen session.
        @rtype: C{str}
        '''
#    package_name = str(package) if not package is None else ''
#    lanchfile_name = str(launchfile).replace('.launch', '') if not launchfile is None else ''
        node_name = str(node).replace('/', cls.SLASH_SEP) if node is not None else ''
#    result = ''.join([node_name, '.', package_name, '.', lanchfile_name])
        return node_name

    @classmethod
    def getScreenLogFile(cls, session=None, node=None):
        '''
        Generates a log file name for the screen session.
        @param session: the name of the screen session
        @type session: C{str}
        @return: the log file name
        @rtype: C{str}
        '''
        if session is not None:
            return os.path.join(cls.LOG_PATH, session + '.log')
        elif node is not None:
            return os.path.join(cls.LOG_PATH, cls.createSessionName(node) + '.log')
        else:
            return os.path.join(cls.LOG_PATH, 'unknown.log')

    @classmethod
    def getROSLogFile(cls, node):
        '''
        Generates a log file name of the ROS log.
        @param node: the name of the node
        @type node: C{str}
        @return: the ROS log file name
        @rtype: C{str}
        @todo: get the run_id from the ROS parameter server and search in this log folder
        for the log file (handle the node started using a launch file).
        '''
        if node is not None:
            return os.path.join(cls.LOG_PATH, node.strip(rospy.names.SEP).replace(rospy.names.SEP, '_') + '.log')
        else:
            return ''

    @classmethod
    def getScreenCfgFile(cls, session=None, node=None):
        '''
        Generates a configuration file name for the screen session.
        @param session: the name of the screen session
        @type session: C{str}
        @return: the configuration file name
        @rtype: C{str}
        '''
        if session is not None:
            return os.path.join(cls.LOG_PATH, session + '.conf')
        elif node is not None:
            return os.path.join(cls.LOG_PATH, cls.createSessionName(node) + '.conf')
        else:
            return os.path.join(cls.LOG_PATH, 'unknown.conf')

    @classmethod
    def getScreenPidFile(cls, session=None, node=None):
        '''
        Generates a PID file name for the screen session.
        @param session: the name of the screen session
        @type session: C{str}
        @return: the PID file name
        @rtype: C{str}
        '''
        if session is not None:
            return os.path.join(cls.LOG_PATH, session + '.pid')
        elif node is not None:
            return os.path.join(cls.LOG_PATH, cls.createSessionName(node) + '.pid')
        else:
            return os.path.join(cls.LOG_PATH, 'unknown.pid')

    @classmethod
    def getSceenCmd(cls, node):
        '''
        Generates a configuration file and return the command prefix to start the given node
        in a screen terminal.
        @param node: the name of the node
        @type node: C{str}
        @return: the command prefix
        @rtype: C{str}
        '''
        f = open(cls.getScreenCfgFile(node=node), 'w')
        f.write(''.join(["logfile ", cls.getScreenLogFile(node=node), "\n"]))
        f.write("logfile flush 0\n")
        f.write("defscrollback 10000\n")
        ld_library_path = os.getenv('LD_LIBRARY_PATH', '')
        if ld_library_path:
            f.write(' '.join(['setenv', 'LD_LIBRARY_PATH', ld_library_path, "\n"]))
        ros_etc_dir = os.getenv('ROS_ETC_DIR', '')
        if ros_etc_dir:
            f.write(' '.join(['setenv', 'ROS_ETC_DIR', ros_etc_dir, "\n"]))
        f.close()
        return ' '.join([cls.SCREEN, '-c', cls.getScreenCfgFile(node=node), '-L', '-dmS', cls.createSessionName(node=node)])


class StartHandler(object):

    @classmethod
    def is_local(cls, hostname, wait=False):
        '''
        Test whether the given host name is the name of the local host or not.
        @param hostname: the name or IP of the host
        @type hostname: C{str}
        @return: C{True} if the hostname is local or None
        @rtype: C{bool}
        @raise Exception: on errors while resolving host
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
            print traceback.format_exc()
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
            if not os.path.isdir(ScreenHandler.LOG_PATH):
                os.makedirs(ScreenHandler.LOG_PATH)
            socket.setdefaulttimeout(3)
            master = xmlrpclib.ServerProxy(masteruri)
            master.getUri(rospy.get_name())
        except:
            # run a roscore
            master_host = get_hostname(masteruri)
            if cls.is_local(master_host, True):
                print "Start ROS-Master with", masteruri, "..."
                master_port = get_port(masteruri)
                new_env = dict(os.environ)
                new_env['ROS_MASTER_URI'] = masteruri
                ros_hostname = get_ros_hostname(masteruri)
                if ros_hostname:
                    new_env['ROS_HOSTNAME'] = ros_hostname
                cmd_args = '%s roscore --port %d' % (ScreenHandler.getSceenCmd('/roscore--%d' % master_port), master_port)
                try:
                    subprocess.Popen(shlex.split(cmd_args), env=new_env)
                    # wait for roscore to avoid connection problems while init_node
                    result = -1
                    count = 1
                    while result == -1 and count < 11:
                        try:
                            print "  retry connect to ROS master", count, '/', 10
                            master = xmlrpclib.ServerProxy(masteruri)
                            result, _, _ = master.getUri(rospy.get_name())  # _:=uri, msg
                        except:
                            time.sleep(1)
                            count += 1
                    if count >= 11:
                        raise StartException('Cannot connect to the ROS-Master: ' + str(masteruri))
                except Exception as e:
                    import sys
                    print >> sys.stderr, e
                    raise
            else:
                raise Exception("ROS master '%s' is not reachable" % masteruri)
        finally:
            socket.setdefaulttimeout(None)

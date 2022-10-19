# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Fraunhofer FKIE/CMS, Alexander Tiderko
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
import subprocess
import sys
import re
from rosclean import get_disk_usage
import rospy
import rospkg
import time

from .settings import LOG_PATH, SETTINGS_PATH
from fkie_multimaster_msgs.system.supervised_popen import SupervisedPopen
from fkie_multimaster_msgs.logging.logging import Log


class ScreenException(Exception):
    pass


SCREEN = "/usr/bin/screen"
''':var SCREEN: Defines the path to screen binary.'''
SLASH_SEP = '_'
''':var SLASH_SEP: this character is used to replace the slashes in ROS-Names.'''


def create_session_name(node: str = '', namespace: str = '/') -> str:
    '''
    Creates a name for the screen session. All slash separators are replaced by `SLASH_SEP`
    and all `SLASH_SEP` are replaced by double `SLASH_SEP`.

    :param str node: the name of the node
    :return: name for the screen session.
    :rtype: str
    '''
    if node is None:
        return ''

    if namespace is None:
        namespace = '/'

    result = rospy.names.ns_join(
        namespace, node).replace(SLASH_SEP, '%s%s' % (SLASH_SEP, SLASH_SEP))
    result = result.replace('/', SLASH_SEP)
    return result


def session_name2node_name(session):
    '''
    Create a node name from screen session name. Revert changes done by :meth:`create_session_name`

    :param str session: the name of the session without pid
    :return: name name.
    :rtype: str
    '''
    node_name = session.replace('%s%s' % (SLASH_SEP, SLASH_SEP), '//')
    node_name = node_name.replace(SLASH_SEP, '/')
    node_name = node_name.replace('//', SLASH_SEP)
    return node_name


def split_session_name(session):
    '''
    Splits the screen session name into PID and session name generated by `create_session_name()`.

    :param str session: the screen session name
    :return: PID, session name generated by `create_session_name()`. Not presented
      values are coded as empty strings. Not valid session names have an empty
      PID string.
    :rtype: int, str
    '''
    if session is None:
        return '', ''
    result = session.split('.', 1)
    if len(result) != 2:
        return -1, ''
    pid = result[0].strip()
    try:
        pid = int(pid)
    except Exception:
        return -1, ''
    node = result[1].split('\t')
    if not node:
        return -1, ''
    return pid, node[0].strip()


def get_active_screens(nodename=''):
    '''
    Returns the dictionary (session name: node name) with all compatible screen names. If the session is set to
    an empty string all screens will be returned.

    :param str nodename: the name of the node.
    :return: On empty nodename returns all screen.
    :rtype: {str: [str]}
    '''
    result = {}
    starttime = time.time()
    ps = SupervisedPopen(
        [SCREEN, '-ls'], stdout=subprocess.PIPE, object_id='get_active_screens')
    output = ps.stdout.read() if sys.version_info[0] <= 2 else str(
        ps.stdout.read(), 'utf-8')
    if output:
        if time.time() - starttime > 1.0:
            Log.warn("'%s -ls' took too long (%.3f sec)! Fix your network configuration!" %
                     (SCREEN, time.time() - starttime))
        splits = output.splitlines()
        for item in splits:
            pid, nodepart = split_session_name(item)
            if pid != -1:
                screen_name = '%d.%s' % (pid, nodepart)
                if nodename:
                    # put all sessions which starts with '_'
                    if nodepart.startswith('_'):
                        if nodename == session_name2node_name(nodepart):
                            result[screen_name] = nodename
                else:
                    # only sessions for given node
                    name = session_name2node_name(nodepart)
                    result[screen_name] = name
    return result


def wipe():
    '''
    Calls 'screen -wipe' command to clean up SockDir.
    '''
    _ps = SupervisedPopen([SCREEN, '-wipe'], object_id='screen wipe')


def test_screen():
    '''
    Tests for whether the SCREEN binary exists and raise an exception if not.

    :raise ScreenHandlerException: if the screen binary not found.
    '''
    if not os.path.isfile(SCREEN):
        raise ScreenException(SCREEN, "%s is missing" % SCREEN)
    with open('%s/screen.cfg' % SETTINGS_PATH, 'w') as sf:
        sf.write('logfile flush 0')


def get_logfile(session: str = None, node: str = None, for_new_screen: bool = False, namespace: str = '/') -> str:
    '''
    Generates a log file name of the ROS log.

    :param str node: the name of the node
    :return: the ROS log file name
    :rtype: str
    :todo: get the run_id from the ROS parameter server and search in this log folder
           for the log file (handle the node started using a launch file).
    '''
    if session is not None:
        path = "%s%s.log" % (LOG_PATH, session)
        if os.path.exists(path):
            return path
    if node is not None:
        path = "%s%s.log" % (LOG_PATH, create_session_name(node, namespace))
        if os.path.exists(path) or for_new_screen:
            return path
    return get_ros_logfile(node)


def get_ros_logfile(node):
    '''
    Generates a log file name for the ROS log

    :param str node: the name of the node
    :return: the log file name
    :rtype: str
    '''
    logfile = ''
    if node is not None:
        logfile = "%s%s.log" % (LOG_PATH, node.strip('/').replace('/', '_'))
        if os.path.exists(logfile):
            return logfile
        else:
            # search in latest subfolder
            logpath = os.path.join(LOG_PATH, "latest")
            if not os.path.exists(logpath):
                logpath = LOG_PATH
            if not os.path.exists(logpath):
                return ''
            p = re.compile(r"%s-\d*.log" % (node.strip('/').replace('/', '-')))
            files = os.listdir(logpath)
            for fn in files:
                if p.match(fn):
                    return os.path.join(logpath, fn)
            p = re.compile(r"%s-\d*-stdout.log" %
                           (node.strip('/').replace('/', '-')))
            for fn in files:
                if p.match(fn):
                    return os.path.join(logpath, fn)
    return logfile


def get_pidfile(session: str = None, node: str = None, namespace: str = "/") -> str:
    '''
    Generates a PID file name for the screen session.

    :param str session: the name of the screen session
    :return: the PID file name
    :rtype: str
    '''
    if session is not None:
        return "%s%s.pid" % (LOG_PATH, session)
    elif node is not None:
        return "%s%s.pid" % (LOG_PATH, create_session_name(node, namespace))
    return "%s%s.pid" % (LOG_PATH, 'unknown')


def get_cmd(node: str, env=[], keys=[], namespace: str = '/') -> str:
    '''
    Return the command prefix to start the given node
    in a screen terminal.

    :param str node: the name of the node
    :return: the command prefix
    :rtype: str
    '''
    # see https://www.gnu.org/software/screen/manual/html_node/
    # If the command begins with a '-' character, the shell will be started as a login-shell.
    # Typical shells do only minimal initialization when not started as a login-shell.
    # E.g. Bash will not read your ~/.bashrc unless it is a login-shell.
    shell = '-/bin/bash'
    if 'SHELL' in os.environ:
        shell = '-%s' % os.environ['SHELL']
    cfg_file = '%s/screen.cfg' % SETTINGS_PATH
    cfg_opt = ''
    if os.path.exists(cfg_file):
        cfg_opt = '-c %s' % cfg_file
    return '%s %s -O -L -Logfile %s -s %s -dmS %s' % (SCREEN, cfg_opt, get_logfile(node=node, for_new_screen=True, namespace=namespace), shell, create_session_name(node=node, namespace=namespace))


def rosclean():
    '''
    Removes the content of the ROS-log directory. We didn't use rosclean purge because it
    removes the log-directory. This needs restart of ROS nodes or recreate log directory
    to get log again.
    '''
    d = rospkg.get_log_dir()
    if d and d != os.path.sep:
        ps = SupervisedPopen(
            ['rm -fr %s/*' % d], stdout=subprocess.PIPE, shell=True, object_id='rosclean')
        output_err = ps.stderr.read()
        if output_err:
            raise Exception(output_err)


def log_dir_size():
    '''
    :return: Disk usage in bytes for ROS log directory.
    :rtype: int
    '''
    d = rospkg.get_log_dir()
    disk_usage = get_disk_usage(d)
    return disk_usage


def delete_log(nodename):
    '''
    Removes log and runtime files located in ROS-log directory.
    These are log and configuration file of node's screen. PID file and ROS-log file of the node.

    :param str nodename: Name of the node.
    '''
    screen_log = get_logfile(node=nodename)
    pid_file = get_pidfile(node=nodename)
    roslog = get_ros_logfile(nodename)
    if os.path.isfile(screen_log):
        os.remove(screen_log)
    if os.path.isfile(pid_file):
        os.remove(pid_file)
    if os.path.isfile(roslog):
        os.remove(roslog)

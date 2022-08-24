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


import subprocess
import sys
import rospy
import time

from .supervised_popen import SupervisedPopen
from fkie_multimaster_msgs.logging.logging import Log


class ScreenException(Exception):
    pass


SCREEN = "/usr/bin/screen"
''':var SCREEN: Defines the path to screen binary.'''
SLASH_SEP = '_'
''':var SLASH_SEP: this character is used to replace the slashes in ROS-Names.'''


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

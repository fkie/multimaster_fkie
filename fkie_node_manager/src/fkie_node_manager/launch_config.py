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



import time
from python_qt_binding.QtCore import QObject
from fkie_node_manager_daemon.common import isstring

# from xml.dom import Node as DomNode #avoid aliasing
class LaunchConfigException(Exception):
    pass


class LaunchConfig(QObject):
    '''
    A class to handle the ROS configuration loaded on node manager daemon.
    '''

    def __init__(self, launch_file, mtime=0, includes={}, args={}, nodes=[]):
        '''
        Creates the LaunchConfig object. Store the informations get from
        node manager daemon.
        :param str launch_file: grpc path to the lauch file
        :param double mtime:  modification time of the launch file used to detect changes.
        :param includes: a dictionary with included files and their modification time stamps.
        :type includes: {str: double}
        :param args: a dictionary with arguments used to load launch file.
        :type args: {str: str}
        :param nodes: a list with node names.
        :type nodes: [str]
        '''
        QObject.__init__(self)
        self.launchfile = launch_file
        self.mtime = mtime
        self.includes = includes
        self.args = args
        self.nodes = nodes
        self.global_param_done = True
        self.__launch_id = '%.9f' % time.time()

    def __del__(self):
        # Delete to avoid segfault if the LaunchConfig class is destroyed recently
        # after creation and xmlrpclib.ServerProxy process a method call.
        pass

    def __eq__(self, item):
        '''
        Compares the path of the item.
        '''
        if isstring(item):
            return self.launchfile == item
        elif not (item is None):
            return self.launchfile == item.launchfile
        return False

    def __gt__(self, item):
        '''
        Compares the path of the item.
        '''
        if isstring(item):
            return self.launchfile > item
        elif not (item is None):
            return self.launchfile > item.launchfile
        return False

    def get_robot_icon(self):
        '''
        Returns the value of the `/robot_icon` parameter or None
        '''
        try:
            # TODO: get it from node manager daemon
            pass
            # return self.roscfg.params['/robot_icon'].value
        except Exception:
            pass
        return None

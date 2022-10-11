# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .host import get_hostname


STRING = 'STRING'
INT32 = 'INT32'
DOUBLE = 'DOUBLE'
BOOL = 'BOOL'
LIST = 'LIST'


class StartConfig():

    def __init__(self, package, binary):
        '''
        '''
        self.package = package
        self.binary = binary
        self.config_path = ''
        self.binary_path = ''
        self.name = ''
        self.namespace = ''
        self.fullname = ''
        self.prefix = ''
        self.cwd = ''
        self.env = {}
        self.remaps = {}
        self.params = {}
        self.param_files = []
        self.clear_params = []
        self.args = []
        self.daemonuri = None
        self.loglevel = ''
        self.logformat = ''
        self.respawn = False
        self.respawn_delay = 30
        self.respawn_max = 0
        self.respawn_min_runtime = 0
        self.cmd = []

    def __repr__(self):
        params = "name=%s" % self.name
        params += ", ns=%s" % self.namespace
        params += ", package=%s" % self.package
        params += ", binary=%s" % self.binary
        params += ", prefix=%s" % self.prefix
        params += ", cwd=%s" % self.cwd
        params += ", daemonuri=%s" % self.daemonuri
        params += ", loglevel=%s" % self.loglevel
        params += ", respawn=%s" % self.respawn
        params += ", cmd=%s" % self.cmd
        return "<StartConfig %s/>" % params

    @property
    def hostname(self):
        '''
        :return: host name from daemonuri if it is not None.
        '''
        if self.daemonuri:
            return get_hostname(self.daemonuri)
        return None

    @property
    def nmduri(self):
        '''
        :return: the nmd uri where to launch the node from daemonuri if it is not None.
        '''
        if self.daemonuri:
            self.daemonuri
        return None

    def _msg_type(self, value):
        valtype = type(value)
        if valtype == int:
            return INT32
        if valtype == float:
            return DOUBLE
        if valtype == bool:
            return BOOL
        if valtype == list:
            return LIST
        return STRING

    @classmethod
    def _from_msg_type(cls, value, value_type):
        if value_type == INT32:
            return int(value)
        if value_type == DOUBLE:
            return float(value)
        if value_type == BOOL:
            return value.lower() in ("yes", "true", "t", "1")
        if value_type == LIST:
            try:
                return eval(value)
            except Exception:
                return []
        return value

    def to_msg(self):
        msg = lmsg.StartConfig(package=self.package, binary=self.binary)
        self.fill_msg(msg)
        return msg

    def fill_msg(self, msg):
        msg.package = self.package
        msg.binary = self.binary
        if self.binary_path:
            msg.binary_path = self.binary_path
        if self.name:
            msg.name = self.name
        if self.namespace:
            msg.namespace = self.namespace
        if self.fullname:
            msg.fullname = self.fullname
        if self.prefix:
            msg.prefix = self.prefix
        if self.cwd:
            msg.cwd = self.cwd
        if self.env:
            msg.env.extend([lmsg.Argument(name=name, value=value)
                            for name, value in self.env.items()])
        if self.remaps:
            msg.remaps.extend([lmsg.Remapping(from_name=name, to_name=value)
                               for name, value in self.remaps.items()])
        if self.params:
            msg.params.extend([lmsg.Argument(name=name, value=value, value_type=self._msg_type(
                value)) for name, value in self.params.items()])
        if self.param_files:
            msg.param_files.extend(self.param_files)
        if self.clear_params:
            msg.clear_params.extend(self.clear_params)
        if self.args:
            msg.args.extend(self.args)
        if self.daemonuri:
            msg.daemonuri = self.daemonuri
        # if self.cmd:
        #    msg.cmd = ' '.join(self.cmd)
        msg.loglevel = self.loglevel
        msg.respawn = self.respawn
        msg.respawn_delay = self.respawn_delay
        msg.respawn_max = self.respawn_max
        msg.respawn_min_runtime = self.respawn_min_runtime

    @classmethod
    def from_msg(cls, msg):
        startcfg = StartConfig(msg.package, msg.binary)
        startcfg.binary_path = msg.binary_path
        startcfg.name = msg.name
        startcfg.namespace = msg.namespace
        startcfg.fullname = msg.fullname
        startcfg.prefix = msg.prefix
        startcfg.cwd = msg.cwd
        startcfg.env = {env.name: env.value for env in msg.env}
        startcfg.remaps = {
            remap.from_name: remap.to_name for remap in msg.remaps}
        startcfg.params = {param.name: cls._from_msg_type(
            param.value, param.value_type) for param in msg.params}
        startcfg.param_files = list(msg.param_files)
        startcfg.clear_params = list(msg.clear_params)
        startcfg.args = list(msg.args)
        startcfg.daemonuri = msg.daemonuri
        startcfg.loglevel = msg.loglevel
        startcfg.respawn = msg.respawn
        startcfg.respawn_delay = msg.respawn_delay
        startcfg.respawn_max = msg.respawn_max
        startcfg.respawn_min_runtime = msg.respawn_min_runtime
        # startcfg.cmd = shlex.split(msg.cmd)
        return startcfg

    @classmethod
    def from_ros_node(cls, node):
        pass

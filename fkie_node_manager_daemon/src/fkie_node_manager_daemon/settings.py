# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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
import rospy
import ruamel.yaml
import threading

from .common import utf8

GRPC_TIMEOUT = 15.0
''':var GRPC_TIMEOUT: timeout for connection to remote gRPC-server'''

RESPAWN_SCRIPT = 'rosrun fkie_node_manager respawn'
''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''

LOG_PATH = ''.join([os.environ.get('ROS_LOG_DIR'), os.path.sep]) if os.environ.get('ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
''':var LOG_PATH: logging path where all screen configuration and log files are stored.'''

SETTINGS_PATH = os.path.expanduser('~/.config/ros.fkie/')

class Settings:

    def __init__(self, filename='', version=''):
        self._mutex = threading.RLock()
        self.version = version
        self.filename = filename
        if not self.filename:
            self.filename = '%snode_manager_daemon.yaml' % SETTINGS_PATH
        cfg_path = os.path.dirname(self.filename)
        if not os.path.isdir(cfg_path):
            os.makedirs(cfg_path)
        self._reload_callbacks = []
        self._cfg = None
        self.reload()
        global GRPC_TIMEOUT
        GRPC_TIMEOUT = self.param('global/grpc_timeout', 15.0)

    def default(self):
        '''
        Creates a new default configuration.
        Value supports follow tags: {:value, :min, :max, :default, :hint(str), :ro(bool)}
        '''
        result = {
            'global': {
                'version': {':value': self.version, ':ro': True},
                'file': {':value': self.filename, ':ro': True},
                'grpc_timeout': {':value': 15.0, ':type': 'float', ':min': 0, ':default': 15.0, ':hint': "timeout for connection to remote gRPC-server"},
                'use_diagnostics_agg': {':value': False, ':hint': "subscribes to '/diagnostics_agg' topic instead of '/diagnostics'"},
                'reset': {':value': False, ':hint': 'if this flag is set to True the configuration will be reseted'},
                'grpc_verbosity': {':value': 'INFO', ':alt': ['DEBUG', 'INFO', 'ERROR'], ':hint': 'change gRPC verbosity', ':need_restart': True},
                'grpc_poll_strategy': {':value': '', ':alt': ['', 'poll', 'epollex', 'epoll1'], ':hint': 'change the strategy if you get warnings. Empty sets to default.', ':need_restart': True}
            },
            'sysmon':
            {
                'CPU':
                {
                    'load_warn_level': {':value': 0.9, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': 'warn if more than one CPU core exceeds this percentage'},
                },
                'Disk':
                {
                    'usage_warn_level': {':value': 0.95, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': "warn if used space exceeds this percentage"},
                    'path': {':value': LOG_PATH, ':hint': "Directory to observe", ':path': 'dir'}
                },
                'Memory':
                {
                    'usage_warn_level': {':value': 0.95, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': "warn if used memory exceeds this percentage"},
                },
                'Network':
                {
                    'load_warn_level': {':value': 0.9, ':type': 'float', ':min': 0, ':max': 1.0, ':default': 0.9, ':hint': "warn if load exceeds the percentage of the maximum speed"},
                    'speed': {':value': 6, ':default': 6, ':type': 'float', ':min': 0, ':hint': "Maximal speed in MBit"},
                    'interface': {':value': '', ':default': '', ':hint': "interface to observe", ':alt': []},
                }
            }
        }
        # :TODO: 'paths': {':type': 'path[]', ':value': {'path': {':type': 'string', ':value': ''}}}
        return result

    def param(self, param_name, default_value=None, extract_value=True):
        '''
        Returns parameter value for given param_name.

        :param str param_name: name of the parameter. Namespace is separated by '/'.
        :param default_value: returns this value if parameter was not found (Default: None)
        :param bool extract_value: Since value is a dictionary with additional informations,
            try to extract value by default on True or return all options by False (Default: True).
        '''
        result = default_value
        try:
            path = param_name.split('/')
            # print "  PATH", path
            value = self._cfg
            for item in path:
                # print "    item", item
                value = value[item]
                # print "      ", value
            if isinstance(value, dict):
                if extract_value and ':value' in value:
                    result = value[':value']
                else:
                    result = value
            else:
                result = value
        except Exception as exc:
            rospy.logdebug("Cant't get parameter '%s', full parameter path: '%s'" % (utf8(exc), param_name))
        return result

    def set_param(self, param_name, value, tag=':value'):
        '''
        Sets new value to a parameter. The parameter can contain namespaces separated by '/'.
        Since a value can contain different tags, you can change the tag value
        by specifying the tag parameter.

        :param: str param_name: parameter name with namespaces.
        :param: value: new value.
        :param: str tag: tag name of parameter. It should begin with ':'.
        '''
        try:
            path = os.path.dirname(param_name).split('/')
            val_tag = tag if tag else ':value'
            cfg_item = self._cfg
            changed = False
            for item in path:
                if item:
                    if item in cfg_item:
                        cfg_item = cfg_item[item]
                    else:
                        cfg_item[item] = {}
                        cfg_item = cfg_item[item]
                        changed = True
            pname = os.path.basename(param_name)
            if pname in cfg_item:
                if isinstance(cfg_item[pname], dict):
                    if self._is_writable(cfg_item[pname]):
                        changed = cfg_item[pname][val_tag] != value
                        cfg_item[pname][val_tag] = value
                    else:
                        raise Exception('%s is a read only parameter!' % param_name)
                else:
                    changed = cfg_item[pname] != value
                    cfg_item[pname] = value
            else:
                # create new parameter entry
                cfg_item[pname] = {val_tag: value}
                changed = True
            if changed:
                self.save()
        except Exception as exc:
            rospy.logdebug("Cant't set parameter '%s', full parameter path: '%s'" % (utf8(exc), param_name))

    def reload(self):
        '''
        Load the configuration from file. If file does not exists default configuration will be used.
        After configuration is loaded all subscribers are notified.
        '''
        with self._mutex:
            try:
                self._cfg = self.default()
                with open(self.filename, 'r') as stream:
                    result = ruamel.yaml.load(stream, Loader=ruamel.yaml.Loader)
                    if result is None:
                        rospy.loginfo('reset configuration file %s' % self.filename)
                        self.save()
                    else:
                        rospy.loginfo('loaded configuration from %s' % self.filename)
                        self._cfg = self._apply_recursive(result, self._cfg)
            except (ruamel.yaml.YAMLError, IOError) as exc:
                rospy.loginfo('%s: use default configuration!' % utf8(exc))
                self._cfg = self.default()
            self._notify_reload_callbacks()

    def save(self):
        '''
        Saves current configuration to file.
        '''
        with open(self.filename, 'w') as stream:
            try:
                ruamel.yaml.dump(self._cfg, stream, Dumper=ruamel.yaml.RoundTripDumper)
                rospy.logdebug("Configuration saved to '%s'" % self.filename)
            except ruamel.yaml.YAMLError as exc:
                rospy.logwarn("Cant't save configuration to '%s': %s" % (self.filename, utf8(exc)))

    def yaml(self, _nslist=[]):
        '''
        :param list nslist: Filter option. Currently not used!
        :return: Create YAML string representation from configuration dictionary structure.
        :rtype: str
        '''
        return ruamel.yaml.dump(self._cfg, Dumper=ruamel.yaml.RoundTripDumper)

    def apply(self, data):
        '''
        Applies data (string representation of YAML).
        After new data are set the configuration will be saved to file.
        All subscribers are notified.

        :param str data: YAML as string representation.
        '''
        with self._mutex:
            self._cfg = self._apply_recursive(ruamel.yaml.load(data, Loader=ruamel.yaml.Loader), self._cfg)
            do_reset = self.param('global/reset', False)
            if do_reset:
                rospy.loginfo("Reset configuration requested!")
                self._cfg = self.default()
            else:
                rospy.logdebug("new configuration applied, save now.")
            self.save()
            self._notify_reload_callbacks()

    def _apply_recursive(self, new_data, curr_value):
        new_cfg = dict()
        for key, value in curr_value.items():
            try:
                if isinstance(value, dict):
                    if self._is_writable(value):
                        new_cfg[key] = self._apply_recursive(new_data[key], value)
                elif key not in [':hint', ':default', ':ro', ':min', ':max', ':alt']:
                    if isinstance(new_data, dict):
                        new_cfg[key] = new_data[key]
                    else:
                        new_cfg[key] = new_data
                else:
                    new_cfg[key] = value
            except Exception:
                import traceback
                print("_apply_recursive error:", traceback.format_exc(), "use old value:", value)
                new_cfg[key] = value
        return new_cfg

    def _is_writable(self, value):
        if ':ro' in value:
            return value[':ro']
        return True

    def add_reload_listener(self, callback, call=True):
        '''
        Adds a subscriber to change notifications. All subscribers are notified on any changes.

        :param callback: Method of type callback(Settings)
        :param call: if True the callback is called after adding. (Default: True)
        '''
        with self._mutex:
            if callback not in self._reload_callbacks:
                self._reload_callbacks.append(callback)
                if call:
                    callback(self)

    def _notify_reload_callbacks(self):
        with self._mutex:
            for callback in self._reload_callbacks:
                callback(self)

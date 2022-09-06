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

import os
import ruamel.yaml
import threading
import fkie_node_manager_daemon as nmd
from .host import ros_host_suffix

DEFAULT_CROSSBAR_PORT = 12921

NM_NS = '_node_manager'
NM_NAMESPACE = '/%s' % NM_NS
NM_DAEMON_NAMESPACE = '/%s' % NM_NS
NM_DISCOVERY_NAMESPACE = '/%s' % NM_NS
NM_DAEMON_NAME = 'daemon_%s' % ros_host_suffix()
NM_DISCOVERY_NAME = f'discovery_{os.environ["ROS_DISTRO"]}_{ros_host_suffix()}'


GRPC_TIMEOUT = 15.0
''':var GRPC_TIMEOUT: timeout for connection to remote gRPC-server'''

RESPAWN_SCRIPT = 'ros2 run fkie_node_manager respawn'
''':var RESPAWN_SCRIPT: start prefix to launch ROS-Nodes with respawn script'''

LOG_PATH = ''.join([os.environ.get('ROS_LOG_DIR'), os.path.sep]) if os.environ.get(
    'ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
''':var LOG_PATH: logging path where all screen configuration and log files are stored.'''


class Settings:

    def __init__(self, filename='', version=''):
        self._mutex = threading.RLock()
        self.version = version
        self.filename = filename
        if not self.filename:
            self.filename = os.path.expanduser(
                '~/.config/ros.fkie/node_manager_daemon.yaml')
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
            nmd.ros_node.get_logger().debug("Cant't get parameter '%s', full parameter path: '%s'; use default value: %s" %
                                            (exc, param_name, default_value))
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
                        raise Exception(
                            '%s is a read only parameter!' % param_name)
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
            nmd.ros_node.get_logger().debug(
                "Cant't set parameter '%s', full parameter path: '%s'" % (exc, param_name))

    def reload(self):
        '''
        Load the configuration from file. If file does not exists default configuration will be used.
        After configuration is loaded all subscribers are notified.
        '''
        with self._mutex:
            try:
                with open(self.filename, 'r') as stream:
                    result = ruamel.yaml.load(
                        stream, Loader=ruamel.yaml.Loader)
                    if result is None:
                        nmd.ros_node.get_logger().info('reset configuration file %s' % self.filename)
                        self._cfg = self.default()
                        self.save()
                    else:
                        nmd.ros_node.get_logger().info('loaded configuration from %s' % self.filename)
                        self._cfg = result
            except (ruamel.yaml.YAMLError, IOError) as exc:
                nmd.ros_node.get_logger().info('%s: use default configuration!' % exc)
                self._cfg = self.default()
            self._notify_reload_callbacks()

    def save(self):
        '''
        Saves current configuration to file.
        '''
        with open(self.filename, 'w') as stream:
            try:
                ruamel.yaml.dump(self._cfg, stream,
                                 Dumper=ruamel.yaml.RoundTripDumper)
                nmd.ros_node.get_logger().debug("Configuration saved to '%s'" % self.filename)
            except ruamel.yaml.YAMLError as exc:
                nmd.ros_node.get_logger().warn(
                    "Cant't save configuration to '%s': %s" % (self.filename, exc))

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
            self._cfg = self._apply_recursive(ruamel.yaml.load(
                data, Loader=ruamel.yaml.Loader), self._cfg)
            do_reset = self.param('global/reset', False)
            if do_reset:
                nmd.ros_node.get_logger().info("Reset configuration requested!")
                self._cfg = self.default()
            else:
                nmd.ros_node.get_logger().debug("new configuration applied, save now.")
            self.save()
            self._notify_reload_callbacks()

    def _apply_recursive(self, new_data, curr_value):
        new_cfg = dict()
        for key, value in curr_value.items():
            try:
                if isinstance(value, dict):
                    if self._is_writable(value):
                        new_cfg[key] = self._apply_recursive(
                            new_data[key], value)
                elif key not in [':hint', ':default', ':ro', ':min', ':max', ':alt']:
                    if isinstance(new_data, dict):
                        new_cfg[key] = new_data[key]
                    else:
                        new_cfg[key] = new_data
                else:
                    new_cfg[key] = value
            except Exception:
                import traceback
                print("_apply_recursive error:",
                      traceback.format_exc(), "use old value:", value)
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

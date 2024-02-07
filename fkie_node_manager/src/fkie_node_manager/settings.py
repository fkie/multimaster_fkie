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



from python_qt_binding.QtGui import QColor  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtGui import QIcon  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtGui import QImage  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtGui import QPixmap  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtCore import QSettings  # pylint: disable=no-name-in-module, import-error

import os
import hashlib
import roslib
import rospy

from fkie_master_discovery.common import masteruri_from_ros
from fkie_node_manager_daemon.common import isstring, utf8
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon import settings as nmd_settings
from fkie_node_manager.detailed_msg_box import MessageBox
from fkie_node_manager.common import get_ros_home


class LoggingConfig(object):
    '''
    Holds the settings for changed log level while start a node.
    '''
    LOGLEVEL = 'DEFAULT'
    LOGLEVEL_ROSCPP = 'INFO'
    LOGLEVEL_SUPERDEBUG = 'WARN'
    CONSOLE_FORMAT = 'DEFAULT'

    def __init__(self):
        self.loglevel = self.LOGLEVEL
        self.loglevel_roscpp = self.LOGLEVEL_ROSCPP
        self.loglevel_superdebug = self.LOGLEVEL_SUPERDEBUG
        self.console_format = self.CONSOLE_FORMAT

    def get_attributes(self):
        '''
        Returns a list with all possible attributes, e.g.: loglevel, console_format

        :rtype: [str]
        '''
        return ['loglevel',
                'loglevel_roscpp',
                'loglevel_superdebug',
                'console_format'
                ]

    def is_default(self, attribute):
        '''
        Checks for an attribute :meth:`get_attributes` however it was changed.
        '''
        return getattr(self, attribute) == getattr(self, attribute.upper())

    def get_alternatives(self, attribute):
        '''
        Retruns a list with all possible values for an attribute :meth:`get_attributes`.

        :rtype: [str]
        '''
        result = []
        if attribute == 'console_format':
            result = [self.CONSOLE_FORMAT,
                      '[${severity}] [${time}]: ${message}',
                      '[${severity}] [${time}] [${logger}]: ${message}']
        elif attribute == 'loglevel':
            result = ['DEFAULT', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
        elif attribute == 'loglevel_roscpp':
            result = ['INFO', 'DEBUG', 'WARN', 'ERROR', 'FATAL']
        elif attribute == 'loglevel_superdebug':
            result = ['WARN', 'DEBUG', 'INFO', 'ERROR', 'FATAL']
        if not self.is_default(attribute):
            result.insert(0, getattr(self, attribute))
        return result


class Settings(object):

    PKG_NAME = 'fkie_node_manager'
    try:
        PACKAGE_DIR = roslib.packages.get_pkg_dir(PKG_NAME)
    except Exception:
        PACKAGE_DIR = "%s/../.." % os.path.realpath(os.path.dirname(__file__))
        if "dist-packages" in __file__:
            PACKAGE_DIR = "%s/../../share/fkie_node_manager" % PACKAGE_DIR
        print("PACKAGE_DIR: %s" % PACKAGE_DIR)
    CFG_PATH = os.path.expanduser('~/.config/ros.fkie/node_manager/')
    ''':ivar CFG_PATH: configuration path to store the settings and history'''
    HELP_FILE = os.path.join(PACKAGE_DIR, 'doc/index.rst')
    CURRENT_DIALOG_PATH = os.path.expanduser('~')
    LOG_PATH = screen.LOG_PATH
    LOG_VIEWER = "/usr/bin/less -fKLnQrSU"
    STARTER_SCRIPT = 'rosrun fkie_node_manager remote_nm.py'
    ''':ivar STARTER_SCRIPT: the script used on remote hosts to start new ROS nodes.'''

    LAUNCH_HISTORY_FILE = 'launch.history'
    PARAM_HISTORY_FILE = 'param.history'
    CFG_FILE = 'settings.ini'
    CFG_GUI_FILE = 'settings.ini'

    TIMEOUT_CONTROL = 5
    TIMEOUT_UPDATES = 20

    SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']
    LAUNCH_VIEW_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xacro']

    DEAFULT_HOST_COLORS = [QColor(255, 255, 235).rgb()]

    def __init__(self):
        self._data = {}
        self.reload()

    def reload(self):
        '''
        Loads the settings from file or sets default values if no one exists.
        '''
        self._terminal_emulator = None
        self._terminal_command_arg = 'e'
        self._noclose_str = ''
        self._terminal_title = '-T'
        self._masteruri = masteruri_from_ros()
        self.CFG_PATH = os.path.expanduser('~/.config/ros.fkie/node_manager/')
        # loads the current configuration path. If the path was changed, a redirection
        # file exists with new configuration folder
        if not os.path.isdir(self.CFG_PATH):
            os.makedirs(self.CFG_PATH)
            self.cfg_path = self.CFG_PATH
        # move all stuff from old location to new
        try:
            import shutil
            old_cfg_path = os.path.join(get_ros_home(), 'node_manager')
            if os.path.exists(old_cfg_path):
                print("move configuration to new destination: %s" % self.CFG_PATH)
                for filename in os.listdir(old_cfg_path):
                    shutil.move(os.path.join(old_cfg_path, filename), os.path.join(self.CFG_PATH, filename))
                shutil.rmtree(old_cfg_path)
        except Exception:
            pass
        print("Configuration path: %s" % self.CFG_PATH)
        # after the settings path was loaded, load other settings
        settings = self.qsettings(self.CFG_FILE)
        self._data = self._load_settings(settings)
        # load stored usernames for hosts
        settings.beginGroup('default_user_hosts')
        self._default_user_hosts = dict()
        for k in settings.childKeys():
            self._default_user_hosts[k] = settings.value(k, self.default_user)
        settings.endGroup()
        self._current_dialog_path = self.CURRENT_DIALOG_PATH
        self._log_viewer = self.LOG_VIEWER
        self._start_remote_script = self.STARTER_SCRIPT
        self.SEARCH_IN_EXT = list(set(self.SEARCH_IN_EXT) | set(self.str2list(self.launch_view_file_ext)))
        # setup logging
        self._rosconsole_cfg_file = 'rosconsole.config'
        self.logging = LoggingConfig()
        self.logging.loglevel = settings.value('logging/level', LoggingConfig.LOGLEVEL)
        self.logging.loglevel_roscpp = settings.value('logging/level_roscpp', LoggingConfig.LOGLEVEL_ROSCPP)
        self.logging.loglevel_superdebug = settings.value('logging/level_superdebug', LoggingConfig.LOGLEVEL_SUPERDEBUG)
        self.logging.console_format = settings.value('logging/rosconsole_format', LoggingConfig.CONSOLE_FORMAT)
        nmd_settings.GRPC_TIMEOUT = self.timeout_grpc
        # setup colors
        settings.beginGroup('host_colors')
        self._host_colors = dict()
        for k in settings.childKeys():
            self._host_colors[k] = settings.value(k, None)
        settings.endGroup()
        self.init_hosts_color_list()
        self._launch_history = None  # list with file names
        self._icons_dir = os.path.join(os.path.dirname(__file__), 'icons')

    def masteruri(self):
        return self._masteruri

    def _load_settings(self, settings):
        '''
        Creates a new default configuration.
        Value supports follow tags: {:value, :min, :max, :default, :hint(str), :ro(bool)}
        '''
        result = {'reset': {':value': False, ':var': 'reset', ':hint': 'if this flag is set to True the configuration will be reseted.'},
                  'reset_cache': {':value': False, ':var': 'reset_cache', ':hint': 'if this flag is set to True cached values will be removed.'},
                  'Default user:': {':value': settings.value('default_user', 'robot'),
                                    ':var': 'default_user',
                                    ':default': 'robot',
                                    ':hint': 'The user used for ssh connection to remote hosts if no one is set for specific master.'
                                    ' <span style="font-weight:600;">Restart required!</span>',
                                    ':need_restart': True
                                    },
                  'Launch history length:': {':value': int(settings.value('launch_history_length', 5)),
                                             ':var': 'launch_history_length',
                                             ':default': 5,
                                             ':min': 0,
                                             ':max': 25,
                                             ':hint': 'The count of recent loaded launch files displayed in the root'
                                             ' of the <span style="font-weight:600;">launch files</span> view.'
                                             },
                  'Param history length:': {':value': int(settings.value('param_history_length', 12)),
                                            ':var': 'param_history_length',
                                            ':default': 12,
                                            ':min': 0,
                                            ':max': 25,
                                            ':hint': 'The count of parameters stored which'
                                            ' are entered in a parameter dialog (Launch file arguments,'
                                            ' parameter server, publishing to a topic, service call)'
                                            },
                  'Settings path:': {':value': settings.value('cfg_path', self.CFG_PATH),
                                     ':var': 'cfg_path',
                                     ':path': 'dir',
                                     ':default': self.CFG_PATH,
                                     ':hint': '',
                                     ':ro': True,
                                     },
                  'Robot icon path:': {':value': os.path.join(self.PACKAGE_DIR, 'images'),
                                       ':var': 'robots_path',
                                       ':path': 'dir',
                                       ':default': os.path.join(self.PACKAGE_DIR, 'images'),
                                       ':hint': 'The path to the folder with robot images'
                                       '(<span style=" font-weight:600;">.png</span>).'
                                       ' The images with robot name will be displayed in the info bar.',
                                       },
                  'Show files extensions:': {':value': settings.value('launch_view_file_ext', ', '.join(self.LAUNCH_VIEW_EXT)),
                                             ':var': 'launch_view_file_ext',
                                             ':default': ', '.join(self.LAUNCH_VIEW_EXT),
                                             ':return_type': 'list',
                                             ':hint': 'Files that are displayed next to Launch'
                                             ' files in the <span style="font-weight:600;">launch files</span> view.',
                                             },
                  'Store window layout:': {':value': self.str2bool(settings.value('store_geometry', True)),
                                           ':var': 'store_geometry',
                                           ':default': True,
                                           ':hint': ''
                                           },
                  'Movable dock widgets:': {':value': self.str2bool(settings.value('movable_dock_widgets', True)),
                                            ':var': 'movable_dock_widgets',
                                            ':default': True,
                                            ':hint': 'On false you can\'t reorganize docking widgets.'
                                            ' <span style="font-weight:600;">Restart required!</span>',
                                            ':need_restart': True
                                            },
                  'Max time difference:': {':value': float(settings.value('max_timediff', 0.5)),
                                           ':var': 'max_timediff',
                                           ':default': 0.5,
                                           ':step': 0.1,
                                           ':hint': 'Shows a warning if the time difference to remote host is greater than this value.',
                                           },
                  'Autoupdate:': {':value': self.str2bool(settings.value('autoupdate', True)),
                                  ':var': 'autoupdate',
                                  ':default': True,
                                  ':hint': 'By default node manager updates the current'
                                  ' state on changes. You can deactivate this behavior to'
                                  ' reduce the network load. If autoupdate is deactivated'
                                  ' you must refresh the state manually.',
                                  },
                  'Start sync with discovery:': {':value': self.str2bool(settings.value('start_sync_with_discovery', False)),
                                                 ':var': 'start_sync_with_discovery',
                                                 ':default': False,
                                                 ':hint': "Sets 'start sync' in 'Start' master discovery"
                                                 "dialog to True, if this option is set to true."
                                                 },
                  'Start daemon with discovery:': {':value': self.str2bool(settings.value('start_daemon_with_discovery', False)),
                                                 ':var': 'start_daemon_with_discovery',
                                                 ':default': False,
                                                 ':hint': "Sets 'start daemons' in 'Start' master discovery"
                                                 "dialog to True, if this option is set to true."
                                                 },
                  'Confirm exit when closing:': {':value': self.str2bool(settings.value('confirm_exit_when_closing', True)),
                                                 ':var': 'confirm_exit_when_closing',
                                                 ':default': True,
                                                 ':hint': "Shows on closing of node_manager a dialog to stop"
                                                 " all ROS nodes if this option is set to true.",
                                                 },
                  'Highlight xml blocks:': {':value': self.str2bool(settings.value('highlight_xml_blocks', True)),
                                            ':var': 'highlight_xml_blocks',
                                            ':default': True,
                                            ':hint': "Highlights the current selected XML block, while editing ROS launch file.",
                                            },
                  'Colorize hosts:': {':value': self.str2bool(settings.value('colorize_hosts', True)),
                                      ':var': 'colorize_hosts',
                                      ':default': True,
                                      ':hint': "Determine automatic a default color for each host if True."
                                      " Manually setting color will be preferred. You can select the color by"
                                      " double-click on hostname in description panel. To remove a setting color"
                                      " delete it manually from %s" % self.CFG_PATH,
                                      },
                  'Check for nodelets at start:': {':value': self.str2bool(settings.value('check_for_nodelets_at_start', True)),
                                                   ':var': 'check_for_nodelets_at_start',
                                                   ':default': True,
                                                   ':hint': "Test the starting nodes for nodelet manager and all nodelets."
                                                   " If one of the nodes is not in the list a dialog is displayed with"
                                                   " proposal to start other nodes, too.",
                                                   },
                  'Show noscreen error:': {':value': self.str2bool(settings.value('show_noscreen_error', True)),
                                           ':var': 'show_noscreen_error',
                                           ':default': True,
                                           ':hint': "Shows an error if requested screen for a node is not available.",
                                           },
                  'Autoreload changed launch files:': {':value': self.str2bool(settings.value('autoreload_launch', False)),
                                             ':var': 'autoreload_launch',
                                             ':default': False,
                                             ':hint': "On change asks for reload launch file. On True reload without asking.",
                                             },
                  'Show domain suffix:': {':value': self.str2bool(settings.value('show_domain_suffix', False)),
                                          ':var': 'show_domain_suffix',
                                          ':default': False,
                                          ':hint': "Shows the domain suffix of the host in the host description panel and node tree view.",
                                          },
                  'Transpose pub/sub description:': {':value': self.str2bool(settings.value('transpose_pub_sub_descr', True)),
                                                     ':var': 'transpose_pub_sub_descr',
                                                     ':default': True,
                                                     ':hint': "Transpose publisher/subscriber in description dock.",
                                                     },
                  'Timeout close dialog:': {':value': float(settings.value('timeout_close_dialog', 30.0)),
                                            ':var': 'timeout_close_dialog',
                                            ':default': 30.0,
                                            ':step': 1.,
                                            ':hint': "Timeout in seconds to close dialog while closing Node Manager."
                                            " 0 disables autoclose functionality.",
                                            },
                  'Group nodes by namespace:': {':value': self.str2bool(settings.value('group_nodes_by_namespace', True)),
                                                ':var': 'group_nodes_by_namespace',
                                                ':default': True,
                                                ':hint': 'Split namespace of the node by / and create groups for each name part.'
                                                ' <span style="font-weight:600;">Restart required!</span>',
                                                ':need_restart': True,
                                                },
                  'Timeout for GRPC requests:': {':value': float(settings.value('timeout_grpc', nmd_settings.GRPC_TIMEOUT)),
                                                 ':var': 'timeout_grpc',
                                                 ':default': nmd_settings.GRPC_TIMEOUT,
                                                 ':step': 1.,
                                                 ':hint': "Timeout in seconds for GRPC requests to daemon.",
                                                 },
                  'Sysmon default interval:': {':value': int(settings.value('sysmon_default_interval', 10)),
                                               ':var': 'sysmon_default_interval',
                                               ':default': 10,
                                               ':step': 1,
                                               ':hint': 'Interval in seconds to get system monitor diagnostics from each remote host.'
                                               ' <span style="font-weight:600;">Restart required!</span>',
                                               ':need_restart': True,
                                               },
                  'Use /diagnostigs_agg:': {':value': self.str2bool(settings.value('use_diagnostics_agg', False)),
                                            ':var': 'use_diagnostics_agg',
                                            ':default': False,
                                            ':hint': 'subscribes to \'/diagnostics_agg\' topic instead of \'/diagnostics\'.'
                                            ' <span style="font-weight:600;">Restart required!</span>',
                                            ':need_restart': True,
                                            },
                  'Use internal log widget:': {':value': self.str2bool(settings.value('use_internal_log_widget', True)),
                                                ':var': 'use_internal_log_widget',
                                                ':default': False,
                                                ':hint': 'Opens the log file in internal dock instead of new terminal. If deactivated still accessible with Ctrl modifier.',
                                                ':need_restart': False,
                                                },
                  }
        return result

    def yaml(self):
        return self._data

    def set_yaml(self, data):
        reset = False
        reset_cache = False
        need_restart = False
        for value in data.values():
            setattr(self, value[':var'], value[':value'])
            if ':need_restart' in value:
                if value[':need_restart']:
                    need_restart = True
            if value[':var'] == 'reset':
                reset = value[':value']
            if value[':var'] == 'reset_cache':
                reset_cache = value[':value']
        if need_restart:
            MessageBox.information(None, "restart Node Manager", "Some of modified parameter requires a restart of Node Manager!")
        # handle reset
        if reset:
            try:
                os.remove(os.path.join(self.CFG_PATH, self.CFG_FILE))
                os.remove(os.path.join(self.CFG_PATH, self.CFG_GUI_FILE))
            except Exception:
                pass
            self.reload()
        if reset_cache:
            try:
                os.remove(os.path.join(self.CFG_PATH, self.LAUNCH_HISTORY_FILE))
                os.remove(os.path.join(self.CFG_PATH, self.PARAM_HISTORY_FILE))
            except Exception:
                pass

    def __getattr__(self, name):
        for value in self._data.values():
            if value[':var'] == name:
                if ':return_type' in value:
                    if value[':return_type'] == 'list':
                        return self.str2list(value[':value'])
                return value[':value']
        raise AttributeError("'Settings' has no attribute '%s'" % name)

    def __setattr__(self, name, value):
        if name == '_data':
            object.__setattr__(self, name, value)
        for val in self._data.values():
            if val[':var'] == name:
                setval = value
                val[':value'] = setval
                # if it is a directory, create it if not exists
                if ':path' in val:
                    if val[':path'] == 'dir':
                        setval = os.path.abspath(setval).rstrip(os.path.sep)
                        val[':value'] = setval
                    if name == 'cfg_path':
                        if not os.path.isdir(setval):
                            os.makedirs(setval)
                settings = self.qsettings(self.CFG_FILE)
                settings.setValue(name, setval)
                return
        object.__setattr__(self, name, value)

    def icon_path(self, name):
        return os.path.join(self._icons_dir, name)

    def icon(self, name):
        return QIcon(self.icon_path(name))

    def image(self, name):
        return QImage(self.icon_path(name))

    def pixmap(self, name):
        return QPixmap(self.icon_path(name))

    def host_user(self, host):
        if host in self._default_user_hosts:
            return self._default_user_hosts[host]
        return self.default_user

    def set_host_user(self, host, user):
        if host and user:
            self._default_user_hosts[host] = user
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('default_user_hosts/%s' % host, user)

    @property
    def current_dialog_path(self):
        return self._current_dialog_path

    @current_dialog_path.setter
    def current_dialog_path(self, path):
        self._current_dialog_path = path

    def robot_image_file(self, robot_name):
        return os.path.join(self.robots_path, '%s.png' % robot_name)

    @property
    def log_viewer(self):
        return self._log_viewer

    @log_viewer.setter
    def log_viewer(self, viewer):
        self._log_viewer = viewer

    @property
    def start_remote_script(self):
        return self._start_remote_script

    @start_remote_script.setter
    def start_remote_script(self, script):
        self._start_remote_script = script

    def rosconsole_cfg_file(self, package):
        result = os.path.join(self.LOG_PATH, '%s.%s' % (package, self._rosconsole_cfg_file))
        with open(result, 'w') as cfg_file:
            cfg_file.write('log4j.logger.ros=%s\n' % self.logging.loglevel)
            cfg_file.write('log4j.logger.ros.roscpp=%s\n' % self.logging.loglevel_roscpp)
            cfg_file.write('log4j.logger.ros.roscpp.superdebug=%s\n' % self.logging.loglevel_superdebug)
        return result

    def store_logging(self):
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('logging/level', self.logging.loglevel)
        settings.setValue('logging/level_roscpp', self.logging.loglevel_roscpp)
        settings.setValue('logging/level_superdebug', self.logging.loglevel_superdebug)
        settings.setValue('logging/rosconsole_format', self.logging.console_format)

    def host_color(self, host, default_color):
        if self.colorize_hosts:
            # get the color from settings file
            if host in self._host_colors:
                result = self._host_colors[host]
                if isstring(result):
                    return int(result)
                return result
            else:
                # determine a color
                hash_str = hashlib.md5(host.encode('utf-8')).hexdigest()
                hash_int = int(hash_str, 16)
                index = abs(hash_int) % len(self.DEAFULT_HOST_COLORS)
                return self.DEAFULT_HOST_COLORS[index]
        return default_color

    def set_host_color(self, host, color):
        if host and color:
            self._host_colors[host] = color
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('host_colors/%s' % host, color)

    @property
    def launch_history(self):
        '''
        Read the history of the recently loaded files from the file stored in ROS_HOME path.

        :return: the list with file names
        :rtype: list(str)
        '''
        if self._launch_history is not None:
            return self._launch_history
        # load history from file, only first time
        result = list()
        history_file = self.qsettings(self.LAUNCH_HISTORY_FILE)
        size = history_file.beginReadArray("launch_history")
        for i in range(size):
            history_file.setArrayIndex(i)
            if i >= self.launch_history_length:
                break
            launch_file = history_file.value("file")
            # TODO: if os.path.isfile(launch_file):
            result.append(launch_file)
        history_file.endArray()
        self._launch_history = result
        return self._launch_history

    def launch_history_add(self, path, replace=None):
        '''
        Adds a path to the list of recently loaded files.

        :param str path: the path with the file name
        :param str replace: the path to replace, e.g. rename
        '''
        to_remove = replace
        if replace is None:
            to_remove = path
        if self._launch_history is None:
            self.launch_history
        try:
            self._launch_history.remove(to_remove)
        except Exception:
            pass
        self._launch_history.append(path)
        while len(self._launch_history) > self.launch_history_length:
            self._launch_history.pop(0)
        self._launch_history_save(self._launch_history)

    def launch_history_remove(self, path):
        '''
        Removes a path from the list of recently loaded files.

        :param list(str) path: the path with the file name
        '''
        try:
            self._launch_history.remove(path)
            self._launch_history_save(self._launch_history)
        except Exception:
            pass

    def _launch_history_save(self, paths):
        '''
        Saves the list of recently loaded files to history. The existing history will be replaced!

        :param list(str) paths: the list with filenames
        '''
        history_file = self.qsettings(self.LAUNCH_HISTORY_FILE)
        history_file.beginWriteArray("launch_history")
        for i, launch_file in enumerate(paths):
            history_file.setArrayIndex(i)
            history_file.setValue("file", launch_file)
        history_file.endArray()
        self._launch_history = list(paths)

    def str2bool(self, v):
        if isinstance(v, bool):
            return v
        return v.lower() in ("yes", "true", "t", "1")

    def str2list(self, lstr):
        if isinstance(lstr, list):
            return lstr
        try:
            lstr = lstr.strip('[]')
            lstr = lstr.replace('u"', '')
            lstr = lstr.replace('"', '')
            lstr = lstr.replace("'", '')
            lstr = lstr.replace(",", ' ')
            return [utf8(i).strip() for i in lstr.split(' ') if i]
        except Exception:
            return []

    def terminal_cmd(self, cmd, title, noclose=False):
        '''
        Creates a command string to run with a terminal prefix

        :param list(str) cmd: the list with a command and args
        :param str title: the title of the terminal
        :return: command with a terminal prefix
        :rtype: str
        '''
        terminal_emulator = ''
        terminal_title = self._terminal_title
        noclose_str = self._noclose_str
        terminal_command_arg = self._terminal_command_arg
        for t in ['/usr/bin/x-terminal-emulator', '/usr/bin/xterm', '/opt/x11/bin/xterm']:
            if os.path.isfile(t) and os.access(t, os.X_OK):
                print(os.path.basename(os.path.realpath(t)))
                # workaround to support the command parameter in different terminal
                if os.path.basename(os.path.realpath(t)) in ['terminator', 'gnome-terminal', 'xfce4-terminal']:
                    terminal_command_arg = 'x'
                else:
                    terminal_command_arg = 'e'
                if os.path.basename(os.path.realpath(t)) in ['terminator', 'gnome-terminal', 'gnome-terminal.wrapper']:
                    noclose_str = '--profile hold'
                    if noclose:
                        rospy.loginfo("If your terminal close after the execution, you can change this behavior in "
                                        "profiles. You can also create a profile with name 'hold'. This profile will "
                                        "be then load by node_manager.")
                elif os.path.basename(os.path.realpath(t)) in ['xfce4-terminal', 'xterm', 'lxterm', 'uxterm']:
                    noclose_str = ''
                    terminal_title = '-T'
                terminal_emulator = t
                break
        if terminal_emulator == '':
            raise Exception("No Terminal found! Please install one of ['/usr/bin/x-terminal-emulator', '/usr/bin/xterm', '/opt/x11/bin/xterm']")
        noclose_str = noclose_str if noclose else ''
        title_opt = ''
        if title:
            title_opt = '%s "%s"' % (terminal_title, title)
        return '%s %s %s -%s %s' % (terminal_emulator, title_opt, noclose_str, terminal_command_arg, ' '.join(cmd))

    def qsettings(self, settings_file):
        path = settings_file
        if not settings_file.startswith(os.path.sep):
            path = os.path.join(self.CFG_PATH, settings_file)
        return QSettings(path, QSettings.IniFormat)

    def init_hosts_color_list(self):
        self.DEAFULT_HOST_COLORS = [
            QColor(255, 255, 235).rgb(),
            QColor(87, 93, 94).rgb(),
            QColor(205, 186, 136).rgb(),
            QColor(249, 168, 0).rgb(),
            QColor(232, 140, 0).rgb(),
            QColor(175, 128, 79).rgb(),
            QColor(221, 175, 39).rgb(),
            QColor(227, 217, 198).rgb(),
            QColor(186, 72, 27).rgb(),
            QColor(246, 120, 40).rgb(),
            QColor(255, 77, 6).rgb(),
            QColor(89, 25, 31).rgb(),
            QColor(216, 160, 166).rgb(),
            QColor(129, 97, 131).rgb(),
            QColor(196, 97, 140).rgb(),
            QColor(118, 104, 154).rgb(),
            QColor(188, 64, 119).rgb(),
            QColor(0, 56, 123).rgb(),
            QColor(15, 76, 100).rgb(),
            QColor(0, 137, 182).rgb(),
            QColor(99, 125, 150).rgb(),
            QColor(5, 139, 140).rgb(),
            QColor(34, 45, 90).rgb(),
            QColor(60, 116, 96).rgb(),
            QColor(54, 103, 53).rgb(),
            QColor(80, 83, 60).rgb(),
            QColor(17, 66, 50).rgb(),
            QColor(108, 124, 89).rgb(),
            QColor(97, 153, 59).rgb(),
            QColor(185, 206, 172).rgb(),
            QColor(0, 131, 81).rgb(),
            QColor(126, 186, 181).rgb(),
            QColor(0, 181, 26).rgb(),
            QColor(122, 136, 142).rgb(),
            QColor(108, 110, 107).rgb(),
            QColor(118, 106, 94).rgb(),
            QColor(56, 62, 66).rgb(),
            QColor(128, 128, 118).rgb(),
            QColor(127, 130, 116).rgb(),
            QColor(197, 199, 196).rgb(),
            QColor(137, 105, 62).rgb(),
            QColor(112, 69, 42).rgb(),
            QColor(141, 73, 49).rgb(),
            QColor(90, 56, 38).rgb(),
            QColor(233, 224, 210).rgb(),
            QColor(236, 236, 231).rgb(),
            QColor(43, 43, 44).rgb(),
            QColor(121, 123, 122).rgb()
        ]

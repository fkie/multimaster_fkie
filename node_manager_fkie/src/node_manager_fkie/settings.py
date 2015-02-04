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
import roslib

from common import get_ros_home, masteruri_from_ros

class Settings(object):

  USER_DEFAULT = 'robot'
  # set the cwd to the package of the node_manager_fkie to support the images
  # in HTML descriptions of the robots and capabilities
  PKG_NAME = 'node_manager_fkie'
  PACKAGE_DIR = roslib.packages.get_pkg_dir(PKG_NAME)
  ROBOTS_DIR = os.path.join(PACKAGE_DIR, 'images')
  CFG_PATH = os.path.join('.node_manager', os.sep)
  '''@ivar: configuration path to store the history.'''
  HELP_FILE = os.path.join(PACKAGE_DIR, 'README.rst')
  CURRENT_DIALOG_PATH = os.path.expanduser('~')
  LOG_PATH = os.environ.get('ROS_LOG_DIR') if os.environ.get('ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')

  LOG_VIEWER = "/usr/bin/less -fKLnQrSU"
  STARTER_SCRIPT = 'rosrun node_manager_fkie remote_nm.py'
  RESPAWN_SCRIPT = 'rosrun node_manager_fkie respawn'
  '''
  the script used on remote hosts to start new ROS nodes
  '''

  LAUNCH_HISTORY_FILE = 'launch.history'
  LAUNCH_HISTORY_LENGTH = 5

  PARAM_HISTORY_FILE = 'param.history'
  PARAM_HISTORY_LENGTH = 12

  CFG_REDIRECT_FILE = 'redirect'
  CFG_FILE = 'settings.ini'
  CFG_GUI_FILE = 'settings.ini'

  TIMEOUT_CONTROL = 5
  TIMEOUT_UPDATES = 20

  SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.sync', '.test', '.xml']
  LAUNCH_VIEW_EXT = ['.yaml', '.conf', '.cfg', '.iface', '.sync', '.test']

  STORE_GEOMETRY = True
  AUTOUPDATE = True

  def __init__(self):
    self.reload()

  def reload(self):
    '''
    Loads the settings from file or sets default values if no one exists.
    '''
    self._terminal_emulator = None
    self._terminal_command_arg = 'e'
    self._masteruri = masteruri_from_ros()
    self.CFG_PATH = os.path.join(get_ros_home(), 'node_manager')
    # loads the current configuration path. If the path was changed, a redirection
    # file exists with new configuration folder
    if not os.path.isdir(self.CFG_PATH):
      os.makedirs(self.CFG_PATH)
      self._cfg_path = self.CFG_PATH
    else:
      settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
      self._cfg_path = settings.value('cfg_path', self.CFG_PATH)
    # after the settings path was loaded, load other settings
    self._robots_path = self.ROBOTS_DIR
    settings = self.qsettings(self.CFG_FILE)
    self._default_user = settings.value('default_user', self.USER_DEFAULT)
    settings.beginGroup('default_user_hosts')
    self._default_user_hosts = dict()
    for k in settings.childKeys():
      self._default_user_hosts[k] = settings.value(k, self._default_user)
    settings.endGroup()
    try:
      self._launch_history_length = int(settings.value('launch_history_length', self.LAUNCH_HISTORY_LENGTH))
    except:
      self._launch_history_length = self.LAUNCH_HISTORY_LENGTH
    try:
      self._param_history_length = int(settings.value('param_history_length', self.PARAM_HISTORY_LENGTH))
    except:
      self._param_history_length = self.PARAM_HISTORY_LENGTH
    self._current_dialog_path = self.CURRENT_DIALOG_PATH
    self._log_viewer = self.LOG_VIEWER
    self._start_remote_script = self.STARTER_SCRIPT
    self._respawn_script = self.RESPAWN_SCRIPT
    self._launch_view_file_ext = self.str2list(settings.value('launch_view_file_ext', ', '.join(self.LAUNCH_VIEW_EXT)))
    self._store_geometry = self.str2bool(settings.value('store_geometry', self.STORE_GEOMETRY))
    self.SEARCH_IN_EXT = list(set(self.SEARCH_IN_EXT) | set(self._launch_view_file_ext))
    self._autoupdate = self.str2bool(settings.value('autoupdate', self.AUTOUPDATE))

  def masteruri(self):
    return self._masteruri

  @property
  def cfg_path(self):
    return self._cfg_path

  @cfg_path.setter
  def cfg_path(self, path):
    if path:
      abspath = os.path.abspath(path).rstrip(os.path.sep)
      if not os.path.isdir(abspath):
        os.makedirs(abspath)
      self._cfg_path = abspath
      if abspath != os.path.abspath(self.CFG_PATH).rstrip(os.path.sep):
        settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
        settings.setValue('cfg_path', abspath)
      else:
        # remove the redirection
        settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
        settings.remove('cfg_path')
      self.reload()

  @property
  def robots_path(self):
    return self._robots_path

  @robots_path.setter
  def robots_path(self, path):
    if path:
      if not os.path.isdir(path):
        os.makedirs(path)
      self._robots_path = path
      settings = self.qsettings(self.CFG_FILE)
      settings.setValue('robots_path', self._robots_path)

  @property
  def default_user(self):
    return self._default_user

  @default_user.setter
  def default_user(self, user):
    if user:
      self._default_user = user
      settings = self.qsettings(self.CFG_FILE)
      settings.setValue('default_user', self._default_user)

  def host_user(self, host):
    if host in self._default_user_hosts:
      return self._default_user_hosts[host]
    return self.default_user

  def set_host_user(self, host, user):
    if host and user:
      self._default_user_hosts[host] = user
      settings = self.qsettings(self.CFG_FILE)
      settings.setValue('default_user_hosts/%s'%host, user)

  @property
  def launch_history_length(self):
    return self._launch_history_length

  @launch_history_length.setter
  def launch_history_length(self, length):
    self._launch_history_length = length
    settings = self.qsettings(self.CFG_FILE)
    settings.setValue('launch_history_length', self._launch_history_length)

  @property
  def param_history_length(self):
    return self._param_history_length

  @param_history_length.setter
  def param_history_length(self, length):
    self._param_history_length = length
    settings = self.qsettings(self.CFG_FILE)
    settings.setValue('param_history_length', self._param_history_length)

  @property
  def current_dialog_path(self):
    return self._current_dialog_path

  @current_dialog_path.setter
  def current_dialog_path(self, path):
    self._current_dialog_path = path

  def robot_image_file(self, robot_name):
    return os.path.join(self.ROBOTS_DIR, '%s.png'%robot_name)

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

  @property
  def respawn_script(self):
    return self._respawn_script

  @respawn_script.setter
  def respawn_script(self, script):
    self._respawn_script = script

  @property
  def launch_view_file_ext(self):
    return self._launch_view_file_ext

  @launch_view_file_ext.setter
  def launch_view_file_ext(self, exts):
    self._launch_view_file_ext = self.str2list('%s'%exts)
    settings = self.qsettings(self.CFG_FILE)
    settings.setValue('launch_view_file_ext', self._launch_view_file_ext)
    self.SEARCH_IN_EXT = list(set(self.SEARCH_IN_EXT) | set(self._launch_view_file_ext))

  @property
  def store_geometry(self):
    return self._store_geometry

  @store_geometry.setter
  def store_geometry(self, value):
    v = self.str2bool(value)
    if self._store_geometry != v:
      self._store_geometry = v
      settings = self.qsettings(self.CFG_FILE)
      settings.setValue('store_geometry', self._store_geometry)

  @property
  def autoupdate(self):
    return self._autoupdate

  @autoupdate.setter
  def autoupdate(self, value):
    v = self.str2bool(value)
    if self._autoupdate != v:
      self._autoupdate = v
      settings = self.qsettings(self.CFG_FILE)
      settings.setValue('autoupdate', self._autoupdate)

  def str2bool(self, v):
    if isinstance(v, bool):
      return v
    return v.lower() in ("yes", "true", "t", "1")

  def str2list(self, l):
    if isinstance(l, list):
      return l
    try:
      l = l.strip('[]')
      l = l.replace('u"', '')
      l = l.replace('"', '')
      l = l.replace("'", '')
      l = l.replace(",", ' ')
      return [str(i).strip() for i in l.split(' ') if i]
    except:
      return []

  def terminal_cmd(self, cmd, title):
    '''
    Creates a command string to run with a terminal prefix
    @param cmd: the list with a command and args
    @type cmd: [str,..]
    @param title: the title of the terminal
    @type title: str
    @return: command with a terminal prefix
    @rtype:  str
    '''
    if self._terminal_emulator is None:
      self._terminal_emulator = ""
      for t in ['/usr/bin/x-terminal-emulator', '/usr/bin/xterm']:
        if os.path.isfile(t) and os.access(t, os.X_OK):
          #workaround to support the command parameter in different terminal
          if os.path.basename(os.path.realpath(t)) in ['terminator', 'gnome-terminal', 'xfce4-terminal']:
            self._terminal_command_arg = 'x'
          self._terminal_emulator = t
          break
    if self._terminal_emulator == "": return ""
    return '%s -T "%s" -%s %s'%(self._terminal_emulator, title, self._terminal_command_arg, ' '.join(cmd))

  def qsettings(self, settings_file):
    from python_qt_binding import QtCore
    path = settings_file
    if not settings_file.startswith(os.path.sep):
      path = os.path.join(self.cfg_path, settings_file)
    return QtCore.QSettings(path, QtCore.QSettings.IniFormat)

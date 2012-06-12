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
#  * Neither the name of I Heart Engineering nor the names of its
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
import time
from ros import roslaunch
import rospy
import roslib

from PySide import QtCore

import node_manager_fkie as nm

class LaunchConfigException(Exception):
  pass


class LaunchConfig(QtCore.QObject):
  '''
  A class to handle the ROS configuration stored in launch file.
  '''
  file_changed = QtCore.Signal(str, str)
  '''@ivar: a signal to inform the receiver about the changes on
  launch file or included file. ParameterB{:} (changed file, launch file)'''
  
  CFG_PATH = ''.join([os.environ['HOME'], '/', '.ros/node_manager/'])
  '''@ivar: configuration path to store the argument history.'''
  
  def __init__(self, launch_file, package=None, masteruri=None, argv=[]):
    '''
    Creates the LaunchConfig object. The launch file will be not loaded on 
    creation, first on request of Roscfg value.
    @param launch_file: The absolute or relative path with the launch file.
                       By using relative path a package must be valid for
                       remote launches.
    @type launch_file: C{str}
    @param package:  the package containing the launch file. If None the 
                    launch_file will be used to determine the launch file.
                    No remote launches a possible without a valid package.
    @type package: C{str} or C{None}
    @param masteruri: The URL of the ROS master.
    @type masteruri: C{str} or C{None}
    @param argv: the list the arguments needed for loading the given launch file
    @type argv: C{[str]}
    @raise roslaunch.XmlParseException: if the launch file can't be found.
    '''
    QtCore.QObject.__init__(self)
    self.__launchFile = launch_file
    self.__package = self.__getPackageName(os.path.dirname(self.__launchFile)) if package is None else package 
    self.__masteruri = masteruri if not masteruri is None else 'localhost'
    self.__roscfg = None
    self.argv = argv
    self.__reqTested = False
    self.global_param_done = [] # masteruri's where the global parameters are registered 
    self.hostname = nm.nameres().getHostname(self.__masteruri)
    self.file_watcher = QtCore.QFileSystemWatcher()
    self.file_watcher.fileChanged.connect(self.on_file_changed)
    self.changed = {}

  def on_file_changed(self, file):
    '''
    callback method, which is called by L{QtCore.QFileSystemWatcher} if the 
    launch file or included files are changed. In this case 
    L{LaunchConfig.file_changed} signal will be emitted.
    '''
    # to avoid to handle from QFileSystemWatcher fired the signal two times 
    if (not self.changed.has_key(file) or (self.changed.has_key(file) and self.changed[file] + 0.05 < time.time())):
      self.changed[file] = time.time()
      self.file_changed.emit(file, self.__launchFile)

  @property
  def masteruri(self):
    '''
    Returns the master URI (host) where the node of this config will be started.
    @rtype: C{str}
    '''
    return self.__masteruri

  @property
  def Roscfg(self):
    '''
    Returns a loaded launch configuration
    @rtype: L{roslaunch.ROSLaunchConfig} or C{None}
    @raise LaunchConfigException: on load error
    @see L{load()}
    '''
    if not (self.__roscfg is None):
      return self.__roscfg
    else:
      result, argv = self.load(self.argv)
      if not result:
        raise LaunchConfigException("not all argv are setted properly!")
      return self.__roscfg

  @property
  def Filename(self):
    '''
    Returns an existing path with file name or an empty string.
    @rtype: C{str}
    '''
    if os.path.isfile(self.__launchFile):
      return self.__launchFile
    elif not (self.__package is None):
      try:
        import roslib
        return roslib.packages.find_resource(self.PackageName, self.LaunchName).pop()
      except Exception:
        raise LaunchConfigException(''.join(['launch file ', self.LaunchName, ' not found!']))

  @property
  def LaunchName(self):
    '''
    Returns the name of the launch file with extension, e.g. 'test.launch'
    @rtype: C{str}
    '''
    return os.path.basename(self.__launchFile)
  
  @property
  def PackageName(self):
    '''
    Returns the name of the package containing the launch file or None.
    @rtype: C{str} or C{None}
    '''
    return self.__package
  
  def __getPackageName(self, dir):
    if not (dir is None) and dir and dir != '/' and os.path.isdir(dir):
      package = os.path.basename(dir)
      fileList = os.listdir(dir)
      for file in fileList:
        if file == 'manifest.xml':
            return package
      return self.__getPackageName(os.path.dirname(dir))
    return None

  def _index(self, text, regexp_list):
    '''
    Searches in the given text for key indicates the including of a file and 
    return their index.
    @param text:
    @type text: C{str}
    @param regexp_list:
    @type regexp_list: C{[L{QtCore.QRegExp},..]}
    @return: the index of the including key or -1
    @rtype: C{int}
    '''
    for pattern in regexp_list:
      index = pattern.indexIn(text)
      if index > -1:
        return index
    return -1

  def interpretPath(self, path, pwd='.'):
    '''
    Tries to determine the path of the included file. The statement of 
    $(find 'package') will be resolved.
    @param path: the sting which contains the included path
    @type path: C{str}
    @param pwd: current working path
    @type pwd: C{str}
    @return: if no leading L{os.sep} is detected, the path set by L{setCurrentPath()}
    will be prepend. C{$(find 'package')} will be resolved. Otherwise the parameter 
    itself will be returned
    @rtype: C{str} 
    '''
    path = path.strip()
    index = path.find('$')
    if index > -1:
      startIndex = path.find('(', index)
      if startIndex > -1:
        endIndex = path.find(')', startIndex+1)
        script = path[startIndex+1:endIndex].split()
        if len(script) == 2 and (script[0] == 'find'):
          pkg = roslib.packages.get_pkg_dir(script[1])
          return ''.join([pkg, '/', path[endIndex+1:]])
    elif len(path) > 0 and path[0] != '/':
      return ''.join([pwd, '/', path])
    return path

  def getIncludedFiles(self):
    '''
    Reads the configuration file and searches for included files. This files
    will be returned in a list.
    @return: the list with all files needed for the configuration
    @rtype: C{[str,...]}
    '''
    result = set(self.__roscfg.roslaunch_files)
    regexp_list = [QtCore.QRegExp("\\binclude\\b"), QtCore.QRegExp("\\btextfile\\b"),
                   QtCore.QRegExp("\\bfile\\b")]
    file = QtCore.QFile(self.__launchFile)
    if file.open(QtCore.QIODevice.ReadOnly | QtCore.QIODevice.Text):
      while not file.atEnd():
        line = str(file.readLine()) # A QByteArray
        index = self._index(line, regexp_list)
        if index > -1:
          startIndex = line.find('"', index)
          if startIndex > -1:
            endIndex = line.find('"', startIndex+1)
            fileName = line[startIndex+1:endIndex]
            if len(fileName) > 0:
              f = QtCore.QFile(self.interpretPath(fileName, os.path.dirname(self.__launchFile)))
              if f.exists():
                result.add(f.fileName())
    return list(result)

  def load(self, argv):
    '''
    @param argv: the list with argv parameter needed to load the launch file. 
                 The name and value are separated by C{:=}
    @type argv: C{[str]}
    @return: a tupel of the laod result and the the list with argv
    @rtype: C{(bool, [str])}
    @raise LaunchConfigException: on load errors
    '''
    import re
    testarg = argv
    doTest = True
    argvAdded = False
    while doTest:
      try:
        roscfg = roslaunch.ROSLaunchConfig()
        loader = roslaunch.XmlLoader()
        loader.load(self.Filename, roscfg, verbose=False, argv=testarg)
        self.__roscfg = roscfg
        files = self.file_watcher.files()
        if files:
          self.file_watcher.removePaths(files)
        self.file_watcher.addPaths(self.getIncludedFiles())
        doTest = False
      except roslaunch.XmlParseException, e:
        result = list(re.finditer(r"requires the '\w+' arg to be set", str(e)))
        if not result:
          message = str(e)
          # NO environment substitution !!!
          test = list(re.finditer(r"environment variable '\w+' is not set", str(e)))
          if test:
            message = ''.join([message, '\n', 'environment substitution is not supported, use "arg" instead!'])
          raise LaunchConfigException(message)
        for m in result:
          argName = m.group(0).split("'")[1]
          testarg.append(''.join([argName, ':=', '$[', argName, ']']))
          argvAdded = True
    return not argvAdded, testarg

  def getRobotDescr(self):
    '''
    @return: the robot description stored in the configuration
    @rtype: C{dict(robot:dict({'robot_type', 'robot_name', 'robot_descr'}:str(value)))}
    '''
    result = dict()
    if not self.Roscfg is None:
      for param, p in self.Roscfg.params.items():
        t = ''
        if param.endswith('robot_type'):
          t = 'robot_type'
        elif param.endswith('robot_name'):
          t = 'robot_name'
        elif param.endswith('robot_descr'):
          t = 'robot_descr'
        if t:
          robot = roslib.names.namespace(param).strip(roslib.names.SEP)
          if not result.has_key(robot):
            result[robot] = dict()
            result[robot]['robot_type'] = ''
            result[robot]['robot_name'] = ''
            result[robot]['robot_descr'] = ''
          result[robot][t] = p.value.replace("\\n ", "\n")
    return result

  def getSensorDesrc(self):
    '''
    @return: the sensor description stored in the configuration
    @rtype: C{dict(robot: dict(sensor:dict({'sensor_type', 'sensor_name', 'sensor_descr'}:str(value))))}
    '''
    # get the sensor description
    result = dict()
    if not self.Roscfg is None:
      for param, p in self.Roscfg.params.items():
        t = ''
        if param.endswith('sensor_type'):
          t = 'sensor_type'
        elif param.endswith('sensor_name'):
          t = 'sensor_name'
        elif param.endswith('sensor_descr'):
          t = 'sensor_descr'
        if t:
          node = roslib.names.namespace(param).strip(roslib.names.SEP)
          node = ''.join([roslib.names.SEP, node])
          machine = ''
          for item in self.Roscfg.nodes:
            if roslib.names.ns_join(item.namespace, item.name) == node:
              if item.machine_name:
                machine = self.Roscfg.machines[item.machine_name].name
          if not result.has_key(machine):
            result[machine] = dict()
          if not result[machine].has_key(node):
            result[machine][node] = dict()
            result[machine][node]['sensor_type'] = ''
            result[machine][node]['sensor_name'] = ''
            result[machine][node]['sensor_descr'] = ''
          result[machine][node][t] = p.value.replace("\\n ", "\n")
    return result
  
  def argvToDict(self, argv):
    result = dict()
    for a in argv:
      key, sep, value = a.partition(':=')
      if sep:
        result[key] = value
    return result

  def getArgHistory(self):
    '''
    Reads the arguments from a history file stored in a defined configuration path.
    @return: the dictionary with arguments
    @rtype: C{dict(str(name):[str(value), ...], ...)}
    '''
    result = {}
    historyFile = ''.join([self.CFG_PATH, 'arg.history'])
    if os.path.isfile(historyFile):
      with open(historyFile, 'r') as f:
        line = f.readline()
        while line:
          if line:
            line = line.strip()
            if line:
              key, sep, value = line.partition(':=')
              if sep:
                if not key in result.keys():
                  result[key] = [value]
                else:
                  result[key].insert(0, value)
          line = f.readline()
      f.closed
    return result

  def addToArgHistory(self, key, value):
    '''
    Adds an argument the history file and save it in a defined configuration path 
    under the name 'arg.history'. For a key there stored up to 
    L{node_manager_fkie.ARG_HISTORY_LENGTH} values.
    @param key: the argument name
    @type key: C{str}
    @param value: the argument value
    @type value: C{str}
    '''
    history = self.getArgHistory()
    save = False
    if not key in history:
      history[key] = [value]
      save = True
    else:
      if not value in history[key]:
        if len(history[key]) >= nm.ARG_HISTORY_LENGTH:
          history[key].pop()
        history[key].append(value)
        save = True
    # safe history
    if save:
      if not os.path.isdir(self.CFG_PATH):
        os.makedirs(self.CFG_PATH)
      with open(''.join([self.CFG_PATH, 'arg.history']), 'w') as f:
        for key in history.keys():
          for value in history[key]:
            f.write(''.join([key, ':=', value, '\n']))
      f.closed

  def getNode(self, name):
    '''
    Returns a configuration node for a given node name.
    @param name: the name of the node.
    @type name: C{str}
    @return: the configuration node stored in this configuration
    @rtype: L{roslaunch.Node} or C{None}
    '''
    nodename = os.path.basename(name)
    namespace = os.path.dirname(name).strip('/')
    for item in self.Roscfg.nodes:
      if (item.name == nodename) and (item.namespace.strip('/') == namespace):
        return item
    return None


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
import re
import sys
import time
import roslaunch
import roslib

from xml.dom.minidom import parse#, parseString
#from xml.dom import Node as DomNode #avoid aliasing

from python_qt_binding import QtCore

from master_discovery_fkie.common import resolve_url
import node_manager_fkie as nm
from common import package_name, resolve_paths

class LaunchConfigException(Exception):
  pass


class LaunchConfig(QtCore.QObject):
  '''
  A class to handle the ROS configuration stored in launch file.
  '''
  
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
    self.__package = package_name(os.path.dirname(self.__launchFile))[0] if package is None else package 
    self.__masteruri = masteruri if not masteruri is None else 'localhost'
    self.__roscfg = None
    self.argv = argv
    self.__reqTested = False
    self.__argv_values = dict()
    self.global_param_done = [] # masteruri's where the global parameters are registered 
    self.hostname = nm.nameres().getHostname(self.__masteruri)
    self.__launch_id = '%.9f'%time.time()
    nm.file_watcher().add(self.__masteruri, self.__launchFile, self.__launch_id, [self.__launchFile])
#    nm.file_watcher().add(self.__masteruri, self.__launchFile, self.__launch_id, self.getIncludedFiles(self.Filename))


  def __del__(self):
    # Delete to avoid segfault if the LaunchConfig class is destroyed recently 
    # after creation and xmlrpclib.ServerProxy process a method call.
#    del self.file_watcher
    nm.file_watcher().rem(self.__masteruri, self.__launchFile, self.__launch_id)

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
      result, _ = self.load(self.argv)#_:=argv
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
        return roslib.packages.find_resource(self.PackageName, self.LaunchName).pop()
      except Exception:
        raise LaunchConfigException(''.join(['launch file ', self.LaunchName, ' not found!']))
    raise LaunchConfigException(''.join(['launch file ', self.__launchFile, ' not found!']))

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
  
  @classmethod
  def _index(cls, text, regexp_list):
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

  @classmethod
  def interpretPath(cls, path, pwd='.'):
    '''
    Tries to determine the path of the included file. The statement of 
    $(find 'package') will be resolved.
    The supported URL begins with `file:///`, `package://` or `pkg://`.
    The package URL will be resolved to a valid file path. If the file is in a
    subdirectory, you can replace the subdirectory by `///`. 
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
    startIndex = path.find('$(')
    if startIndex > -1:
      endIndex = path.find(')', startIndex+2)
      script = path[startIndex+2:endIndex].split()
      if len(script) == 2 and (script[0] == 'find'):
        pkg = roslib.packages.get_pkg_dir(script[1])
        return os.path.join(pkg, path[endIndex+2:].strip(os.path.sep))
    elif len(path) > 0 and path[0] != os.path.sep:
      try:
        return resolve_url(path)
      except ValueError, _:
        if len(path) > 0 and path[0] != os.path.sep:
          return os.path.normpath(''.join([pwd, os.path.sep, path]))
    return path

  @classmethod
  def getIncludedFiles(cls, inc_file, regexp_list=[QtCore.QRegExp("\\binclude\\b"),
                                               QtCore.QRegExp("\\btextfile\\b"),
                                               QtCore.QRegExp("\\bfile\\b")]):
    '''
    Reads the configuration file and searches for included files. This files
    will be returned in a list.
    @param inc_file: path of the ROS launch file
    @param regexp_list: pattern of 
    @return: the list with all files needed for the configuration
    @rtype: C{[str,...]}
    '''
    result = set()
    with open(inc_file, 'r') as f:
      content = f.read()
      # remove the comments
      comment_pattern = QtCore.QRegExp("<!--.*?-->")
      pos = comment_pattern.indexIn(content)
      while pos != -1:
        content = content[:pos] + content[pos+comment_pattern.matchedLength():]
        pos = comment_pattern.indexIn(content)
      lines = content.splitlines()
    for line in lines:
      index = cls._index(line, regexp_list)
      if index > -1:
        startIndex = line.find('"', index)
        if startIndex > -1:
          endIndex = line.find('"', startIndex+1)
          fileName = line[startIndex+1:endIndex]
          if len(fileName) > 0:
            try:
              path = cls.interpretPath(fileName, os.path.dirname(inc_file))
              if os.path.isfile(path):
                result.add(path)
                if path.endswith('.launch'):
                  result.update(cls.getIncludedFiles(path, regexp_list))
            except:
              pass
    return list(result)

  def load(self, argv):
    '''
    @param argv: the list with argv parameter needed to load the launch file. 
                 The name and value are separated by C{:=}
    @type argv: C{[str]}
    @return True, if the launch file was loaded
    @rtype boolean
    @raise LaunchConfigException: on load errors
    '''
    try:
      roscfg = roslaunch.ROSLaunchConfig()
      loader = roslaunch.XmlLoader()
      self.argv = self.resolveArgs(argv)
      loader.load(self.Filename, roscfg, verbose=False, argv=self.argv)
      self.__roscfg = roscfg
      nm.file_watcher().add(self.__masteruri, self.__launchFile, self.__launch_id, self.getIncludedFiles(self.Filename))
      if not nm.is_local(nm.nameres().getHostname(self.__masteruri)):
        nm.file_watcher_param().add(self.__masteruri, self.__launchFile, self.__launch_id, 
                                    self.getIncludedFiles(self.Filename, 
                                                          regexp_list = [QtCore.QRegExp("\\bvalue=.*pkg:\/\/\\b"),
                                                                         QtCore.QRegExp("\\bvalue=.*package:\/\/\\b"),
                                                                         QtCore.QRegExp("\\bvalue=.*\$\(find\\b")])
                                                          )
    except roslaunch.XmlParseException, e:
      test = list(re.finditer(r"environment variable '\w+' is not set", str(e)))
      message = str(e)
      if test:
        message = ''.join([message, '\n', 'environment substitution is not supported, use "arg" instead!'])
      raise LaunchConfigException(message)
    return True, self.argv

  def resolveArgs(self, argv):
    argv_dict = self.argvToDict(argv)
    # replace $(arg ...) in arg values
    for k, _ in argv_dict.items():
      self._replaceArg(k,argv_dict, self.__argv_values)
    return ["%s:=%s"%(k,v) for k, v in argv_dict.items()]
  
  def _replaceArg(self, arg, argv_defaults, argv_values):
    '''
    Replace the arg-tags in the value in given argument recursively.
    '''
    rec_inc = 0
    value = argv_defaults[arg]
    arg_match = re.search(r"\$\(\s*arg\s*", value)
    while not arg_match is None:
      rec_inc += 1
      endIndex = value.find(')', arg_match.end())
      if endIndex > -1:
        arg_name = value[arg_match.end():endIndex].strip()
        if arg == arg_name:
          raise LaunchConfigException("Can't resolve the argument `%s` argument: the argument referenced to itself!"%arg_name)
        if rec_inc > 100:
          raise LaunchConfigException("Can't resolve the argument `%s` in `%s` argument: recursion depth of 100 reached!"%(arg_name, arg))
        if argv_defaults.has_key(arg_name):
          argv_defaults[arg] = value.replace(value[arg_match.start():endIndex+1], argv_defaults[arg_name])
        elif argv_values.has_key(arg_name):
          argv_defaults[arg] = value.replace(value[arg_match.start():endIndex+1], argv_values[arg_name])
        else:
          raise LaunchConfigException("Can't resolve the argument `%s` in `%s` argument"%(arg_name, arg))
      else:
        raise LaunchConfigException("Can't resolve the argument in `%s` argument: `)` not found"%arg)
      value = argv_defaults[arg]
      arg_match = re.search(r"\$\(\s*arg\s*", value)

  def getArgs(self):
    '''
    @return: a list with args being used in the roslaunch file. Only arg tags that are a direct child of <launch> will 
             be returned
    @rtype: C{[str]}
    @raise roslaunch.XmlParseException: on parse errors
    '''
    self._argv_values = dict()
    arg_subs = []
    args = []
#    for filename in self.getIncludedFiles(self.Filename):
    # get only the args in the top launch file
    for filename in [self.Filename]:
      try: 
        if filename.endswith('.launch'):
          args[len(args):-1] = parse(filename).getElementsByTagName('arg')
      except Exception as e: 
        raise roslaunch.XmlParseException("Invalid roslaunch XML syntax: %s"%e) 

      for arg in args:
        arg_name = arg.getAttribute("name")
        if not arg_name:
          raise roslaunch.XmlParseException("arg tag needs a name, xml is %s"%arg.toxml())

        # we only want argsargs at top level:
        if not arg.parentNode.tagName=="launch":
          continue 

        arg_default = arg.getAttribute("default")
        arg_value = arg.getAttribute("value")
        arg_sub = ''.join([arg_name, ':=', arg_default])
        if (not arg_value) and not arg_sub in arg_subs:
          arg_subs.append(arg_sub)
        elif arg_value:
          self.__argv_values[arg_name] = arg_value

    return arg_subs

  def _decode(self, val):
    result = val.replace("\\n ", "\n")
    try:
      result = result.decode(sys.getfilesystemencoding())
    except:
      pass
    return result

  def getRobotDescr(self):
    '''
    Parses the launch file for C{robots} parameter to get the description of the
    robot.
    @return: the robot description stored in the configuration
    @rtype: C{dict(robot:dict('type' :str, 'name': str, 'images' : [str], 'description': str))}
    '''
    result = dict()
    if not self.Roscfg is None:
      for param, p in self.Roscfg.params.items():
        if param.endswith('robots'):
          if isinstance(p.value, list):
            if len(p.value) > 0 and len(p.value[0]) != 5:
              print "WRONG format, expected: ['host', 'type', 'name', 'images', 'description'] -> ignore", param
            else:
              for entry in p.value:
                result[entry[0]] = { 'type' : entry[1], 'name' : entry[2], 'images' : resolve_paths(entry[3]).split(','), 'description' : resolve_paths(self._decode(entry[4])) }
    return result

  def getCapabilitiesDesrc(self):
    '''
    Parses the launch file for C{capabilities} and C{capability_group} parameter 
    and creates  dictionary for grouping the nodes.
    @return: the capabilities description stored in this configuration
    @rtype: C{dict(machine : dict(namespace: dict(group:dict('type' : str, 'images' : [str], 'description' : str, 'nodes' : [str]))))}
    '''
    result = dict()
    capabilies_descr = dict()
    if not self.Roscfg is None:
      # get the capabilities description
      # use two separate loops, to create the description list first
      # TODO read the group description depending on namespace 
      for param, p in self.Roscfg.params.items():
        if param.endswith('capabilities'):
          if isinstance(p.value, list):
            if len(p.value) > 0 and len(p.value[0]) != 4:
              print "WRONG format, expected: ['name', 'type', 'images', 'description'] -> ignore", param
            else:
              for entry in p.value:
                capabilies_descr[entry[0]] = { 'type' : ''.join([entry[1]]), 'images' : resolve_paths(entry[2]).split(','), 'description' : resolve_paths(self._decode(entry[3]))}
      # get the capability nodes
      for item in self.Roscfg.nodes:
        node_fullname = roslib.names.ns_join(item.namespace, item.name)
        machine_name = item.machine_name if not item.machine_name is None and not item.machine_name == 'localhost' else ''
        added = False
        cap_param = roslib.names.ns_join(node_fullname, 'capability_group')
        cap_ns = node_fullname
        #find the capability group parameter in namespace
        while not self.Roscfg.params.has_key(cap_param) and cap_param.count(roslib.names.SEP) > 1:
          cap_ns = roslib.names.namespace(cap_ns).rstrip(roslib.names.SEP)
          if not cap_ns:
            cap_ns = roslib.names.SEP
          cap_param = roslib.names.ns_join(cap_ns, 'capability_group')
        if cap_ns == node_fullname:
          cap_ns = item.namespace.rstrip(roslib.names.SEP)
        # if the 'capability_group' parameter found, assign node to the group
        if self.Roscfg.params.has_key(cap_param) and self.Roscfg.params[cap_param].value:
          p = self.Roscfg.params[cap_param]
          if not result.has_key(machine_name):
            result[machine_name] = dict()
          for (ns, groups) in result[machine_name].items():
            if ns == cap_ns and groups.has_key(p.value):
              groups[p.value]['nodes'].append(node_fullname)
              added = True
              break
          if not added:
            ns = cap_ns
            # add new group in the namespace of the node
            if not result[machine_name].has_key(ns):
              result[machine_name][ns] = dict()
            if not result[machine_name][ns].has_key(p.value):
              try:
                result[machine_name][ns][p.value] = { 'type' : capabilies_descr[p.value]['type'], 'images': capabilies_descr[p.value]['images'], 'description' : capabilies_descr[p.value]['description'], 'nodes' : [] }
              except:
                result[machine_name][ns][p.value] = { 'type' : '', 'images': [], 'description' : '', 'nodes' : [] }
            result[machine_name][ns][p.value]['nodes'].append(node_fullname)
    return result
  
  def argvToDict(self, argv):
    result = dict()
    for a in argv:
      key, sep, value = a.partition(':=')
      if sep:
        result[key] = value
    return result

  def getNode(self, name):
    '''
    Returns a configuration node for a given node name.
    @param name: the name of the node.
    @type name: C{str}
    @return: the configuration node stored in this configuration
    @rtype: L{roslaunch.Node} or C{None}
    '''
    nodename = os.path.basename(name)
    namespace = os.path.dirname(name).strip(roslib.names.SEP)
    for item in self.Roscfg.nodes:
      if (item.name == nodename) and (item.namespace.strip(roslib.names.SEP) == namespace):
        return item
    return None

  def get_robot_icon(self):
    '''
    Returns the value of the `/robot_icon` parameter or None 
    '''
    try:
      return self.Roscfg.params['/robot_icon'].value
    except:
      pass
    return None

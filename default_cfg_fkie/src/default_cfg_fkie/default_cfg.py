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
import sys
import shlex
import subprocess
import threading

import roslib
import roslib.names
import rospy
import roslib.network
from ros import roslaunch
import rosgraph.masterapi


from multimaster_msgs_fkie.msg import Capability
from multimaster_msgs_fkie.srv import ListDescription, ListNodes, LoadLaunch, Task, ListDescriptionResponse, ListNodesResponse
from screen_handler import ScreenHandler, ScreenHandlerException

class LoadException(Exception):
  ''' The exception throwing while searching for the given launch file. '''
  pass
class StartException(Exception):
  ''' The exception throwing while run a node containing in the loaded configuration. '''
  pass


class DefaultCfg(object):
  
  def __init__(self):
    self.nodes = [] 
    '''@var: the list with names of nodes with name spaces '''
    self.sensors = {}
    '''@ivar: Sensor description: C{dict(node name : [(sensor type, sensor name, sensor description), ...])}'''
    self.robot_descr = ('', '', '')
    '''@ivar: robot description as tupel of (type, name, text) '''
    self.package = ''
    self.file = ''
    self.__lock = threading.RLock()
    # initialize the ROS services
#    rospy.Service('~load', LoadLaunch, self.rosservice_load_launch)
    rospy.Service('~description', ListDescription, self.rosservice_description)
    self.runService = None
    '''@ivar: The service will be created on each load of a launch file to
    inform the caller about a new configuration. '''
    self.listService = None
    '''@ivar: The service will be created on each load of a launch file to
    inform the caller about a new configuration. '''
    self.description_response = ListDescriptionResponse()
#    self.global_parameter_setted = False

  
  def load(self, package, file, argv):
    '''
    Load the launch file configuration
    @param package: the package containing the launch file, or empty string to load
    the given file
    @type package: C{str}
    @param file: the launch file or complete path, if the package is empty
    @type file: C{str}
    @param argv: the argv needed to load the launch file
    @type argv: C{str}
    '''
    with self.__lock:
      # shutdown the services to inform the caller about a new configuration.
      if not self.runService is None:
        self.runService.shutdown('reload config')
      self.runService = None
      if not self.listService is None:
        self.listService.shutdown('reload config')
      self.listService = None
      self.nodes = [] # the name of nodes with namespace
      self.sensors = {} # sensor descriptions
      self.launch_file = launch_file = self.getPath(file, package)
      rospy.loginfo("loading launch file: %s", launch_file)
      self.masteruri = self._masteruri_from_ros()
      self.roscfg = roslaunch.ROSLaunchConfig()
      loader = roslaunch.XmlLoader()
      loader.load(launch_file, self.roscfg, verbose=False, argv=argv)
      # create the list with node names
      for item in self.roscfg.nodes:
        if item.machine_name and not item.machine_name == 'localhost':
          machine = self.roscfg.machines[item.machine_name]
          if roslib.network.is_local_address(machine.address):
            self.nodes.append(str(''.join([item.namespace, item.name])))
        else:
          self.nodes.append(str(''.join([item.namespace, item.name])))
      # get the robot description
      self.description_response = dr = ListDescriptionResponse()
      dr.robot_name = ''
      dr.robot_type = ''
      dr.robot_descr = ''
      for param, p in self.roscfg.params.items():
        if param.endswith('robots'):
          if isinstance(p.value, list):
            if len(p.value) > 0 and len(p.value[0]) != 5:
              print "WRONG format, expected: ['host(ROS master Name)', 'type', 'name', 'images', 'description']  -> ignore", param
            else:
              for entry in p.value:
                try:
                  print entry[0], rospy.get_param('/mastername', '')
                  if not entry[0] or entry[0] == rospy.get_param('/mastername', ''):
                    dr.robot_name = self._decode(entry[2])
                    dr.robot_type = entry[1]
                    dr.robot_images = entry[3].split()
                    dr.robot_descr = self._decode(entry[4])
                    break
                except:
                  pass
      # get the sensor description
      tmp_cap_dict = self.getCapabilitiesDesrc()
      for machine, ns_dict in tmp_cap_dict.items():
        if self.roscfg.machines.has_key(machine):
          machine = self.roscfg.machines[machine].address
        if not machine or roslib.network.is_local_address(machine):
          for ns, group_dict in ns_dict.items():
            for group, descr_dict in group_dict.items():
              if descr_dict['nodes']:
                cap = Capability()
                cap.namespace = ns
                cap.name = group
                cap.type = descr_dict['type']
                cap.images = list(descr_dict['images'])
                cap.description = descr_dict['description']
                cap.nodes = list(descr_dict['nodes'])
                dr.capabilities.append(cap)
      # initialize the ROS services
      self._timed_service_creation()
      #HACK to let the node_manager to update the view
#      t = threading.Timer(2.0, self._timed_service_creation)
#      t.start()
      self.loadParams()
  #    self.timer = rospy.Timer(rospy.Duration(2), self.timed_service_creation, True)
  #    if self.nodes:
  #      self.runService = rospy.Service('~run', Task, self.rosservice_start_node)
  #    self.listServic = rospy.Service('~list_nodes', ListNodes, self.rosservice_list_nodes)
#    except:
#      import traceback
#      print traceback.format_exc()

  def _decode(self, val):
    '''
    Replaces the '\\n' by '\n' and decode the string entry from system default 
    coding to unicode.
    @param val: the string coding as system default
    @return: the decoded string
    @rtype: C{unicode} or original on error
    '''
    result = val.replace("\\n ", "\n")
    try:
      result = result.decode(sys.getfilesystemencoding())
    except:
      pass
    return result

  def getCapabilitiesDesrc(self):
    '''
    Parses the launch file for C{capabilities} and C{capability_group} parameter 
    and creates  dictionary for grouping the nodes.
    @return: the capabilities description stored in this configuration
    @rtype: C{dict(machine : dict(namespace: dict(group:dict('type' : str, 'description' : str, 'nodes' : [str]))))}
    '''
    result = dict()
    capabilies_descr = dict()
    if not self.roscfg is None:
      # get the capabilities description
      # use two separate loops, to create the description list first
      for param, p in self.roscfg.params.items():
        if param.endswith('capabilities'):
          if isinstance(p.value, list):
            if len(p.value) > 0 and len(p.value[0]) != 4:
              print "WRONG format, expected: ['name', 'type', 'images', 'description'] -> ignore", param
            else:
              for entry in p.value:
                capabilies_descr[entry[0]] = { 'type' : ''.join([entry[1]]), 'images' : entry[2].split(), 'description' : self._decode(entry[3])}
      # get the capability nodes
      for item in self.roscfg.nodes:
        node_fullname = roslib.names.ns_join(item.namespace, item.name)
        machine_name = item.machine_name if not item.machine_name is None and not item.machine_name == 'localhost' else ''
        added = False
        cap_param = roslib.names.ns_join(node_fullname, 'capability_group')
        cap_ns = node_fullname
        #find the capability group parameter in namespace
        while not self.roscfg.params.has_key(cap_param) and cap_param.count(roslib.names.SEP) > 1:
          cap_ns = roslib.names.namespace(cap_ns).rstrip(roslib.names.SEP)
          if not cap_ns:
            cap_ns = roslib.names.SEP
          cap_param = roslib.names.ns_join(cap_ns, 'capability_group')
        if cap_ns == node_fullname:
         cap_ns = item.namespace.rstrip(roslib.names.SEP)        # if the parameter group parameter found, assign node to the group
        # if the 'capability_group' parameter found, assign node to the group
        if self.roscfg.params.has_key(cap_param):
          p = self.roscfg.params[cap_param]
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

  def _masteruri_from_ros(self):
    '''
    Returns the master URI depending on ROS distribution API.
    @return: ROS master URI
    @rtype: C{str}
    '''
    try:
      import rospkg.distro
      distro = rospkg.distro.current_distro_codename()
      if distro in ['electric', 'diamondback', 'cturtle']:
        return roslib.rosenv.get_master_uri()
      else:
        import rosgraph
        return rosgraph.rosenv.get_master_uri()
    except:
      return roslib.rosenv.get_master_uri()

  def _timed_service_creation(self):
    with self.__lock:
      try:
        if self.runService is None:
          self.runService = rospy.Service('~run', Task, self.rosservice_start_node)
        if self.listService is None:
          self.listService = rospy.Service('~list_nodes', ListNodes, self.rosservice_list_nodes)
      except:
        import traceback
        print traceback.format_exc()

  def getPath(self, file, package=''):
    '''
    Searches for a launch file. If package is given, try first to find the launch
    file in the given package. If more then one launch file with the same name 
    found in the package, the first one will be tacked.
    @param file: the file name of the launch file
    @type file: C{str}
    @param package: the package containing the launch file or an empty string, 
    if the C{file} is an absolute path
    @type package: C{str}
    @return: the absolute path of the launch file
    @rtype: C{str}
    @raise LoadException: if the given file is not found 
    '''
    launch_file = file
    # if package is set, try to find the launch file in the given package
    if package:
      paths = roslib.packages.find_resource(package, launch_file)
      if len(paths) > 0:
        # if more then one launch file is found, take the first one
        launch_file = paths[0]
    if os.path.isfile(launch_file) and os.path.exists(launch_file):
      return launch_file
    raise LoadException(str(' '.join(['File', file, 'in package ', package, 'not found'])))

  def rosservice_list_nodes(self, req):
    '''
    Callback for the ROS service to get the list with available nodes.
    '''
    return ListNodesResponse(self.nodes)

  def rosservice_start_node(self, req):
    '''
    Callback for the ROS service to start a node.
    '''
    self.runNode(req.node)
    return []
#    except:
#      import traceback
#      return TaskResponse(str(traceback.format_exc().splitlines()[-1]))
#    return TaskResponse('')

#  def rosservice_load_launch(self, req):
#    '''
#    Load the launch file
#    '''
#    try:
#      self.__lock.acquire()
#      self.load(req.package, req.file, req.argv)
#    finally:
#      self.__lock.release()
#    return []

  def rosservice_description(self, req):
    '''
    Returns the current description.
    '''
    return self.description_response
  
  def loadParams(self):
    '''
    Loads all parameter into ROS parameter server.
    '''
    params = dict()
    for param, value in self.roscfg.params.items():
      params[param] = value
#      rospy.loginfo("register PARAMS:\n%s", '\n'.join(params))
    self._load_parameters(self.masteruri, params, self.roscfg.clear_params)

  
  def runNode(self, node):
    '''
    Start the node with given name from the currently loaded configuration.
    @param node: the name of the node
    @type node: C{str}
    @raise StartException: if an error occurred while start.
    '''
    n = None
    nodename = os.path.basename(node)
    namespace = os.path.dirname(node).strip('/')
    for item in self.roscfg.nodes:
      if (item.name == nodename) and (item.namespace.strip('/') == namespace):
        n = item
        break
    if n is None:
      raise StartException(''.join(["Node '", node, "' not found!"]))
    
#    env = n.env_args
    prefix = n.launch_prefix if not n.launch_prefix is None else ''
    args = [''.join(['__ns:=', n.namespace]), ''.join(['__name:=', n.name])]
    if not (n.cwd is None):
      args.append(''.join(['__cwd:=', n.cwd]))
    
    # add remaps
    for remap in n.remap_args:
      args.append(''.join([remap[0], ':=', remap[1]]))

#    masteruri = self.masteruri
    
#    if n.machine_name and not n.machine_name == 'localhost':
#      machine = self.roscfg.machines[n.machine_name]
      #TODO: env-loader support?
#      if machine.env_args:
#        env[len(env):] = machine.env_args

#    nm.screen().testScreen()
    try:
      cmd = roslib.packages.find_node(n.package, n.type)
    except roslib.packages.ROSPkgException as e:
      # multiple nodes, invalid package
      raise StartException(str(e))
    # handle diferent result types str or array of string
    import types
    if isinstance(cmd, types.StringTypes):
      cmd = [cmd]
    if cmd is None or len(cmd) == 0:
      raise StartException(' '.join([n.type, 'in package [', n.package, '] not found!']))
    # determine the current working path, Default: the package of the node
    cwd = self.get_ros_home()
    if not (n.cwd is None):
      if n.cwd == 'ROS_HOME':
        cwd = self.get_ros_home()
      elif n.cwd == 'node':
        cwd = os.path.dirname(cmd[0])
    node_cmd = ['rosrun node_manager_fkie respawn' if n.respawn else '', prefix, cmd[0]]
    cmd_args = [ScreenHandler.getSceenCmd(node)]
    cmd_args[len(cmd_args):] = node_cmd
    cmd_args.append(n.args)
    cmd_args[len(cmd_args):] = args
#    print 'runNode: ', cmd_args
    popen_cmd = shlex.split(str(' '.join(cmd_args)))
    rospy.loginfo("run node '%s as': %s", node, str(' '.join(popen_cmd)))
    new_env = dict(os.environ)
    for k, v in n.env_args:
      new_env[k] = v
    ps = subprocess.Popen(popen_cmd, cwd=cwd, env=new_env)
    # wait for process to avoid 'defunct' processes
    thread = threading.Thread(target=ps.wait)
    thread.setDaemon(True)
    thread.start()
#    subprocess.Popen(popen_cmd, cwd=cwd)
    if len(cmd) > 1:
      raise StartException('Multiple executables are found! The first one was started! Exceutables:\n' + str(cmd))

  def get_ros_home(self):
    '''
    Returns the ROS HOME depending on ROS distribution API.
    @return: ROS HOME path
    @rtype: C{str}
    '''
    try:
      import rospkg.distro
      distro = rospkg.distro.current_distro_codename()
      if distro in ['electric', 'diamondback', 'cturtle']:
        import roslib.rosenv
        return roslib.rosenv.get_ros_home()
      else:
        import rospkg
        return rospkg.get_ros_home()
    except:
      import traceback
      print traceback.format_exc()
      import roslib.rosenv
      return roslib.rosenv.get_ros_home()

#  @classmethod
#  def getGlobalParams(cls, roscfg):
#    '''
#    Return the parameter of the configuration file, which are not associated with 
#    any nodes in the configuration.
#    @param roscfg: the launch configuration
#    @type roscfg: L{roslaunch.ROSLaunchConfig}
#    @return: the list with names of the global parameter
#    @rtype: C{dict(param:value, ...)}
#    '''
#    result = dict()
#    nodes = []
#    for item in roscfg.resolved_node_names:
#      nodes.append(item)
#    for param, value in roscfg.params.items():
#      nodesparam = False
#      for n in nodes:
#        if param.startswith(n):
#          nodesparam = True
#          break
#      if not nodesparam:
#        result[param] = value
#    return result

  @classmethod
  def _load_parameters(cls, masteruri, params, clear_params):
    """
    Load parameters onto the parameter server
    """
    import roslaunch
    import roslaunch.launch
    import xmlrpclib
    param_server = xmlrpclib.ServerProxy(masteruri)
    p = None
    try:
      # multi-call style xmlrpc
      param_server_multi = xmlrpclib.MultiCall(param_server)

      # clear specified parameter namespaces
      # #2468 unify clear params to prevent error
      for p in clear_params:
        param_server_multi.deleteParam(rospy.get_name(), p)
      r = param_server_multi()
#      for code, msg, _ in r:
#        if code != 1:
#          raise StartException("Failed to clear parameter: %s"%(msg))

      # multi-call objects are not reusable
      param_server_multi = xmlrpclib.MultiCall(param_server)
      for p in params.itervalues():
        # suppressing this as it causes too much spam
        #printlog("setting parameter [%s]"%p.key)
        param_server_multi.setParam(rospy.get_name(), p.key, p.value)
      r  = param_server_multi()
      for code, msg, _ in r:
        if code != 1:
          raise StartException("Failed to set parameter: %s"%(msg))
    except roslaunch.core.RLException, e:
      raise StartException(e)
    except Exception as e:
      raise #re-raise as this is fatal

#  @classmethod
#  def packageName(cls, dir):
#    '''
#    Returns for given directory the package name or None
#    @rtype: C{str} or C{None}
#    '''
#    if not (dir is None) and dir and dir != '/' and os.path.isdir(dir):
#      package = os.path.basename(dir)
#      fileList = os.listdir(dir)
#      for file in fileList:
#        if file == 'manifest.xml':
#            return package
#      return cls.packageName(os.path.dirname(dir))
#    return None

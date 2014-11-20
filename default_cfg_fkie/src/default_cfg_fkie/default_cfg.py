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

import roslib.names
import roslib.network
import rospy
from roslaunch import ROSLaunchConfig, XmlLoader
import rosgraph.masterapi
import rosgraph.names
from rosgraph.rosenv import ROS_NAMESPACE

import std_srvs.srv

from multimaster_msgs_fkie.msg import Capability
from multimaster_msgs_fkie.srv import ListDescription, ListNodes, Task, ListDescriptionResponse, ListNodesResponse#, LoadLaunch
from screen_handler import ScreenHandler#, ScreenHandlerException

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
    # Load parameter
    self.launch_file = rospy.get_param('~launch_file', '')
    rospy.loginfo("launch_file: %s"%self.launch_file)
    self.package = rospy.get_param('~package', '')
    rospy.loginfo("package: %s"%self.package)
    self.do_autostart = rospy.get_param('~autostart', False)
    rospy.loginfo("do_autostart: %s"%self.do_autostart)
    self.argv = rospy.get_param('~argv', [])
    rospy.loginfo("argv: %s"%self.argv)
    if not isinstance(self.argv, list):
      self.argv = ["%s"%self.argv]
    sys.argv.extend(self.argv)
    if self.do_autostart:
      rospy.set_param('~autostart', False)
    # initialize the ROS services
#    rospy.Service('~load', LoadLaunch, self.rosservice_load_launch)
    self._reload_service = rospy.Service('~reload', std_srvs.srv.Empty, self.rosservice_reload)
    rospy.Service('~description', ListDescription, self.rosservice_description)
    self.runService = None
    '''@ivar: The service will be created on each load of a launch file to
    inform the caller about a new configuration. '''
    self.listService = None
    '''@ivar: The service will be created on each load of a launch file to
    inform the caller about a new configuration. '''
    self.description_response = ListDescriptionResponse()
    # variables to print the pending autostart nodes 
    self._pending_starts = set()
    self._pending_starts_last_printed = set()

  def load(self, delay_service_creation=0.):
    '''
    Load the launch file configuration
    '''
    with self.__lock:
      self._pending_starts.clear()
      # shutdown the services to inform the caller about a new configuration.
      if not self.runService is None:
        self.runService.shutdown('reload config')
      self.runService = None
      if not self.listService is None:
        self.listService.shutdown('reload config')
      self.listService = None
      self.nodes = [] # the name of nodes with namespace
      self.sensors = {} # sensor descriptions
      launch_path = self.getPath(self.launch_file, self.package)
      rospy.loginfo("loading launch file: %s", launch_path)
      self.masteruri = self._masteruri_from_ros()
      self.roscfg = ROSLaunchConfig()
      loader = XmlLoader()
      argv = [a for a in sys.argv if not a.startswith('__ns:=')]
      # remove namespace from sys.argv to avoid load the launchfile info local namespace
      sys.argv = [a for a in sys.argv if not a.startswith('__ns:=')]
      # set the global environment to empty namespace
      os.environ[ROS_NAMESPACE] = rospy.names.SEP
      loader.load(launch_path, self.roscfg, verbose=False, argv=argv)
      # create the list with node names
      for item in self.roscfg.nodes:
        if item.machine_name and not item.machine_name == 'localhost':
          machine = self.roscfg.machines[item.machine_name]
          if roslib.network.is_local_address(machine.address):
            self.nodes.append(roslib.names.ns_join(item.namespace, item.name))
        else:
          self.nodes.append(roslib.names.ns_join(item.namespace, item.name))
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
                    dr.robot_images = entry[3].split(',')
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
      # load parameters into the ROS parameter server
      self.loadParams()
      # initialize the ROS services
      #HACK to let the node_manager to update the view
      if delay_service_creation > 0.:
        t = threading.Timer(delay_service_creation, self._timed_service_creation)
        t.start()
      else:
        self._timed_service_creation()
  #    self.timer = rospy.Timer(rospy.Duration(2), self.timed_service_creation, True)
  #    if self.nodes:
  #      self.runService = rospy.Service('~run', Task, self.rosservice_start_node)
  #    self.listServic = rospy.Service('~list_nodes', ListNodes, self.rosservice_list_nodes)
#    except:
#      import traceback
#      print traceback.format_exc()
      if self.do_autostart:
        for n in self.nodes:
          try:
            self.runNode(n, self.do_autostart)
          except Exception as e:
            rospy.logwarn("Error while start %s: %s", n, e)
        self.do_autostart = False

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
                capabilies_descr[entry[0]] = { 'type' : ''.join([entry[1]]), 'images' : entry[2].split(','), 'description' : self._decode(entry[3])}
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
        if self.roscfg.params.has_key(cap_param) and self.roscfg.params[cap_param].value:
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

  def getPath(self, path, package=''):
    '''
    Searches for a launch file. If package is given, try first to find the launch
    file in the given package. If more then one launch file with the same name 
    found in the package, the first one will be tacked.
    @param path: the file name of the launch file
    @type path: C{str}
    @param package: the package containing the launch file or an empty string, 
    if the C{file} is an absolute path
    @type package: C{str}
    @return: the absolute path of the launch file
    @rtype: C{str}
    @raise LoadException: if the given file is not found 
    '''
    launch_file = path
    # if package is set, try to find the launch file in the given package
    if package:
      paths = roslib.packages.find_resource(package, launch_file)
      if len(paths) > 0:
        # if more then one launch file is found, take the first one
        launch_file = paths[0]
    if os.path.isfile(launch_file) and os.path.exists(launch_file):
      return launch_file
    raise LoadException('File %s in package [%s] not found!'%(path, package))

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

  def rosservice_reload(self, req):
    self.load(2.)
    return []

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

  def runNode(self, node, autostart=False):
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
      if (item.name == nodename) and ((item.namespace.strip('/') == namespace) or not namespace):
        n = item
        break
    if n is None:
      raise StartException("Node '%s' not found!"%node)

    if autostart and self._get_start_exclude(rospy.names.ns_join(n.namespace, n.name)):
      # skip autostart
      rospy.loginfo("%s is in exclude list, skip autostart", n.name)
      return

#    env = n.env_args
    prefix = n.launch_prefix if not n.launch_prefix is None else ''
    args = ['__ns:=%s'%n.namespace, '__name:=%s'%n.name]
    if not (n.cwd is None):
      args.append('__cwd:=%s'%n.cwd)

    # add remaps
    for remap in n.remap_args:
      args.append('%s:=%s'%(remap[0], remap[1]))

#    masteruri = self.masteruri

#    if n.machine_name and not n.machine_name == 'localhost':
#      machine = self.roscfg.machines[n.machine_name]
      #TODO: env-loader support?
#      if machine.env_args:
#        env[len(env):] = machine.env_args

#    nm.screen().testScreen()
    cmd = self._get_node(n.package, n.type)
    # determine the current working path, Default: the package of the node
    cwd = self.get_ros_home()
    if not (n.cwd is None):
      if n.cwd == 'ROS_HOME':
        cwd = self.get_ros_home()
      elif n.cwd == 'node':
        cwd = os.path.dirname(cmd[0])
    respawn = ['']
    if n.respawn:
      respawn = self._get_node('node_manager_fkie', 'respawn')
      # set the respawn environment variables
      respawn_params = self._get_respawn_params(rospy.names.ns_join(n.namespace, n.name))
      if respawn_params['max'] > 0:
        n.env_args.append(('RESPAWN_MAX', '%d'%respawn_params['max']))
      if respawn_params['min_runtime'] > 0:
        n.env_args.append(('RESPAWN_MIN_RUNTIME', '%d'%respawn_params['min_runtime']))
      if respawn_params['delay'] > 0:
        n.env_args.append(('RESPAWN_DELAY', '%d'%respawn_params['delay']))
    node_cmd = [respawn[0], prefix, cmd[0]]
    cmd_args = [ScreenHandler.getSceenCmd(node)]
    cmd_args[len(cmd_args):] = node_cmd
    cmd_args.append(n.args)
    cmd_args[len(cmd_args):] = args
#    print 'runNode: ', cmd_args
    popen_cmd = shlex.split(str(' '.join(cmd_args)))
    rospy.loginfo("run node '%s as': %s", node, str(' '.join(popen_cmd)))
    # remove the 'BASH_ENV' and 'ENV' from environment
    new_env = dict(os.environ)
    try:
      for k in ['BASH_ENV', 'ENV']:
        del new_env[k]
    except:
      pass
    # add node environment parameter
    for k, v in n.env_args:
      new_env[k] = v
    # set delayed autostart parameter
    self._run_node(popen_cmd, cwd, new_env, rospy.names.ns_join(n.namespace, n.name), autostart)
    if len(cmd) > 1:
      raise StartException('Multiple executables are found! The first one was started! Exceutables:\n%s'%str(cmd))

  def _run_node(self, cmd, cwd, env, node, autostart=False):
    self._pending_starts.add(node)
    start_now = True
    start_delay = self._get_start_delay(node)
    start_required = self._get_start_required(node)
    if autostart and start_required:
      start_now = False
      # get published topics from ROS master
      master = rosgraph.masterapi.Master(self.masteruri)
      for topic, datatype in master.getPublishedTopics(''):
        if start_required == topic:
          start_now = True
          break
      if not start_now:
        # Start the timer for waiting for the topic
        start_timer = threading.Timer(3., self._run_node, args=(cmd, cwd, env, node, autostart))
        start_timer.start()
    if start_now and autostart and start_delay > 0:
      start_now = False
      # start timer for delayed start
      start_timer = threading.Timer(start_delay, self._run_node, args=(cmd, cwd, env, node, False))
      start_timer.start()
    if start_now:
      ps = subprocess.Popen(cmd, cwd=cwd, env=env)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=ps.wait)
      thread.setDaemon(True)
      thread.start()
      # remove from pending autostarts
      try:
        self._pending_starts.remove(node)
      except:
        pass
    # print the current pending autostarts
    if self._pending_starts_last_printed != self._pending_starts:
      self._pending_starts_last_printed.clear()
      self._pending_starts_last_printed.update(self._pending_starts)
      rospy.loginfo("Pending autostarts %d: %s", len(self._pending_starts), self._pending_starts)

  def _get_node(self, pkg, filename):
    cmd = None
    try:
      cmd = roslib.packages.find_node(pkg, filename)
    except roslib.packages.ROSPkgException as e:
      # multiple nodes, invalid package
      raise StartException(str(e))
    except Exception as e:
      raise StartException(str(e))
    # handle different result types str or array of string
    import types
    if isinstance(cmd, types.StringTypes):
      cmd = [cmd]
    if cmd is None or len(cmd) == 0:
      raise StartException('%s in package [%s] not found!'%(filename, pkg))
    return cmd

  def _get_start_exclude(self, node):
    param_name = rospy.names.ns_join(node, 'default_cfg/autostart/exclude')
    try:
      return bool(self.roscfg.params[param_name].value)
    except:
      pass
    return False

  def _get_start_delay(self, node):
    param_name = rospy.names.ns_join(node, 'default_cfg/autostart/delay')
    try:
      return float(self.roscfg.params[param_name].value)
    except:
      pass
    return 0.

  def _get_start_required(self, node):
    param_name = rospy.names.ns_join(node, 'default_cfg/autostart/required/publisher')
    topic = ''
    try:
      topic = self.roscfg.params[param_name].value
      if rosgraph.names.is_private(topic):
        rospy.logwarn('Private for autostart required topic `%s` is ignored!'%topic)
        topic = ''
      elif not rosgraph.names.is_global(topic):
        topic = rospy.names.ns_join(rosgraph.names.namespace(node), topic)
    except:
      pass
    return topic

  def _get_respawn_params(self, node):
    result = { 'max' : 0, 'min_runtime' : 0, 'delay': 0 }
    respawn_max = rospy.names.ns_join(node, 'respawn/max')
    respawn_min_runtime = rospy.names.ns_join(node, 'respawn/min_runtime')
    respawn_delay = rospy.names.ns_join(node, 'respawn/delay')
    try:
      result['max'] = int(self.roscfg.params[respawn_max].value)
    except:
      pass
    try:
      result['min_runtime'] = int(self.roscfg.params[respawn_min_runtime].value)
    except:
      pass
    try:
      result['delay'] = int(self.roscfg.params[respawn_delay].value)
    except:
      pass
    return result

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

  @classmethod
  def _load_parameters(cls, masteruri, params, clear_params):
    """
    Load parameters onto the parameter server
    """
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
    except Exception:
      raise #re-raise as this is fatal

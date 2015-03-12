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

import cStringIO
import threading
import xmlrpclib
import socket
import time
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SocketServer import ThreadingMixIn

import roslib; roslib.load_manifest('master_discovery_fkie')
import roslib.network
import rospy

try: # to avoid the problems with autodoc on ros.org/wiki site
  from multimaster_msgs_fkie.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo, SyncServiceInfo
  from multimaster_msgs_fkie.srv import DiscoverMasters, GetSyncInfo
except:
  pass
from common import masteruri_from_ros
from filter_interface import FilterInterface
from master_info import MasterInfo, NodeInfo, TopicInfo, ServiceInfo
import interface_finder

class MasterConnectionException(Exception):
  '''
  The exception class to handle the connection problems with ROS Master.
  '''
  pass

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise Exception("remote call failed: %s"%msg)
    return val

class RPCThreading(ThreadingMixIn, SimpleXMLRPCServer):
  pass

class RPCThreadingV6(ThreadingMixIn, SimpleXMLRPCServer):
  address_family = socket.AF_INET6
  pass

class MasterMonitor(object):
  '''
  This class provides methods to get the state from the ROS master using his 
  RPC API and test for changes. Furthermore an XML-RPC server will be created
  to offer the complete current state of the ROS master by one method call.

  :param rpcport: the port number for the XML-RPC server
  
  :type rpcport:  int
  
  :param do_retry: retry to create XML-RPC server
  
  :type do_retry: bool

  :see: :mod:`master_discovery_fkie.master_monitor.MasterMonitor.getCurrentState()`, respectively
        :mod:`master_discovery_fkie.master_monitor.MasterMonitor.updateState()`
  
  :RPC Methods:
      :mod:`master_discovery_fkie.master_monitor.MasterMonitor.getListedMasterInfo()` or 
      :mod:`master_discovery_fkie.master_monitor.MasterMonitor.getMasterContacts()` as RPC: 
      ``masterInfo()`` and ``masterContacts()``
  '''

  MAX_PING_SEC = 10.0
  ''' The time to update the node URI, ID or service URI (Default: ``10.0``)'''

  INTERVAL_UPDATE_LAUNCH_URIS = 15.0

  def __init__(self, rpcport=11611, do_retry=True, ipv6=False):
    '''
    Initialize method. Creates an XML-RPC server on given port and starts this
    in its own thread.
    
    :param rpcport: the port number for the XML-RPC server
    
    :type rpcport:  int
    
    :param do_retry: retry to create XML-RPC server
    
    :type do_retry: bool
    
    :param ipv6: Use ipv6
    
    :type ipv6: bool
    '''
    self._state_access_lock = threading.RLock()
    self._create_access_lock = threading.RLock()
    self._lock = threading.RLock()
    self.__masteruri = masteruri_from_ros()
    self.__new_master_state = None
    self.__masteruri_rpc = None
    self.__mastername = None
    self.__cached_nodes = dict()
    self.__cached_services = dict()
    self.ros_node_name = str(rospy.get_name())
    if rospy.has_param('~name'):
      self.__mastername = rospy.get_param('~name')
    self.__mastername = self.getMastername()
    rospy.set_param('/mastername', self.__mastername)

    self.__master_state = None
    '''the current state of the ROS master'''
    self.rpcport = rpcport
    '''the port number of the RPC server'''

    self._printed_errors = dict()
    self._last_clearup_ts = time.time()

    self._master_errors = list()

    # Create an XML-RPC server
    self.ready = False
    while not self.ready and (not rospy.is_shutdown()):
      try:
        RPCClass = RPCThreading
        if ipv6:
          RPCClass = RPCThreadingV6
        self.rpcServer = RPCClass(('', rpcport), logRequests=False, allow_none=True)
        rospy.loginfo("Start RPC-XML Server at %s", self.rpcServer.server_address)
        self.rpcServer.register_introspection_functions()
        self.rpcServer.register_function(self.getListedMasterInfo, 'masterInfo')
        self.rpcServer.register_function(self.getListedMasterInfoFiltered, 'masterInfoFiltered')
        self.rpcServer.register_function(self.getMasterContacts, 'masterContacts')
        self.rpcServer.register_function(self.getMasterErrors, 'masterErrors')
        self._rpcThread = threading.Thread(target = self.rpcServer.serve_forever)
        self._rpcThread.setDaemon(True)
        self._rpcThread.start()
        self.ready = True
      except socket.error as e:
        if not do_retry:
          raise Exception("Error while start RPC-XML server on port %d: %s\nIs a Node Manager already running?"%(rpcport, e))
        rospy.logwarn("Error while start RPC-XML server on port %d: %s\nTry again..."%(rpcport, e))
        time.sleep(1)
      except:
        import traceback
        print traceback.format_exc()
        if not do_retry:
          raise

    self._master = xmlrpclib.ServerProxy(self.getMasteruri())
    # === UPDATE THE LAUNCH URIS Section ===
    # subscribe to get parameter updates
    rospy.loginfo("Subscribe to parameter `/roslaunch/uris`")
    self.__mycache_param_server = rospy.impl.paramserver.get_param_server_cache()
    # HACK: use own method to get the updates also for parameters in the subgroup 
    self.__mycache_param_server.update = self.__update_param
    # first access, make call to parameter server
    self._update_launch_uris_lock = threading.RLock()
    self.__launch_uris = {}
    code, msg, value = self._master.subscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
    # the new timer will be created in self._update_launch_uris()
    self._timer_update_launch_uris = None
    if code == 1:
      for k,v in value.items():
        self.__launch_uris[roslib.names.ns_join('/roslaunch/uris', k)] = v
    self._update_launch_uris()
    # === END: UPDATE THE LAUNCH URIS Section ===

  def __update_param(self, key, value):
    # updates the /roslaunch/uris parameter list
    with self._update_launch_uris_lock:
      try:
        if value:
          self.__launch_uris[key] = value
        else:
          del self.__launch_uris[key]
      except:
        pass

  def shutdown(self):
    '''
    Shutdown the RPC Server.
    '''
    if hasattr(self, 'rpcServer'):
      if not self._master is None:
        rospy.loginfo("Unsubscribe from parameter `/roslaunch/uris`")
        try:
          self._master.unsubscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
        except Exception as e:
          rospy.logwarn("Error while unsubscribe from `/roslaunch/uris`: %s"%e)
      rospy.loginfo("shutdown own RPC server")
      self.rpcServer.shutdown()
      if not self._timer_update_launch_uris is None:
        self._timer_update_launch_uris.cancel()
      del self.rpcServer.socket
      del self.rpcServer
      rospy.loginfo("exit")

  def _update_launch_uris(self, params={}):
    with self._update_launch_uris_lock:
      if params:
        self.__launch_uris = params
      try:
        socket.setdefaulttimeout(3.0)
        for key, value in self.__launch_uris.items():
          try:
            # contact the launch server
            launch_server = xmlrpclib.ServerProxy(value)
            c, m, pid = launch_server.get_pid()
          except:
            try:
              # remove the parameter from parameter server on error
              master = xmlrpclib.ServerProxy(self.getMasteruri())
              master.deleteParam(self.ros_node_name, key)
            except:
              pass
      finally:
        socket.setdefaulttimeout(None)
        # create the new timer
        if not rospy.is_shutdown():
          self._timer_update_launch_uris = threading.Timer(self.INTERVAL_UPDATE_LAUNCH_URIS, self._update_launch_uris)
          self._timer_update_launch_uris.start()

  def _getNodePid(self, nodes):
    '''
    Gets process id of the node.
    This method blocks until the info is retrieved or socket timeout is reached (0.7 seconds).
    
    :param nodename: the name of the node
    
    :type nodename: str
    
    :param uri: the uri of the node
    
    :type uri: str
    '''
    for (nodename, uri) in nodes.items():
      if not uri is None:
        pid = None
        try:
#          print "_getNodePid _lock try..", threading.current_thread()
          with self._lock:
            if self.__cached_nodes.has_key(nodename):
              if time.time() - self.__cached_nodes[nodename][2] < self.MAX_PING_SEC:
#                print "_getNodePid _lock RETe", threading.current_thread()
                return
#          print "_getNodePid _lock RET", threading.current_thread()
          socket.setdefaulttimeout(0.7)
          node = xmlrpclib.ServerProxy(uri)
          pid = _succeed(node.getPid(self.ros_node_name))
        except (Exception, socket.error) as e:
          with self._lock:
            self._limited_log(nodename, "can't get PID: %s"%str(e))
    #      import traceback
    #      print traceback.format_exc()
          master = xmlrpclib.ServerProxy(self.getMasteruri())
    #      print "request again nodeuri for", nodename
          code, message, new_uri = master.lookupNode(self.ros_node_name, nodename)
#          print "_getNodePid _lock try..", threading.current_thread()
          with self._lock:
            self.__new_master_state.getNode(nodename).uri = None if (code == -1) else new_uri
            if code == -1:
              self._limited_log(nodename, "can't update contact information. ROS master responds with: %s"%message)
            try:
#              print "remove cached node", nodename
              del self.__cached_nodes[nodename]
            except:
              pass
#          print "_getNodePid _lock RET", threading.current_thread()
        else:
#          print "_getNodePid _lock try..", threading.current_thread()
          with self._lock:
#            print "get info about node", nodename, uri
            self.__new_master_state.getNode(nodename).pid = pid
            self.__cached_nodes[nodename] = (uri, pid, time.time())
#          print "_getNodePid _lock RET", threading.current_thread()
        finally:
          socket.setdefaulttimeout(None)

  def _getServiceInfo(self, services):
    '''
    Gets service info through the RPC interface of the service. 
    This method blocks until the info is retrieved or socket timeout is reached (0.5 seconds).
    
    :param service: the name of the service
    
    :type service: str
    
    :param uri: the uri of the service
    
    :type uri: str
    '''
    for (service, uri) in services.items():
      with self._lock:
        if self.__cached_services.has_key(service):
          if time.time() - self.__cached_services[service][2] < self.MAX_PING_SEC:
            return
      if not uri is None:
        type = dest_addr = dest_port = None
        try:
          dest_addr, dest_port = rospy.parse_rosrpc_uri(uri)
        except:
    #      print "ERROR while get service info"
          continue
#          return
    #      raise ROSServiceException("service [%s] has an invalid RPC URI [%s]"%(service, uri))
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
          # connect to service and probe it to get the headers
          s.settimeout(0.5)
          s.connect((dest_addr, dest_port))
          header = { 'probe':'1', 'md5sum':'*',
                    'callerid':self.ros_node_name, 'service':service}
          roslib.network.write_ros_handshake_header(s, header)
          stype = roslib.network.read_ros_handshake_header(s, cStringIO.StringIO(), 2048)
#          print "_getServiceInfo _lock try..", threading.current_thread()
          with self._lock:
#            print "get info about service", service, uri
            self.__new_master_state.getService(service).type = stype['type']
            self.__cached_services[service] = (uri, stype['type'], time.time())
#          print "_getServiceInfo _lock RET", threading.current_thread()
        except socket.error:
#          print "_getServiceInfo _lock try..", threading.current_thread()
          with self._lock:
            try:
#              print "delete socket service", service, uri
              del self.__cached_services[service]
            except:
              pass
    #      raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service, uri))
#          print "_getServiceInfo _lock RET", threading.current_thread()
        except:
          import traceback
          with self._lock:
            self._limited_log(service, "can't get service type: %s"%traceback.format_exc())
#          print traceback.format_exc()
#          print "_getServiceInfo _lock try..", threading.current_thread()
          with self._lock:
            try:
#              print "delete service", service, uri
              del self.__cached_services[service]
            except:
              pass
#          print "_getServiceInfo _lock RET", threading.current_thread()
          pass
        finally:
          if s is not None:
            s.close()


  def getListedMasterInfo(self):
    '''
    :return: a extended ROS Master State.
    
    :rtype:  :mod:`master_discovery_fkie.master_info.MasterInfo.listedState()` for result type 
    '''
    #'print "MASTERINFO ===================="
    t = str(time.time())
    result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [] )
    if not (self.__master_state is None):
      try:
        #'print "getListedMasterInfo _state_access_lock try...", threading.current_thread()
        with self._state_access_lock:
          #'print "  getListedMasterInfo _state_access_lock locked", threading.current_thread()
          result = self.__master_state.listedState()
        #'print "getListedMasterInfo _state_access_lock RET", threading.current_thread()
      except:
        import traceback
        print traceback.format_exc()
    #'print "MASTERINFO <<<<<<<<<<<<<<<<<<<<<"
    return result

  def getListedMasterInfoFiltered(self, filter_list):
    '''
    :return: a extended filtered ROS Master State.
    
    :rtype:  :mod:`master_discovery_fkie.master_info.MasterInfo.listedState()` for result type 
    '''
    #'print "MASTERINFO ===================="
    t = str(time.time())
    result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [] )
    if not (self.__master_state is None):
      try:
        #'print "getListedMasterInfo _state_access_lock try...", threading.current_thread()
        with self._state_access_lock:
          #'print "  getListedMasterInfo _state_access_lock locked", threading.current_thread()
#          print "FILTER_LISTE______*********", filter_list
          result = self.__master_state.listedState(FilterInterface.from_list(filter_list))
#          print result
        #'print "getListedMasterInfo _state_access_lock RET", threading.current_thread()
      except:
        import traceback
        print traceback.format_exc()
    #'print "MASTERINFO <<<<<<<<<<<<<<<<<<<<<"
    return result

  def getCurrentState(self):
    '''
    :return: The current ROS Master State
    
    :rtype: :mod:`master_discovery_fkie.master_info.MasterInfo` or ``None``
    '''
    #'print "getCurrentState _state_access_lock try...", threading.current_thread()
    with self._state_access_lock:
      #'print "  getCurrentState _state_access_lock locked", threading.current_thread()
      return self.__master_state
    #'print "getCurrentState _state_access_lock RET", threading.current_thread()

  def updateState(self, clear_cache=False):
    '''
    Gets state from the ROS Master through his RPC interface.
    
    :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                        If remote hosted nodes or services was restarted, the cache must 
                        be cleared! The local nodes will be updated periodically after
                        :mod:`master_discovery_fkie.master_monitor.MasterMonitor.MAX_PING_SEC`.
    
    :type clear_cache: bool (Default: ``False``) 
    
    :rtype: :mod:`master_discovery_fkie.master_info.MasterInfo`
    
    :raise: ``MasterConnectionException``, if not complete information was get from the ROS master.
    '''
    #'print "updateState _create_access_lock try...", threading.current_thread()
    with self._create_access_lock:
      #'print "  updateState _create_access_lock locked", threading.current_thread()
      now = time.time()
      threads = []
      try:
#        import os                                ###################
#        cputimes = os.times()                    ###################
#        cputime_init = cputimes[0] + cputimes[1] ###################
        #'print "  updateState _lock try...", threading.current_thread()
        self._lock.acquire(True)
        #'print "    updateState _lock locked", threading.current_thread()
        if clear_cache:
          self.__cached_nodes = dict()
          self.__cached_services = dict()
        socket.setdefaulttimeout(5)
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################
        self.__new_master_state = master_state = MasterInfo(self.getMasteruri(), self.getMastername())
        # update master state
        master = self._master
        #master = xmlrpclib.ServerProxy(self.getMasteruri())
        # get topic types
        code, message, topicTypes = master.getTopicTypes(self.ros_node_name)
#        cputimes2 = os.times()                                            ###################
#        print "Get types: ", (cputimes2[0] + cputimes2[1] - cputime_init2) ###################
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################
        #convert topicType list to the dict
        topicTypesDict = {}
        for topic, type in topicTypes:
          topicTypesDict[topic] = type
#        cputimes2 = os.times()                                            ###################
#        print "Set types: ", (cputimes2[0] + cputimes2[1] - cputime_init2) ###################
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################
        # get system state
        code, message, state = master.getSystemState(self.ros_node_name)
#        cputimes2 = os.times()                                            ###################
#        print "Get state: ", (cputimes2[0] + cputimes2[1] - cputime_init2) ###################
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################

        # add published topics
        for t, l in state[0]:
          master_state.topics = t
          for n in l:
            master_state.nodes = n
            master_state.getNode(n).publishedTopics = t
            master_state.getTopic(t).publisherNodes = n
            master_state.getTopic(t).type = topicTypesDict.get(t, 'None')
#        cputimes2 = os.times()                                            ###################
#        print "read pub topics: ", (cputimes2[0] + cputimes2[1] - cputime_init2) ###################
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################
        # add subscribed topics
        for t, l in state[1]:
          master_state.topics = t
          for n in l:
            master_state.nodes = n
            master_state.getNode(n).subscribedTopics = t
            master_state.getTopic(t).subscriberNodes = n
            master_state.getTopic(t).type = topicTypesDict.get(t, 'None')
#        cputimes2 = os.times()                                            ###################
#        print "read sub topics: ", (cputimes2[0] + cputimes2[1] - cputime_init2) ###################
#        cputimes2 = os.times()                    ###################
#        cputime_init2 = cputimes2[0] + cputimes2[1] ###################
#        cputimes = os.times()                                            ###################
#        print "Auswertung: ", (cputimes[0] + cputimes[1] - cputime_init) ###################
  
        # add services
#        cputimes = os.times()                    ###################
#        cputime_init = cputimes[0] + cputimes[1] ###################

        services = dict()
        tmp_slist = []
        # multi-call style xmlrpc to lock up the service uri
        param_server_multi = xmlrpclib.MultiCall(master)
        for t, l in state[2]:
          master_state.services = t
          for n in l:
            master_state.nodes = n
            master_state.getNode(n).services = t
            service = master_state.getService(t)
            service.serviceProvider = n
            if self.__cached_services.has_key(service.name):
              service.uri = self.__cached_services[service.name][0]
              service.type = self.__cached_services[service.name][1]
              if service.isLocal and time.time() - self.__cached_services[service.name][2] > self.MAX_PING_SEC:
                services[service.name] = service.uri
            else:
              #'print "request service:", service.name
              tmp_slist.append(service)
              param_server_multi.lookupService(self.ros_node_name, t)
  #          code, message, service.uri = master.lookupService(rospy.get_name(), t)
  #          if (code == -1):
  #            service.uri = None
  #          elif service.isLocal:
  #            services[service.name] = service.uri
        try:
          r = param_server_multi()
          for (code, msg, uri), service in zip(r, tmp_slist):
            if code == 1:
              service.uri = uri
              if service.isLocal:
#                print "service local: ", service.uri, service.masteruri
                services[service.name] = uri
              else:
                self.__cached_services[service.name] = (uri, None, time.time())
            else:
              self._limited_log(service.name, "can't get contact information. ROS master responds with: %s"%msg)
        except:
          import traceback
          traceback.print_exc()
        if services:
          pidThread = threading.Thread(target = self._getServiceInfo, args=((services,)))
          pidThread.start()
          threads.append(pidThread)

        #get additional node information
        nodes = dict()
        try:
          # multi-call style xmlrpc to loock up the node uri
          param_server_multi = xmlrpclib.MultiCall(master)
          tmp_nlist = []
          for name, node in master_state.nodes.items():
            if self.__cached_nodes.has_key(node.name):
              node.uri = self.__cached_nodes[node.name][0]
              node.pid = self.__cached_nodes[node.name][1]
              if node.isLocal and time.time() - self.__cached_nodes[node.name][2] > self.MAX_PING_SEC:
                nodes[node.name] = node.uri
            else:
              #'print "request node:", node.name
              tmp_nlist.append(node)
              param_server_multi.lookupNode(self.ros_node_name, name)
          r = param_server_multi()
          for (code, msg, uri), node in zip(r, tmp_nlist):
            if code == 1:
              node.uri = uri
              if node.isLocal:
                nodes[node.name] = uri
              else:
                self.__cached_nodes[node.name] = (uri, None, time.time())
            else:
              self._limited_log(node.name, "can't get contact information. ROS master responds with: %s"%msg)
        except:
          import traceback
          traceback.print_exc()
#        cputimes = os.times() ###################
#        print "Nodes+Services:", (cputimes[0] + cputimes[1] - cputime_init), ", count nodes:", len(nodes) ###################

        if nodes:
          # get process id of the nodes
          pidThread = threading.Thread(target = self._getNodePid, args=((nodes,)))
          pidThread.start()
          threads.append(pidThread)

        master_state.timestamp = now
      except socket.error, e:
        if isinstance(e, tuple):
          (errn, msg) = e
          if not errn in [100, 101, 102]:
            import traceback
            formatted_lines = traceback.format_exc().splitlines()
      #      print "Service call failed: %s"%traceback.format_exc()
            raise MasterConnectionException(formatted_lines[-1])
        else:
          import traceback
          raise MasterConnectionException(traceback.format_exc())
      except:
        import traceback
        formatted_lines = traceback.format_exc().splitlines()
  #      print "Service call failed: %s"%traceback.format_exc()
        raise MasterConnectionException(formatted_lines[-1])
      finally:
        self._lock.release()
        #'print "  updateState _lock RET", threading.current_thread()
        socket.setdefaulttimeout(None)

#      cputimes = os.times()                    ###################
#      cputime_init = cputimes[0] + cputimes[1] ###################
      
#      print "threads:", len(threads)
      # wait for all threads are finished 
      while threads:
        th = threads.pop()
        if th.isAlive():
  #        print "join"
          th.join()
  #        print "release"
        del th
      if time.time() - self._last_clearup_ts > 300:
        self._last_clearup_ts = time.time()
        self._clearup_cached_logs()
#      cputimes = os.times() ###################
#      print "Threads:", (cputimes[0] + cputimes[1] - cputime_init) ###################
  #    print "state update of ros master", self.__masteruri, " finished"
#      return MasterInfo.from_list(master_state.listedState())
      #'print "updateState _create_access_lock RET", threading.current_thread()

      return master_state

  def _limited_log(self, provider, msg):
    if not provider in self._printed_errors:
      self._printed_errors[provider] = dict()
    if not msg in self._printed_errors[provider]:
      self._printed_errors[provider][msg] = time.time()
      rospy.logwarn("MasterMonitor[%s]: %s"%(provider, msg))

  def _clearup_cached_logs(self, age=300):
    cts = time.time()
    for p, msgs in self._printed_errors.items():
      for msg, ts in msgs.items():
        if cts - ts > age:
          del self._printed_errors[p][msg]
      if not self._printed_errors[p]:
        del self._printed_errors[p]

  def updateSyncInfo(self):
    '''
    This method can be called to update the origin ROS master URI of the nodes
    and services in new ``master_state``. This is only need, if a synchronization is 
    running. The synchronization service will be detect automatically by searching
    for the service ending with ``get_sync_info``. The method will be called by 
    :mod:`master_discovery_fkie.master_monitor.MasterMonitor.checkState()`.
    '''
    #'print "updateSyncInfo _create_access_lock try...", threading.current_thread()

    def getNodeuri(nodename, publisher, subscriber, services):
      for p in publisher:
        if nodename == p.node:
          return p.nodeuri
      for p in subscriber:
        if nodename == p.node:
          return p.nodeuri
      for s in services:
        if nodename == s.node:
          return s.nodeuri
      return None

    with self._create_access_lock:
      #'print "  updateSyncInfo _create_access_lock locked", threading.current_thread()
      master_state = self.__new_master_state
      sync_info = None
      # get synchronization info, if sync node is running
      # to determine the origin ROS MASTER URI of the nodes
      for name, service in master_state.services.items():
        if service.name.endswith('get_sync_info'):
          if interface_finder.hostFromUri(self.getMasteruri()) == interface_finder.hostFromUri(service.uri):
            socket.setdefaulttimeout(3)
            get_sync_info = rospy.ServiceProxy(service.name, GetSyncInfo)
            try:
              sync_info = get_sync_info()
            except rospy.ServiceException, e:
              rospy.logwarn("ERROR Service call 'get_sync_info' failed: %s", str(e))
            finally:
              socket.setdefaulttimeout(None)

      #update the origin ROS MASTER URI of the nodes, if sync node is running
      if sync_info:
        for m in sync_info.hosts:
          for n in m.nodes:
            try:
              # TODO: add nodeuri to the nodes (needs changes in the MSG definitions)
              # set the sync node only if it has the same uri
              nuri = getNodeuri(n, m.publisher, m.subscriber, m.services)
              state_node = master_state.getNode(n)
              if not state_node is None and (state_node.uri == nuri or nuri is None):
                state_node.masteruri = m.masteruri
            except:
              pass
#              import traceback
#              print traceback.format_exc()
          for s in m.services:
            try:
              state_service = master_state.getService(s.service)
              if not state_service is None and state_service.uri == s.serviceuri:
                state_service.masteruri = m.masteruri
            except:
              pass
      #'print "updateSyncInfo _create_access_lock RET", threading.current_thread()


  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it.
    
    :return: ROS master URI
    
    :rtype: str or ``None``
    '''
    code = -1
    if self.__masteruri_rpc is None:
      master = xmlrpclib.ServerProxy(self.__masteruri)
      code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
    return self.__masteruri_rpc if code >= 0 or not self.__masteruri_rpc is None else self.__masteruri

  def getMastername(self):
    '''
    Returns the name of the master. If no name is set, the hostname of the 
    ROS master URI will be extracted.
    
    :return: the name of the ROS master
    
    :rtype: str or ``None``
    '''
    if self.__mastername is None:
      try:
        self.__mastername = interface_finder.hostFromUri(self.getMasteruri())
        try:
          from urlparse import urlparse
          master_port = urlparse(self.__masteruri).port
          if master_port != 11311:
            self.__mastername = '--'.join([self.__mastername, str(master_port)])
        except:
          pass
      except:
        pass
    return self.__mastername
  
  def getMasterContacts(self):
    '''
    The RPC method called by XML-RPC server to request the master contact information.
    
    :return: (``timestamp of the ROS master state``, ``ROS master URI``, ``master name``, ``name of this service``, ``URI of this RPC server``)
    :rtype: (str, str, str, str, str)
    '''
    #'print "MASTERCONTACTS ===================="
    t = 0
    if not self.__master_state is None:
      #'print "getMasterContacts _state_access_lock try...", threading.current_thread()
      with self._state_access_lock:
        #'print "  getMasterContacts _state_access_lock locked", threading.current_thread()
        t = self.__master_state.timestamp
      #'print "getMasterContacts _state_access_lock RET", threading.current_thread()
    #'print "MASTERCONTACTS <<<<<<<<<<<<<<<<<<<<"
    return (str(t), str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, roslib.network.create_local_xmlrpc_uri(self.rpcport))

  def getMasterErrors(self):
    '''
    The RPC method called by XML-RPC server to request the occured network errors.

    :return: (``ROS master URI``, ``list with errors``)
    :rtype: (str, str, [str])
    '''
    return (str(self.getMasteruri()), self._master_errors)

  def checkState(self, clear_cache=False):
    '''
    Gets the state from the ROS master and compares it to the stored state.

    :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                        If remote hosted nodes or services was restarted, the cache must 
                        be cleared! The local nodes will be updated periodically after
                        :mod:`master_discovery_fkie.master_monitor.MasterMonitor.MAX_PING_SEC`.
    
    :type clear_cache: bool (Default: ``False``) 

    :return: ``True`` if the ROS master state is changed
    
    :rtype: bool
    '''
    result = False
    s = self.updateState(clear_cache)
    #'print "checkState _create_access_lock try...", threading.current_thread()
    with self._create_access_lock:
      #'print "  checkState _create_access_lock locked", threading.current_thread()
      #'print "  checkState _state_access_lock try...", threading.current_thread()
      do_update = False
      with self._state_access_lock:
        #'print "    checkState _state_access_lock locked", threading.current_thread()
        if s != self.__master_state:
          do_update = True
      if do_update:
        self.updateSyncInfo()
        with self._state_access_lock:
          # test for local changes
          ts_local = self.__new_master_state.timestamp_local
          if not self.__master_state is None and not self.__master_state.has_local_changes(s):
            ts_local = self.__master_state.timestamp_local
          self.__master_state = self.__new_master_state
          self.__master_state.timestamp_local = ts_local
          result = True
      self.__master_state.check_ts = self.__new_master_state.timestamp
      #'print "  checkState _state_access_lock RET", threading.current_thread()
      #'print "checkState _create_access_lock RET", threading.current_thread()
      return result

  def reset(self):
    '''
    Sets the master state to ``None``. 
    '''
    #'print "reset _state_access_lock try...", threading.current_thread()
    with self._state_access_lock:
      #'print "  reset _state_access_lock locked", threading.current_thread()
      if not self.__master_state is None:
        del self.__master_state
      self.__master_state = None
      #'print "reset _state_access_lock RET", threading.current_thread()

  def update_master_errors(self, error_list):
    self._master_errors = list(error_list)

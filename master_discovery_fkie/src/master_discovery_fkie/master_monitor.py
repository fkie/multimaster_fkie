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
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from SocketServer import ThreadingMixIn

import roslib; roslib.load_manifest('master_discovery_fkie')
import roslib.network
import rospy
import rosnode

from master_discovery_fkie.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo
from master_discovery_fkie.srv import DiscoverMasters, GetSyncInfo
from master_info import MasterInfo, NodeInfo, TopicInfo, ServiceInfo
import interface_finder

class MasterConnectionException(Exception):
  '''
  The default exception class.
  '''
  pass

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise rosnode.ROSNodeException("remote call failed: %s"%msg)
    return val

class RPCThreading(ThreadingMixIn, SimpleXMLRPCServer):
  pass

class MasterMonitor(object):
  '''
  This class provides methods to get the state from the ROS master using his 
  RPC API and test for changes. Furthermore an XML-RPC server will be created
  to offer the complete current state of the ROS master by one method call.
  @see: L{getState()}
  RPC Methods:
  @see: L{getListedMasterInfo()} or L{getMasterContacts()} as RPC: C{masterInfo()} and 
  C{masterContacts()}
  @group RPC-methods: getListedMasterInfo, getMasterContacts
  '''

  MAX_PING_SEC = 10.0

  def __init__(self, rpcport=11611):
    '''
    Initialize method. Creates an XML-RPC server on given port and starts this
    in its own thread.
    @param rpcport: the port number for the XML-RPC server
    @type rpcport:  C{int}
    '''
    self._state_access_lock = threading.RLock()
    self._create_access_lock = threading.RLock()
    self._lock = threading.RLock()
    self.__masteruri = self._masteruri_from_ros()
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
    '''@ivar: the current state of the ROS master'''
    self.rpcport = rpcport
    '''@ivar: the port number of the RPC server'''
    
    # Create an XML-RPC server
    ready = False
    while not ready and (not rospy.is_shutdown()):
      try:
        self.rpcServer = RPCThreading(('', rpcport), logRequests=False, allow_none=True)
        rospy.loginfo("Start RPC-XML Server at %s", self.rpcServer.server_address)
        self.rpcServer.register_introspection_functions()
        self.rpcServer.register_function(self.getListedMasterInfo, 'masterInfo')
        self.rpcServer.register_function(self.getMasterContacts, 'masterContacts')
        self._rpcThread = threading.Thread(target = self.rpcServer.serve_forever)
        self._rpcThread.setDaemon(True)
        self._rpcThread.start()
        ready = True
      except socket.error:
        rospy.logwarn(''.join(["Error while start RPC-XML server on port ", str(rpcport), ". Try again..."]))
        time.sleep(1)
      except:
        import traceback
        print traceback.format_exc()

  @classmethod
  def _masteruri_from_ros(cls):
    '''
    Returns the master URI depending on ROS distribution API.
    @return: ROS master URI
    @rtype C{str}
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
      import os
      return os.environ['ROS_MASTER_URI']

  def shutdown(self):
    '''
    Shutdown the RPC Server.
    '''
    if hasattr(self, 'rpcServer'):
      self.rpcServer.shutdown()

  def _getNodePid(self, nodes):
    '''
    Gets process id of the node.
    @param nodename: the name of the node
    @type nodename: C{str}
    @param uri: the uri of the node
    @type uri: C{str}
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
        except (Exception, socket.error):
    #      import traceback
    #      print traceback.format_exc()
          master = xmlrpclib.ServerProxy(self.getMasteruri())
    #      print "request again nodeuri for", nodename
          code, message, new_uri = master.lookupNode(self.ros_node_name, nodename)
#          print "_getNodePid _lock try..", threading.current_thread()
          with self._lock:
            self.__new_master_state.getNode(nodename).uri = None if (code == -1) else new_uri
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
    @param service: the name of the service
    @type service: C{str}
    @param uri: the uri of the service
    @type uri: C{str}
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
          type = roslib.network.read_ros_handshake_header(s, cStringIO.StringIO(), 2048)
#          print "_getServiceInfo _lock try..", threading.current_thread()
          with self._lock:
#            print "get info about service", service, uri
            self.__new_master_state.getService(service).type = type['type']
            self.__cached_services[service] = (uri, type['type'], time.time())
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
#          import traceback
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
    Returns a extended roscore state. 
    @return: complete roscore state as
             
             C{(stamp, stamp_local, masteruri, name, publishers, subscribers, services, topicTypes, nodes, serviceProvider)}
             
               - C{publishers} is of the form
                 
                 C{[ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]}
               
               - C{subscribers} is of the form
                 
                 C{[ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]}
               
               - C{services} is of the form
                 
                 C{[ [service1, [service1Provider1...service1ProviderN]] ... ]}
               
               - C{topicTypes} is a list of 
                 
                 C{[[topicName1, topicType1], ... ]}
               
               - C{nodes} is a list of (the pid of remote Nodes will not be resolved)
                 
                 C{[nodename, XML-RPC URI, pid, E{lb} local, remote E{rb}]}
               
               - C{serviceProvider} is a list of (the type, serviceClass and args of remote Services will not be resolved)
                 
                 C{[service, XML-RPC URI, type, E{lb} local, remote E{rb}]}
               
    @rtype: C{(float,
               float,
               str,
               str,
               [ [str,[str] ] ], 
               [ [str,[str] ] ], 
               [ [str,[str] ] ], 
               [ [str,str] ], 
               [ [str,str,int,str] ], 
               [ [str,str,str,str] ])}
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

  def getCurrentState(self):
    #'print "getCurrentState _state_access_lock try...", threading.current_thread()
    with self._state_access_lock:
      #'print "  getCurrentState _state_access_lock locked", threading.current_thread()
      return self.__master_state
    #'print "getCurrentState _state_access_lock RET", threading.current_thread()
    
  def updateState(self, clear_cache=False):
    '''
    Gets state from the ROS master through his RPC interface.
    @rtype: L{MasterInfo}
    @raise MasterConnectionException: if not complete information was get from the ROS master.
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
        master = xmlrpclib.ServerProxy(self.getMasteruri())
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
#      cputimes = os.times() ###################
#      print "Threads:", (cputimes[0] + cputimes[1] - cputime_init) ###################
  #    print "state update of ros master", self.__masteruri, " finished"
#      return MasterInfo.from_list(master_state.listedState())
      #'print "updateState _create_access_lock RET", threading.current_thread()

      return master_state
  
  def updateSyncInfo(self):
    '''
    This method can be called to update the origin ROS master URI of the nodes
    and services in new master_state. This is only need, if a synchronization is 
    running. The synchronization service will be detect automatically by searching
    for the service ending with C{get_sync_info}. The method will be called by 
    C{checkState()}.
    '''
    #'print "updateSyncInfo _create_access_lock try...", threading.current_thread()

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
              master_state.getNode(n).masteruri = m.masteruri
            except:
              pass
          for s in m.services:
            try:
              master_state.getService(s).masteruri = m.masteruri
            except:
              pass
      #'print "updateSyncInfo _create_access_lock RET", threading.current_thread()


    
  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it.
    @return: ROS master URI
    @rtype: C{str} or C{None}
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
    @return: the name of the ROS master
    @rtype: C{str} or C{None}
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
    @return: (timestamp of the ROS master state, ROS master URI, master name, name of this service, URI of this RPC server)
    @rtype: C{(str, str, str, str, str)}
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
  
  def checkState(self, clear_cache=False):
    '''
    Gets the state from the ROS master and compares it to the stored state.
    @param clear_cache Clears cache for ROS nodes and sevices. Is neaded on changes on remote hosts. 
    @return: True if the ROS master state is changed
    @rtype: C{boolean}
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
    Sets the master state to None. 
    '''
    #'print "reset _state_access_lock try...", threading.current_thread()
    with self._state_access_lock:
      #'print "  reset _state_access_lock locked", threading.current_thread()
      if not self.__master_state is None:
        del self.__master_state
      self.__master_state = None
      #'print "reset _state_access_lock RET", threading.current_thread()

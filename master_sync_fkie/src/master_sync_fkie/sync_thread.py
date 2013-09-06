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


import time
import threading
import xmlrpclib
import random
import socket

import roslib; roslib.load_manifest('master_sync_fkie')
import roslib.message
import rospy
import rosgraph.masterapi

from master_discovery_fkie.common import masteruri_from_ros
from master_discovery_fkie.filter_interface import FilterInterface
from multimaster_msgs_fkie.msg import SyncTopicInfo, SyncMasterInfo

class MasterInfo(object):
  '''
  The Master representation stored in the L{SyncThread}. 
  '''
  def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, lastsync=0.0):
    self.name = name
    self.uri = uri
    self.timestamp = timestamp
    self.discoverer_name = discoverer_name
    self.monitoruri = monitoruri
    self.lastsync = lastsync
    self.syncts = 0.0
  
  def __repr__(self):
    """
      Produce a string representation of the master item.
    """
    return ''.join(["Master [ ", self.name, "]"
                    "\n  uri: ", self.uri,
                    "\n  timestamp: ", self.timestamp,
                    "\n  syncts: ", self.syncts])



class SyncThread(threading.Thread):
  '''
  A thread to synchronize the local ROS master with a remote master. While the 
  synchronization only the topic of the remote ROS master will be registered by
  the local ROS master. The remote ROS master will be keep unchanged.
  '''
  
  def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, sync_on_demand=False):
    '''
    Initialization method for the SyncThread. 
    @param name: the name of the ROS master synchronized with.
    @type name:  C{str}
    @param uri: the URI of the ROS master synchronized with
    @type uri:  C{str}
    @param discoverer_name: the name of the discovery node running on ROS master synchronized with.
    @type discoverer_name:  C{str}
    @param monitoruri: The URI of RPC server of the discovery node to get the ROS master state by calling a method only once.
    @type monitoruri:  C{str}
    @param timestamp: The timestamp of the current state of the ROS master info.
    @type timestamp:  C{float64}
    @param sync_on_demand: Synchronize topics on demand
    @type sync_on_demand: bool
    '''
    # init thread
    threading.Thread.__init__(self)
    self.masterInfo = MasterInfo(name, uri, discoverer_name, monitoruri, timestamp)
    self.localMasteruri = masteruri_from_ros()
    rospy.logdebug("SyncThread[%s]: create this sync thread", self.masterInfo.name)
    # synchronization variables 
    self.__cv = threading.Condition()
    self.__lock_info = threading.RLock()
    self.__sync_info = None #SyncMasterInfo with currently synchronized nodes, publisher (topic, node, nodeuri), subscriber(topic, node, nodeuri) and services
    self.__stop = False
    # a list with published topics as a tuple of (topic name, node name, node URL)
    self.__publisher = []
    # a list with subscribed topics as a tuple of (topic name, node name, node URL)
    self.__subscriber = []
    # a list with services as a tuple of (service name, service URL, node name, node URL)
    self.__services = []
    self.__own_state = None
    
    #setup the filter
    self._filter = FilterInterface()
    self._filter.load(self.masterInfo.name,
                      ['/rosout', rospy.get_name().replace('/', '/*')+'*', self.masterInfo.discoverer_name.replace('/', '/*')+'*', '/*node_manager', '/*zeroconf'], [],
                      ['/rosout', '/rosout_agg'], ['/'] if sync_on_demand else [],
                      ['/*get_loggers', '/*set_logger_level'], [],
                      # do not sync the bond message of the nodelets!!
                      ['bond/Status'])

    # congestion avoidance: wait for minimum 1 sec. If an update request is received wait 
    # for next 1 second. Force update after maximum 5 sec since first update request.
    self._ts_first_update_request = None
    self._ts_last_update_request = None
    self._use_filtered_method = None

    self.start()

  def getSyncInfo(self):
    with self.__lock_info:
      if self.__sync_info is None:
        # create a sync info
        result_set = set()
        result_publisher = []
        result_subscriber = []
        result_service_set = set()
        for (t_n, n_n, n_uri) in self.__publisher:
          result_publisher.append(SyncTopicInfo(t_n, n_n, n_uri))
          result_set.add(n_n)
        for (t_n, n_n, n_uri) in self.__subscriber:
          result_subscriber.append(SyncTopicInfo(t_n, n_n, n_uri))
          result_set.add(n_n)
        for (s_n, s_uri, n_n, n_uri) in self.__services:
          result_service_set.add(s_n)
          result_set.add(n_n)
        self.__sync_info = SyncMasterInfo(self.masterInfo.uri, list(result_set), result_publisher, result_subscriber, list(result_service_set)) 
      return self.__sync_info

  def update(self, name, uri, discoverer_name, monitoruri, timestamp):
    '''
    Sets a request to synchronize the local ROS master with this ROS master. 
    @note: If currently a synchronization is running this request will be ignored!
    @param name: the name of the ROS master synchronized with.
    @type name:  C{str}
    @param uri: the URI of the ROS master synchronized with
    @type uri:  C{str}
    @param discoverer_name: the name of the discovery node running on ROS master synchronized with.
    @type discoverer_name:  C{str}
    @param monitoruri: The URI of RPC server of the discovery node to get the ROS master state by calling a method only once.
    @type monitoruri:  C{str}
    @param timestamp: The timestamp of the current state of the ROS master info.
    @type timestamp:  C{float64}
    '''
#    rospy.logdebug("SyncThread[%s]: update request", self.masterInfo.name)
    with self.__cv:
      str_time = str(timestamp)
      if (str(self.masterInfo.timestamp_local) != str_time):
        rospy.logdebug("SyncThread[%s]: update notify new timestamp(%s), old(%s)", self.masterInfo.name, str_time, str(self.masterInfo.timestamp_local))
        self.masterInfo.name = name
        self.masterInfo.uri = uri
        self.masterInfo.discoverer_name = discoverer_name
        self.masterInfo.monitoruri = monitoruri
        self.masterInfo.syncts = 0.0
        # for congestion avoidance 
        self._ts_last_update_request = time.time()
        if self._ts_first_update_request is None:
          self._ts_first_update_request = time.time()
        self.__cv.notify()
#    rospy.logdebug("SyncThread[%s]: update exit", self.masterInfo.name)

  def setOwnMasterState(self, own_state):
    '''
    Sets the state of the local ROS master state. If this state is not None, the topics on demand will be synchronized. 
    @param own_state: the state of the local ROS master state
    @type own_state:  C{master_discovery_fkie/MasterInfo}
    '''
#    rospy.logdebug("SyncThread[%s]: setOwnMasterState", self.masterInfo.name)
    with self.__cv:
      timestamp_local = own_state.timestamp_local
      if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
        rospy.logdebug("SyncThread[%s]: local state update notify new timestamp(%s), old(%s)", self.masterInfo.name, str(timestamp_local), str(self.__own_state.timestamp_local if not self.__own_state is None else 'None'))
        self.__own_state = own_state
        self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
        self.masterInfo.syncts = 0.0
        # for congestion avoidance 
        self._ts_last_update_request = time.time()
        if self._ts_first_update_request is None:
          self._ts_first_update_request = time.time()
        self.__cv.notify()
#    rospy.logdebug("SyncThread[%s]: setOwnMasterState exit", self.masterInfo.name)

  def stop(self):
    '''
    Stops running thread.
    '''
    rospy.logdebug("SyncThread[%s]: stop request", self.masterInfo.name)
    with self.__cv:
      self.__stop = True
      self.__cv.notify()
    rospy.logdebug("SyncThread[%s]: stop exit", self.masterInfo.name)

  def run(self):
    '''
    The synchronization procedure waits for notifications of the L{update()} method.
    If the remote ROS master is changed, the changes will be performed on the 
    local ROS master.
    '''
    do_wait = False
    while not self.__stop and not rospy.is_shutdown():
      if do_wait:
        time.sleep(3)
      with self.__cv:
        ''' wait for new sync update '''
        rospy.logdebug("SyncThread[%s]: run waiting timestamp(%s), syncts(%s)", self.masterInfo.name, str(self.masterInfo.timestamp), str(self.masterInfo.syncts))
        if not (self.masterInfo.syncts == 0.0) and self.masterInfo.lastsync != 0.0:
          self.__cv.wait()
        rospy.logdebug("SyncThread[%s]: run notify received", self.masterInfo.name)
        if (not self.__stop):
          rospy.logdebug("SyncThread[%s]: congestion avoidance...", self.masterInfo.name)
          # congestion avoidance
          if not self._ts_first_update_request is None:
            current_time = time.time()
            while ((current_time - self._ts_first_update_request) < 5 and
                   (current_time - self._ts_last_update_request) < 1.1):
              wait = random.random() * 2
              print "sleep", self.masterInfo.name, wait
              time.sleep(wait)
              current_time = time.time()
          rospy.logdebug("SyncThread[%s]: run sync", self.masterInfo.name)
          ''' try to sync ''' 
          try:
            #connect to master_monitor rpc-xml server
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpclib.ServerProxy(self.masterInfo.monitoruri)
            if self._use_filtered_method is None:
              try:
                self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
              except:
                self._use_filtered_method = False
            remote_state = None
            if self._use_filtered_method:
              remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
              remote_state = remote_monitor.masterInfo()
            stamp = float(remote_state[0])
            stamp_local = float(remote_state[1])
            remote_masteruri = remote_state[2]
#            remote_mastername = remote_state[3]
            publishers = remote_state[4]
            subscribers = remote_state[5]
            rservices = remote_state[6]
            topicTypes = remote_state[7]
            nodeProviders = remote_state[8]
            serviceProviders = remote_state[9]

            own_master = xmlrpclib.ServerProxy(self.localMasteruri)
            own_master_multi = xmlrpclib.MultiCall(own_master)
            
            handler = []
            # sync the publishers
            publisher = []
            publisher_to_register = []
            for (topic, nodes) in publishers:
              for node in nodes:
                topictype = self._getTopicType(topic, topicTypes)
                nodeuri = self._getNodeUri(node, nodeProviders, remote_masteruri)
                if topictype and nodeuri and not self._doIgnoreNT(node, topic, topictype):
                  # register the nodes only once
                  if not ((topic, node, nodeuri) in self.__publisher):
                    publisher_to_register.append((topic, topictype, node, nodeuri))
                  publisher.append((topic, node, nodeuri))
            # unregister not updated subscriber
            for (topic, node, nodeuri) in set(self.__publisher)-set(publisher):
              own_master_multi.unregisterPublisher(node, topic, nodeuri)
              handler.append(('upub', topic, node, nodeuri))
            #register new subscriber
            for (topic, topictype, node, nodeuri) in publisher_to_register:
              own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
              handler.append(('pub', topic, topictype, node, nodeuri))
    
            # sync the subscribers
            subscriber = []
            subscriber_to_register = []
            for (topic, nodes) in subscribers:
              for node in nodes:
                topictype = self._getTopicType(topic, topicTypes)
                nodeuri = self._getNodeUri(node, nodeProviders, remote_masteruri)
                if topictype and nodeuri and not self._doIgnoreNT(node, topic, topictype):
                  # register the node as subscriber in local ROS master
                  if not ((topic, node, nodeuri) in self.__subscriber):
                    subscriber_to_register.append((topic, topictype, node, nodeuri))
                  subscriber.append((topic, node, nodeuri))
            # unregister not updated topics
            for (topic, node, nodeuri) in set(self.__subscriber)-set(subscriber):
              own_master_multi.unregisterSubscriber(node, topic, nodeuri)
              handler.append(('usub', topic, node, nodeuri))
            #register new subscriber
            for (topic, topictype, node, nodeuri) in subscriber_to_register:
              own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
              handler.append(('sub', topic, topictype, node, nodeuri))
            
            # sync the services
            services = []
            services_to_register = []
            for (service, nodes) in rservices:
              for node in nodes:
                serviceuri = self._getServiceUri(service, serviceProviders, remote_masteruri)
                nodeuri = self._getNodeUri(node, nodeProviders, remote_masteruri)
                if serviceuri and nodeuri and not self._doIgnoreNS(node, service):
                  # register the node as publisher in local ROS master
                  if not ((service, serviceuri, node, nodeuri) in self.__services):
                    services_to_register.append((service, serviceuri, node, nodeuri))
                  services.append((service, serviceuri, node, nodeuri))
            # unregister not updated services
            for (service, serviceuri, node, nodeuri) in set(self.__services)-set(services):
              own_master_multi.unregisterService(node, service, serviceuri)
              handler.append(('usrv', service, serviceuri, node, nodeuri))
            #register new services
            for (service, serviceuri, node, nodeuri) in services_to_register:
              own_master_multi.registerService(node, service, serviceuri, nodeuri)
              handler.append(('srv', service, serviceuri, node, nodeuri))

            # exceute the multicall and update the current publicher, subscriber and services
            with self.__lock_info:
              self.__sync_info = None
              socket.setdefaulttimeout(3)
              result = own_master_multi()
              # Horrible hack: the response from registerSubscriber() can contain a
              # list of current publishers.  But we don't have a way of injecting them
              # into rospy here.  Now, if we get a publisherUpdate() from the master,
              # everything will work.  So, we ask the master if anyone is currently
              # publishing the topic, grab the advertised type, use it to advertise
              # ourselves, then unadvertise, triggering a publisherUpdate() along the
              # way.

              # We create publisher locally as a hack, to get callback set up properly for already registered local publishers
              for h,(code, statusMessage, r) in zip(handler, result):
                try:
                  if h[0] == 'sub' and code == 1 and len(r) > 0:
                    topicPub = rospy.Publisher(h[1], roslib.message.get_message_class(h[2]))
                    topicPub.unregister()
                    del topicPub
                  if h[0] == 'sub':
                    rospy.loginfo("SyncThread[%s] topic subscribed: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                  elif h[0] == 'pub':
                    rospy.loginfo("SyncThread[%s] topic advertised: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                  elif h[0] == 'usub':
                    rospy.loginfo("SyncThread[%s] topic unsubscribed: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                  elif h[0] == 'upub':
                    rospy.loginfo("SyncThread[%s] topic unadvertised: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                  elif h[0] == 'srv':
                    rospy.loginfo("SyncThread[%s] service registered: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                  elif h[0] == 'usrv':
                    rospy.loginfo("SyncThread[%s] service unregistered: %s, %s %s", self.masterInfo.name, h[1], str(code), str(statusMessage))
                except:
                  import traceback
                  rospy.logwarn("SyncThread[%s] ERROR while hack subscriber[%s]: %s", self.masterInfo.name, h[1], traceback.format_exc())
              self.__publisher = publisher
              self.__subscriber = subscriber
              self.__services = services
                
              # set the last synchronization time
              self.masterInfo.timestamp = stamp
              self.masterInfo.timestamp_local = stamp_local
              self.masterInfo.lastsync = stamp
              self.masterInfo.syncts = stamp
              self._ts_first_update_request = None
              rospy.logdebug("SyncThread[%s]: seteeddd timestamp %s, local %s", self.masterInfo.name, str(stamp), str(stamp_local))
            socket.setdefaulttimeout(None)
            do_wait = False
          except:
            self.masterInfo.syncts = 0.0
            import traceback
            rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())
            do_wait = True

    try:
      rospy.logdebug("SyncThread[%s] clear all registrations", self.masterInfo.name)
      socket.setdefaulttimeout(5)
      own_master = xmlrpclib.ServerProxy(self.localMasteruri)
      own_master_multi = xmlrpclib.MultiCall(own_master)
      #end routine if the master was removed
      for topic, node, uri in self.__subscriber:
        own_master_multi.unregisterSubscriber(node, topic, uri)
      for topic, node, uri in self.__publisher:
        own_master_multi.unregisterPublisher(node, topic, uri)
      for service, serviceuri, node, uri in self.__services:
        own_master_multi.unregisterService(node, service, serviceuri)
      rospy.logdebug("SyncThread[%s] execute a MultiCall", self.masterInfo.name)
      r = own_master_multi()
      rospy.logdebug("SyncThread[%s] finished", self.masterInfo.name)
    except:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR while ending: %s", self.masterInfo.name, traceback.format_exc())
    socket.setdefaulttimeout(None)

  def _doIgnoreNT(self, node, topic, topictype):
    return self._filter.is_ignored_topic(node, topic, topictype)

  def _doIgnoreNS(self, node, service):
    return self._filter.is_ignored_service(node, service)

  def _getTopicType(self, topic, topicTypes):
    for (topicname, type) in topicTypes:
      if (topicname == topic):
        return type
    return None

  def _getNodeUri(self, node, nodes, remote_masteruri):
    for (nodename, uri, masteruri, pid, local) in nodes:
      if (nodename == node) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
        # the node was registered originally to another ROS master -> do sync
        if  masteruri != self.localMasteruri:
          return uri
    return None

  def _getServiceUri(self, service, nodes, remote_masteruri):
    for (servicename, uri, masteruri, type, local) in nodes:
      if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
        if  masteruri != self.localMasteruri:
          return uri
    return None

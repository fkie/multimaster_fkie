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


import threading
import xmlrpclib
import random
import socket

import roslib; roslib.load_manifest('master_sync_fkie')
import roslib.message
import rospy

from master_discovery_fkie.common import masteruri_from_ros
from master_discovery_fkie.filter_interface import FilterInterface
from multimaster_msgs_fkie.msg import SyncTopicInfo, SyncServiceInfo, SyncMasterInfo

class SyncThread(object):
  '''
  A thread to synchronize the local ROS master with a remote master. While the 
  synchronization only the topic of the remote ROS master will be registered by
  the local ROS master. The remote ROS master will be keep unchanged.
  '''

  MAX_UPDATE_DELAY = 5 # times

  MSG_ANY_TYPE = '*'

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
    self.name = name
    self.uri = uri
    self.discoverer_name = discoverer_name
    self.monitoruri = monitoruri
    self.timestamp = timestamp
    self.timestamp_local = 0.
    self.timestamp_remote = 0.

    self.localMasteruri = masteruri_from_ros()
    rospy.logdebug("SyncThread[%s]: create this sync thread", self.name)
    # synchronization variables 
    self.__lock_info = threading.RLock()
    self.__lock_intern = threading.RLock()
    self._use_filtered_method = None
    # SyncMasterInfo with currently synchronized nodes, publisher (topic, node, nodeuri), subscriber(topic, node, nodeuri) and services
    self.__sync_info = None
    self.__unregistered = False
    # a list with published topics as a tuple of (topic name, node name, node URL)
    self.__publisher = []
    # a list with subscribed topics as a tuple of (topic name, node name, node URL)
    self.__subscriber = []
    # a list with services as a tuple of (service name, service URL, node name, node URL)
    self.__services = []
    # the state of the own ROS master is used if `sync_on_demand` is enabled or
    # to determine the type of topic subscribed remote with `Empty` type
    self.__own_state = None

    #setup the filter
    self._filter = FilterInterface()
    self._filter.load(self.name,
                      ['/rosout', rospy.get_name().replace('/', '/*')+'*', self.discoverer_name.replace('/', '/*')+'*', '/*node_manager', '/*zeroconf'], [],
                      ['/rosout', '/rosout_agg'], ['/'] if sync_on_demand else [],
                      ['/*get_loggers', '/*set_logger_level'], [],
                      # do not sync the bond message of the nodelets!!
                      ['bond/Status'],
                      [], [])

    # congestion avoidance: wait for random.random*2 sec. If an update request 
    # is received try to cancel and restart the current timer. The timer can be
    # canceled for maximal MAX_UPDATE_DELAY times.
    self._update_timer = None
    self._delayed_update = 0
    self.__on_update = False

  def getSyncInfo(self):
    '''
    Returns the synchronized publisher, subscriber and services.
    @rtype: SyncMasterInfo
    '''
    with self.__lock_info:
      if self.__sync_info is None:
        # create a sync info
        result_set = set()
        result_publisher = []
        result_subscriber = []
        result_services = []
        for (t_n, n_n, n_uri) in self.__publisher:
          result_publisher.append(SyncTopicInfo(t_n, n_n, n_uri))
          result_set.add(n_n)
        for (t_n, n_n, n_uri) in self.__subscriber:
          result_subscriber.append(SyncTopicInfo(t_n, n_n, n_uri))
          result_set.add(n_n)
        for (s_n, s_uri, n_n, n_uri) in self.__services:
          result_services.append(SyncServiceInfo(s_n, s_uri, n_n, n_uri))
          result_set.add(n_n)
        self.__sync_info = SyncMasterInfo(self.uri, list(result_set), result_publisher, result_subscriber, result_services) 
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
#    rospy.logdebug("SyncThread[%s]: update request", self.name)
    with self.__lock_intern:
      if (self.timestamp_local != timestamp):
        rospy.logdebug("SyncThread[%s]: update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp, self.timestamp_local)
        self.name = name
        self.uri = uri
        self.discoverer_name = discoverer_name
        self.monitoruri = monitoruri
        self.timestamp_remote = timestamp
        self._request_update()
#    rospy.logdebug("SyncThread[%s]: update exit", self.name)

  def setOwnMasterState(self, own_state, sync_on_demand=False):
    '''
    Sets the state of the local ROS master state. If this state is not None, the topics on demand will be synchronized. 
    @param own_state: the state of the local ROS master state
    @type own_state:  C{master_discovery_fkie/MasterInfo}
    @param sync_on_demand: if True, sync only topic, which are also local exists (Default: False)
    @type sync_on_demand:  bool
    '''
#    rospy.logdebug("SyncThread[%s]: setOwnMasterState", self.name)
    with self.__lock_intern:
      timestamp_local = own_state.timestamp_local
      if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
        rospy.logdebug("SyncThread[%s]: local state update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp_local, self.__own_state.timestamp_local if not self.__own_state is None else float('nan'))
        self.__own_state = own_state
        if sync_on_demand:
          self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
        self._request_update()
#    rospy.logdebug("SyncThread[%s]: setOwnMasterState exit", self.name)

  def stop(self):
    '''
    Stops running thread.
    '''
    rospy.logdebug("  SyncThread[%s]: stop request", self.name)
    with self.__lock_intern:
      if not self._update_timer is None:
        self._update_timer.cancel()
      self._unreg_on_finish()
    rospy.logdebug("  SyncThread[%s]: stop exit", self.name)

  def _request_update(self):
    with self.__lock_intern:
      r = random.random() * 2.
      # start update timer with a random waiting time to avoid a congestion picks on changes of ROS master state
      if self._update_timer is None or not self._update_timer.isAlive():
        del self._update_timer
        self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
        self._update_timer.start()
      else:
        if self._delayed_update < self.MAX_UPDATE_DELAY:
          # if the timer thread can be canceled start new one
          self._update_timer.cancel()
          # if callback (XMLRPC request) is already running the timer is not canceled -> test for `self.__on_update`
          if not self._update_timer.isAlive() or not self.__on_update:
            self._delayed_update += 1
            self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
            self._update_timer.start()

  def _request_remote_state(self, handler):
    self._delayed_update = 0
    self.__on_update = True
    try:
      # connect to master_monitor rpc-xml server of remote master discovery
      socket.setdefaulttimeout(20)
      remote_monitor = xmlrpclib.ServerProxy(self.monitoruri)
      # determine the getting method: older versions have not a filtered method
      if self._use_filtered_method is None:
        try:
          self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
        except:
          self._use_filtered_method = False
      remote_state = None
      # get the state informations
      if self._use_filtered_method:
        remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
      else:
        remote_state = remote_monitor.masterInfo()
      if not self.__unregistered:
        handler(remote_state)
    except:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
    finally:
      self.__on_update = False
      socket.setdefaulttimeout(None)

  def _apply_remote_state(self, remote_state):
    try:
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

      # create a multicall object
      own_master = xmlrpclib.ServerProxy(self.localMasteruri)
      own_master_multi = xmlrpclib.MultiCall(own_master)
      # fill the multicall object
      handler = []
      # sync the publishers
      publisher = []
      publisher_to_register = []
      for (topic, nodes) in publishers:
        for node in nodes:
          topictype = self._getTopicType(topic, topicTypes)
          nodeuri = self._getNodeUri(node, nodeProviders, remote_masteruri)
          if topictype and nodeuri and not self._doIgnoreNTP(node, topic, topictype):
            # register the nodes only once
            if not ((topic, node, nodeuri) in self.__publisher):
              publisher_to_register.append((topic, topictype, node, nodeuri))
            publisher.append((topic, node, nodeuri))
      # unregister not updated publishers
      for (topic, node, nodeuri) in set(self.__publisher)-set(publisher):
        own_master_multi.unregisterPublisher(node, topic, nodeuri)
        handler.append(('upub', topic, node, nodeuri))
      #register new publishers
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
          # if remote topictype is None, try to set to the local topic type
#          if not topictype and not self.__own_state is None:
#            if topic in self.__own_state.topics:
#              topictype = self.__own_state.topics[topic].type
          if not topictype:
            topictype = self.MSG_ANY_TYPE
          if topictype and nodeuri and not self._doIgnoreNTS(node, topic, topictype):
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

      # execute the multicall and update the current publicher, subscriber and services
      if not self.__unregistered:
        with self.__lock_info:
          self.__sync_info = None
          self.__publisher = publisher
          self.__subscriber = subscriber
          self.__services = services
        # update the local ROS master
        socket.setdefaulttimeout(3)
        result = own_master_multi()
        # analyze the results of the registration call
        #HACK param to reduce publisher creation, see line 372
        hack_pub = set()
        for h,(code, statusMessage, r) in zip(handler, result):
          try:
            if h[0] == 'sub':
              if code == -1:
                rospy.logwarn("SyncThread[%s] topic subscription error: %s (%s), %s %s, node: %s", self.name, h[1], h[2], str(code), str(statusMessage), h[3])
              else:
                rospy.logdebug("SyncThread[%s] topic subscribed: %s, %s %s, node: %s", self.name, h[1], str(code), str(statusMessage), h[3])
            if h[0] == 'sub' and code == 1 and len(r) > 0:
              # Horrible hack: see line 372
              hack_pub.add(h[1])
            elif h[0] == 'pub':
              if code == -1:
                rospy.logwarn("SyncThread[%s] topic advertise error: %s (%s), %s %s", self.name, h[1], h[2], str(code), str(statusMessage))
              else:
                rospy.logdebug("SyncThread[%s] topic advertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
            elif h[0] == 'usub':
              rospy.logdebug("SyncThread[%s] topic unsubscribed: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
            elif h[0] == 'upub':
              rospy.logdebug("SyncThread[%s] topic unadvertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
            elif h[0] == 'srv':
              if code == -1:
                rospy.logwarn("SyncThread[%s] service registration error: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
              else:
                rospy.logdebug("SyncThread[%s] service registered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
            elif h[0] == 'usrv':
              rospy.logdebug("SyncThread[%s] service unregistered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
          except:
            import traceback
            rospy.logerr("SyncThread[%s] ERROR while analyzing the results of the registration call [%s]: %s", self.name, h[1], traceback.format_exc())
        # Horrible hack: the response from registerSubscriber() can contain a
        # list of current publishers.  But we don't have a way of injecting them
        # into rospy here.  Now, if we get a publisherUpdate() from the master,
        # everything will work.  So, we ask the master if anyone is currently
        # publishing the topic, grab the advertised type, use it to advertise
        # ourselves, then unadvertise, triggering a publisherUpdate() along the
        # way.
        # We create publisher locally as a hack, to get callback set up properly for already registered local publishers
        if hack_pub:
          rospy.loginfo("SyncThread[%s] Horrible hack: create and delete publisher to trigger an update for subscribed topics: %s", self.name, hack_pub)
        for m in hack_pub:
          try:
            topicPub = rospy.Publisher(m, rospy.msg.AnyMsg, queue_size=1)
            topicPub.unregister()
            del topicPub
          except:
              import traceback
              rospy.logerr("SyncThread[%s] ERROR while hack subscriber[%s]: %s", self.name, h[1], traceback.format_exc())
        # set the last synchronization time
        self.timestamp = stamp
        self.timestamp_local = stamp_local
        rospy.logdebug("SyncThread[%s]: current timestamp %.9f, local %.9f", self.name, stamp, stamp_local)
        if self.timestamp_remote > stamp_local:
          rospy.logdebug("SyncThread[%s]: invoke next update, remote ts: %.9f", self.name, self.timestamp_remote)
          self._update_timer = threading.Timer(random.random() * 2., self._request_remote_state, args=(self._apply_remote_state,))
          self._update_timer.start()
    except:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
    finally:
      socket.setdefaulttimeout(None)

  def _unreg_on_finish(self):
    with self.__lock_info:
      self.__unregistered = True
      try:
        rospy.logdebug("    SyncThread[%s] clear all registrations", self.name)
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
        rospy.logdebug("    SyncThread[%s] execute a MultiCall", self.name)
        r = own_master_multi()
        rospy.logdebug("    SyncThread[%s] finished", self.name)
      except:
        import traceback
        rospy.logwarn("SyncThread[%s] ERROR while ending: %s", self.name, traceback.format_exc())
      socket.setdefaulttimeout(None)

  def _doIgnoreNTP(self, node, topic, topictype):
    return self._filter.is_ignored_publisher(node, topic, topictype)

  def _doIgnoreNTS(self, node, topic, topictype):
    return self._filter.is_ignored_subscriber(node, topic, topictype)

  def _doIgnoreNS(self, node, service):
    return self._filter.is_ignored_service(node, service)

  def _getTopicType(self, topic, topicTypes):
    for (topicname, topic_type) in topicTypes:
      if (topicname == topic):
        return topic_type.replace('None', '')
    return None

  def _getNodeUri(self, node, nodes, remote_masteruri):
    for (nodename, uri, masteruri, pid, local) in nodes:
      if (nodename == node) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
        # the node was registered originally to another ROS master -> do sync
        if  masteruri != self.localMasteruri:
          return uri
    return None

  def _getServiceUri(self, service, nodes, remote_masteruri):
    for (servicename, uri, masteruri, topic_type, local) in nodes:
      if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
        if  masteruri != self.localMasteruri:
          return uri
    return None

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


import threading
import xmlrpclib

import roslib; roslib.load_manifest('master_sync_fkie')
import rospy
import rosgraph.masterapi


class MasterInfo(object):
  '''
  The Master representation stored in the L{SyncThread}. 
  '''
  def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, lastsync=rospy.Time(0)):
    self.name = name
    self.uri = uri
    self.timestamp = timestamp
    self.discoverer_name = discoverer_name
    self.monitoruri = monitoruri
    self.lastsync = lastsync
    self.syncts = rospy.Time(0)
  
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
  
  def __init__(self, name, uri, discoverer_name, monitoruri, timestamp):
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
    @type timestamp:  C{rospy.Time}
    '''
    # init thread
    threading.Thread.__init__(self)
    self.masterInfo = MasterInfo(name, uri, discoverer_name, monitoruri, timestamp)
    # synchronization variables 
    self.__cv = threading.Condition()
    self.__stop = False
    # a dictionary with published topics, the key is a tuple of (topic name, node name, node URL), value is a boolean
    self.__publishers = {}
    # a dictionary with subscribed topics, the key is a tuple of (topic name, node name, node URL), value is a boolean
    self.__subscribers = {}
    # a dictionary with services, the key is a tuple of (service name, service URL, node name, node URL), value is a boolean
    self.__services = {}
    
    self.ignore = ['/rosout', rospy.get_name(), self.masterInfo.discoverer_name, '/default_cfg', '/node_manager']
    if rospy.has_param('~ignore_nodes'):
      self.ignore[len(self.ignore):] = rospy.get_param('~ignore_nodes')
    self.start()


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
    @type timestamp:  C{rospy.Time}
    '''
    master_info = MasterInfo(name, uri, discoverer_name, monitoruri, timestamp)
    rospy.logdebug("SyncThread[%s]: update request", self.masterInfo.name)
    if self.__cv.acquire(blocking=False):
      rospy.logdebug("SyncThread[%s]: update notify", self.masterInfo.name)
      if (self.masterInfo.timestamp != master_info.timestamp):
        master_info.lastsync = self.masterInfo.lastsync
        self.masterInfo = master_info
        self.masterInfo.syncts = rospy.Time(0)
        self.__cv.notify()
      self.__cv.release()
    rospy.logdebug("SyncThread[%s]: update exit", self.masterInfo.name)

  def stop(self):
    '''
    Stops running thread.
    '''
    rospy.logdebug("SyncThread[%s]: stop request", self.masterInfo.name)
    if self.__cv.acquire(blocking=False):
      self.__stop = True
      self.__cv.notify()
      self.__cv.release()
    rospy.logdebug("SyncThread[%s]: stop exit", self.masterInfo.name)

  def run(self):
    '''
    The synchronization procedure waits for notifications of the L{update()} method.
    If the remote ROS master is changed, the changes will be performed on the 
    local ROS master.
    '''
    while not self.__stop and not rospy.is_shutdown():
      self.__cv.acquire()
      ''' wait for new sync update '''
      rospy.logdebug("SyncThread[%s]: run waiting", self.masterInfo.name)
      if not (self.masterInfo.syncts == rospy.Time(0)):
        self.__cv.wait()
      rospy.logdebug("SyncThread[%s]: run notify received", self.masterInfo.name)
      if (not self.__stop):
        rospy.logdebug("SyncThread[%s]: run sync", self.masterInfo.name)
        ''' try to sync ''' 
        try:
          # initialize the lists to update topics
          for key in self.__publishers.keys():
            self.__publishers[key] = False
          for key in self.__subscribers.keys():
            self.__subscribers[key] = False
          for key in self.__services.keys():
            self.__services[key] = False
            
          #coonect to master_monitor rpc-xml server
          remote_monitor = xmlrpclib.ServerProxy(self.masterInfo.monitoruri)
          remote_state = remote_monitor.masterInfo()
          stamp = rospy.Time.from_sec(float(remote_state[0])/1000000000)
          remote_masteruri = remote_state[1]
          remote_mastername = remote_state[2]
          publishers = remote_state[3]
          subscribers = remote_state[4]
          services = remote_state[5]
          topicTypes = remote_state[6]
          nodeProviders = remote_state[7]
          serviceProviders = remote_state[8]
          # sync the publishers
          for (topic, nodes) in publishers:
            if not (topic in ['/rosout', 'rosout_agg']):
              for node in nodes:
                topictype = self._getTopicType(topic, topicTypes)
                nodeuri = self._getNodeUri(node, nodeProviders)
                if topictype and nodeuri and (not self._doIgnore(node)):
                  # register the nodes only once
                  if not ((topic, node, nodeuri) in self.__publishers):
                    self.__registerPublisher(topic, topictype, node, nodeuri)
                  self.__publishers[(topic, node, nodeuri)] = True
          # unregister not updated topics
          for key in self.__publishers.keys():
            if (not self.__publishers[key]):
              self.__unregisterPublisher(key[0], key[1], key[2])
              del self.__publishers[key]
  
          # sync the subscribers
          for (topic, nodes) in subscribers:
            if not (topic in ['/rosout', 'rosout_agg']):
              for node in nodes:
                topictype = self._getTopicType(topic, topicTypes)
                nodeuri = self._getNodeUri(node, nodeProviders)
                if topictype and nodeuri and (not self._doIgnore(node)):
                  # register the node as subscriber in local ROS master
                  if not ((topic, node, nodeuri) in self.__subscribers):
                    self.__registerSubscriber(topic, topictype, node, nodeuri)
                  self.__subscribers[(topic, node, nodeuri)] = True
          # unregister not updated topics
          for key in self.__subscribers.keys():
            if (not self.__subscribers[key]):
              self.__unregisterSubscriber(key[0], key[1], key[2])
              del self.__subscribers[key]
          
          # sync the services
          for (service, nodes) in services:
            for node in nodes:
              serviceuri = self._getServiceUri(service, serviceProviders)
              nodeuri = self._getNodeUri(node, nodeProviders)
              if serviceuri and (not self._doIgnore(node)):
                # register the node as publisher in local ROS master
                if not ((service, serviceuri, node, nodeuri) in self.__services):
                  self.__registerService(service, serviceuri, node, nodeuri)
                self.__services[(service, serviceuri, node, nodeuri)] = True
          # unregister not updated topics
          for key in self.__services.keys():
            if (not self.__services[key]):
              self.__unregisterService(key[0], key[1], key[2])
              del self.__services[key]

          # set the last synchronization time
          self.masterInfo.lastsync = stamp
          self.masterInfo.syncts = stamp
        except:
          self.masterInfo.syncts = rospy.Time(0)
          import traceback
          rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())
          rospy.sleep(3)
      self.__cv.release()

    #end routine if the master was removed
    for topic, node, uri in self.__publishers:
      self.__unregisterPublisher(topic, node, uri)
    for topic, node, uri in self.__subscribers:
      self.__unregisterSubscriber(topic, node, uri)
    for service, serviceuri, node, uri in self.__services:
      self.__unregisterService(service, serviceuri, node)

  def _doIgnore(self, node):
    for n in self.ignore:
      if node.startswith(n):
        return True
    return False
    
  def _getTopicType(self, topic, topicTypes):
    for (topicname, type) in topicTypes:
      if (topicname == topic):
        return type
    return None

  def _getNodeUri(self, node, nodes):
    for (nodename, uri, pid, local) in nodes:
      if (nodename == node) and local == 'local':
        return uri
    return None

  def _getServiceUri(self, service, nodes):
    for (servicename, uri, type, local) in nodes:
      if (servicename == service) and local == 'local':
        return uri
    return None

  def __registerPublisher(self, topic, type, node, nodeuri):
    try:
      rospy.loginfo("SendThread[%s] register published topic: %s [%s(%s)]", self.masterInfo.name, topic, node, nodeuri)
      lm = rosgraph.masterapi.Master(node)
      lm.registerPublisher(topic, type, nodeuri)
    except Exception:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())

  def __unregisterPublisher(self, topic, node, nodeuri):
    try:
      rospy.loginfo("SendThread[%s] unregister published topic: %s [%s]", self.masterInfo.name, topic, nodeuri)
      lm = rosgraph.masterapi.Master(node)
      lm.unregisterPublisher(topic, nodeuri)
    except Exception:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())


  def __registerSubscriber(self, topic, type, node, nodeuri):
    try:
      rospy.loginfo("SendThread[%s] register subscriber topic: %s [%s(%s)]", self.masterInfo.name, topic, node, nodeuri)
      lm = rosgraph.masterapi.Master(node)
      lm.registerSubscriber(topic, type, nodeuri)
      # Horrible hack: the response from registerSubscriber() can contain a
      # list of current publishers.  But we don't have a way of injecting them
      # into rospy here.  Now, if we get a publisherUpdate() from the master,
      # everything will work.  So, we ask the master if anyone is currently
      # publishing the topic, grab the advertised type, use it to advertise
      # ourselves, then unadvertise, triggering a publisherUpdate() along the
      # way.

      # We create publisher locally as a hack, to get callback set up properly for already registered local publishers
      topics = lm.getPublishedTopics('')
      for (t, type) in topics:
        # create the publisher only if one already exists
        if t == topic:
          topicPub = rospy.Publisher(topic, roslib.message.get_message_class(type))
          topicPub.unregister()
          del topicPub
    except:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())

  def __unregisterSubscriber(self, topic, node, nodeuri):
    try:
      rospy.loginfo("SendThread[%s] unregister subscriber topic: %s [%s]", self.masterInfo.name, topic, nodeuri)
      lm = rosgraph.masterapi.Master(node)
      lm.unregisterSubscriber(topic, nodeuri)
    except Exception:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())

  def __registerService(self, service, serviceuri, node, nodeuri):
    try:
      rospy.loginfo("SendThread[%s] register service: %s [%s, %s(%s)]", self.masterInfo.name, service, serviceuri, node, nodeuri)
      lm = rosgraph.masterapi.Master(node)
      lm.registerService(service, serviceuri, nodeuri)
    except Exception:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())

  def __unregisterService(self, service, serviceuri, node):
    try:
      rospy.loginfo("SendThread[%s] unregister service: %s [%s]", self.masterInfo.name, service, serviceuri)
      lm = rosgraph.masterapi.Master(node)
      lm.unregisterService(service, serviceuri)
    except Exception:
      import traceback
      rospy.logwarn("SyncThread[%s] ERROR: %s", self.masterInfo.name, traceback.format_exc())

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

import uuid
import threading
import time
import xmlrpclib
import socket

import roslib; roslib.load_manifest('master_sync_fkie')
import rospy

from master_discovery_fkie.common import masteruri_from_ros, resolve_url, read_interface, create_pattern, is_empty_pattern
from sync_thread import SyncThread
from multimaster_msgs_fkie.msg import MasterState#, LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo
from multimaster_msgs_fkie.srv import DiscoverMasters, GetSyncInfo, GetSyncInfoResponse
import master_discovery_fkie.interface_finder as interface_finder
from master_discovery_fkie.master_info import MasterInfo



class Main(object):
  '''
  '''
  
  UPDATE_INTERVALL = 30
  
  def __init__(self):
    '''
    Creates a new instance. Find the topic of the master_discovery node using 
    L{master_discovery_fkie.interface_finder.get_changes_topic()}. Also the 
    parameter C{~ignore_hosts} will be analyzed to exclude hosts from sync.
    '''
    self.masters = {}
    # the connection to the local service master 
    self.materuri = self.getMasteruri()
    '''@ivar: the ROS master URI of the C{local} ROS master. '''
    self.__lock = threading.RLock()
    #load interface
    self._loadInterface()
    # subscribe to changes notifier topics
    topic_names = interface_finder.get_changes_topic(self.getMasteruri())
    self.sub_changes = dict()
    '''@ivar: {dict} with topics C{(name: L{rospy.Subscriber})} publishes the changes of the discovered ROS masters.'''
    for topic_name in topic_names:
      rospy.loginfo("listen for updates on %s", topic_name)
      self.sub_changes[topic_name] = rospy.Subscriber(topic_name, MasterState, self.handlerMasterStateMsg)
    self.__timestamp_local = None
    self.__own_state = None
    self.update_timer = None
    self.own_state_getter = None
    self._join_threads = dict() # threads waiting for stopping the sync thread
    # initialize the ROS services
    rospy.Service('~get_sync_info', GetSyncInfo, self.rosservice_get_sync_info)
    rospy.on_shutdown(self.finish)
    self.retrieveMasters()

  def handlerMasterStateMsg(self, data):
    '''
    The method to handle the received MasterState messages. Based on this message
    new threads to synchronize with remote ROS master will be created, updated or
    removed.
    @param data: the received message
    @type data: L{master_discovery_fkie.MasterState}
    '''
    with self.__lock:
      if not rospy.is_shutdown():
        if data.state in [MasterState.STATE_REMOVED]:
          self.removeMaster(data.master.name)
        elif data.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
          m = data.master
          self.updateMaster(m.name, m.uri, m.timestamp, m.timestamp_local, m.discoverer_name, m.monitoruri)

  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it. The 'materuri' attribute will be set to the requested value.
    @return: ROS master URI
    @rtype: C{str} or C{None}
    '''
    if not hasattr(self, 'materuri') or self.materuri is None:
      masteruri = masteruri_from_ros()
      master = xmlrpclib.ServerProxy(masteruri)
      code, message, self.materuri = master.getUri(rospy.get_name())
    return self.materuri

  def retrieveMasters(self):
    '''
    This method use the service 'list_masters' of the master_discoverer to get 
    the list of discovered ROS master. Based on this list the L{SyncThread} for
    synchronization will be created.
    @see: L{master_discovery_fkie.interface_finder.get_listmaster_service()}
    '''
    if not rospy.is_shutdown():
      rospy.loginfo("Update ROS master list...")
      service_names = interface_finder.get_listmaster_service(self.getMasteruri(), False)
      for service_name in service_names:
        rospy.loginfo("service 'list_masters' found on %s", service_name)
        try:
          with self.__lock:
    #        rospy.wait_for_service(service_name)
            try:
              socket.setdefaulttimeout(5)
              discoverMasters = rospy.ServiceProxy(service_name, DiscoverMasters)
              resp = discoverMasters()
              masters = []
              for m in resp.masters:
                if not self._re_ignore_hosts.match(m.name) or self._re_sync_hosts.match(m.name): # do not sync to the master, if it is in ignore list
                  masters.append(m.name)
                self.updateMaster(m.name, m.uri, m.timestamp, m.timestamp_local, m.discoverer_name, m.monitoruri)
              for key in set(self.masters.keys()) - set(masters):
                self.removeMaster(self.masters[key].name)
            except rospy.ServiceException, e:
              rospy.logwarn("ERROR Service call 'list_masters' failed: %s", str(e))
        except:
          import traceback
          rospy.logwarn("ERROR while initial list masters: %s", traceback.format_exc())
        finally:
          socket.setdefaulttimeout(None)
      self.update_timer = threading.Timer(self.UPDATE_INTERVALL, self.retrieveMasters)
      self.update_timer.start()


  def updateMaster(self, mastername, masteruri, timestamp, timestamp_local, discoverer_name, monitoruri):
    '''
    Updates the timestamp of the given ROS master, or creates a new SyncThread to
    synchronize the local master with given ROS master.
    @param mastername: the name of the remote ROS master to update or synchronize.
    @type mastername: C{str}
    @param masteruri: the URI of the remote ROS master.
    @type masteruri: C{str}
    @param timestamp: the timestamp of the remote ROS master.
    @type timestamp: L{float64}
    @param timestamp_local: the timestamp of the remote ROS master. (only local changes)
    @type timestamp_local: L{float64}
    @param discoverer_name: the name of the remote master_discoverer node
    @type discoverer_name: C{str}
    @param monitoruri: the URI of the RPC interface of the remote master_discoverer node.
    @type monitoruri: C{str}
    '''
    try:
      with self.__lock:
        if (masteruri != self.materuri):
          if (is_empty_pattern(self._re_ignore_hosts) or not self._re_ignore_hosts.match(mastername)
              or (not is_empty_pattern(self._re_sync_hosts) and not self._re_sync_hosts.match(mastername) is None)): # do not sync to the master, if it is in ignore list
            if (mastername in self.masters):
              # updates only, if local changes are occured
              self.masters[mastername].update(mastername, masteruri, discoverer_name, monitoruri, timestamp_local)
            else:
              self.masters[mastername] = SyncThread(mastername, masteruri, discoverer_name, monitoruri, 0.0, self.__sync_topics_on_demand)
              if not self.__own_state is None:
                self.masters[mastername].setOwnMasterState(MasterInfo.from_list(self.__own_state))
              self.masters[mastername].update(mastername, masteruri, discoverer_name, monitoruri, timestamp_local)
#              self.own_state_getter = threading.Thread(target=self.get_own_state, args=(monitoruri,))
#              self.own_state_getter.start()
        elif self.__timestamp_local != timestamp_local and self.__sync_topics_on_demand:
          # get the master info from local discovery master and set it to all sync threads
          self.own_state_getter = threading.Thread(target=self.get_own_state, args=(monitoruri,))
          self.own_state_getter.start()
    except:
      import traceback
      rospy.logwarn("ERROR while update master[%s]: %s", str(mastername), traceback.format_exc())

  def get_own_state(self, monitoruri):
    # get the master info from local discovery master and set it to all sync threads
    # This function is running in a thread!!!
    try:
      socket.setdefaulttimeout(3)
      own_monitor = xmlrpclib.ServerProxy(monitoruri)
      self.__own_state = own_monitor.masterInfo()
      own_state = MasterInfo.from_list(self.__own_state)
      socket.setdefaulttimeout(None)
      with self.__lock:
        # update the state for all sync threads
        for (mastername, s) in self.masters.iteritems():
          s.setOwnMasterState(own_state, self.__sync_topics_on_demand)
        self.__timestamp_local = own_state.timestamp_local
    except:
      import traceback
      print traceback.print_exc()
      socket.setdefaulttimeout(None)
      time.sleep(3)
      if not self.own_state_getter is None and not rospy.is_shutdown():
        self.own_state_getter = threading.Thread(target=self.get_own_state, args=(monitoruri,))
        self.own_state_getter.start()

  def removeMaster(self, ros_master_name):
    '''
    Removes the master with given name from the synchronization list.
    @param ros_master_name: the name of the ROS master to remove.
    @type ros_master_name: C{str}
    '''
    try:
      with self.__lock:
        if (ros_master_name in self.masters):
          m = self.masters.pop(ros_master_name)
          ident = uuid.uuid4()
          thread = threading.Thread(target=self._threading_stop_sync, args=(m, ident))
          self._join_threads[ident] = thread
          thread.start()
    except Exception:
      import traceback
      rospy.logwarn("ERROR while removing master[%s]: %s", ros_master_name, traceback.format_exc())

  def _threading_stop_sync(self, sync_thread, ident):
    if isinstance(sync_thread, SyncThread):
      rospy.loginfo("  Stop synchronization to `%s`"%sync_thread.name)
      sync_thread.stop()
      with self.__lock:
        del self._join_threads[ident]
      rospy.loginfo("  Finished synchronization to `%s`"%sync_thread.name)
      del sync_thread

  def finish(self, msg=''):
    '''
    Removes all remote masters and unregister their topics and services.
    '''
    rospy.loginfo("Stop synchronization...")
    with self.__lock:
      # stop update timer
      rospy.loginfo("  Stop timers...")
      if not self.update_timer is None:
        self.update_timer.cancel()
      # unregister from update topics
      rospy.loginfo("  Unregister from master discovery...")
      for (k, v) in self.sub_changes.iteritems():
        v.unregister()
      self.own_state_getter = None
      # Stop all sync threads
      for key in self.masters.keys():
        rospy.loginfo("  Remove master: %s", key)
        self.removeMaster(key)
    # wait for their ending
    while len(self._join_threads) > 0:
      rospy.loginfo("  Wait for ending of %s threads ...", str(len(self._join_threads)))
      time.sleep(1)
    rospy.loginfo("Synchronization is now off")

  def rosservice_get_sync_info(self, req):
    '''
    Callback for the ROS service to get the info to synchronized nodes.
    '''
    masters = list()
    try:
      with self.__lock:
        for (mastername, s) in self.masters.iteritems():
          masters.append(s.getSyncInfo())
    except:
      import traceback
      traceback.print_exc()
    finally:
      return GetSyncInfoResponse(masters)

  def _loadInterface(self):
    interface_file = resolve_url(rospy.get_param('~interface_url', ''))
    if interface_file:
      rospy.loginfo("interface_url: %s", interface_file)
    try:
      data = read_interface(interface_file) if interface_file else {}
      # set the ignore hosts list
      self._re_ignore_hosts = create_pattern('ignore_hosts', data, interface_file, [])
      # set the sync hosts list
      self._re_sync_hosts = create_pattern('sync_hosts', data, interface_file, [])
      self.__sync_topics_on_demand = False
      if interface_file:
        if data.has_key('sync_topics_on_demand'):
          self.__sync_topics_on_demand = data['sync_topics_on_demand']
      elif rospy.has_param('~sync_topics_on_demand'):
        self.__sync_topics_on_demand = rospy.get_param('~sync_topics_on_demand')
      rospy.loginfo("sync_topics_on_demand: %s", self.__sync_topics_on_demand)
    except:
      import traceback
      # kill the ros node, to notify the user about the error
      rospy.logerr("Error on load interface: %s", traceback.format_exc())
      import os, signal
      os.kill(os.getpid(), signal.SIGKILL)

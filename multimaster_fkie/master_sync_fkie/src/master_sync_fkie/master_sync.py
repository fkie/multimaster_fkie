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
import sys
import time
import xmlrpclib

import roslib; roslib.load_manifest('master_sync_fkie')
import rospy

from sync_thread import SyncThread
from master_discovery_fkie.msg import *
from master_discovery_fkie.srv import *
import master_discovery_fkie.interface_finder as interface_finder


class Main(object):
  '''
  '''

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
    self.ignore = []
    '''@ivar: the list with host names, which are not sync.'''
    if rospy.has_param('~ignore_hosts'):
      self.ignore[len(self.ignore):] = rospy.get_param('~ignore_hosts')
    topic_names = interface_finder.get_changes_topic(self.getMasteruri())
    self.sub_changes = dict()
    '''@ivar: {dict} with topics C{(name: L{rospy.Subscriber})} publishes the changes of the discovered ROS masters.'''
    for topic_name in topic_names:
      rospy.loginfo("listen for updates on %s", topic_name)
      self.sub_changes[topic_name] = rospy.Subscriber(topic_name, MasterState, self.handlerMasterStateMsg)
    rospy.on_shutdown(self.finish)
    # initialize the ROS services
    rospy.Service('~get_sync_info', GetSyncInfo, self.rosservice_get_sync_info)

    self.update_timer = None
    self.retrieveMasters()

  def handlerMasterStateMsg(self, data):
    '''
    The method to handle the received MasterState messages. Based on this message
    new threads to synchronize with remote ROS master will be created, updated or
    removed.
    @param data: the received message
    @type data: L{master_discovery_fkie.MasterState}
    '''
    try:
      self.__lock.acquire()
      if data.state in [MasterState.STATE_REMOVED]:
        self.removeMaster(data.master.name)
      elif data.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
        m = data.master
        self.updateMaster(m.name, m.uri, m.timestamp, m.discoverer_name, m.monitoruri)
    finally:
      self.__lock.release()

  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it. The 'materuri' attribute will be set to the requested value.
    @return: ROS master URI
    @rtype: C{str} or C{None}
    '''
    if not hasattr(self, 'materuri') or self.materuri is None:
      masteruri = self._masteruri_from_ros()
      master = xmlrpclib.ServerProxy(masteruri)
      code, message, self.materuri = master.getUri(rospy.get_name())
    return self.materuri

  def _masteruri_from_ros(self):
    '''
    Returns the master URI depending on ROS distribution API.
    @return: ROS master URI
    @rtype: C{str}
    @see: L{rosgraph.rosenv.get_master_uri()} (fuerte)
    @see: L{roslib.rosenv.get_master_uri()} (prior)
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
 
  
  def retrieveMasters(self):
    '''
    This method use the service 'list_masters' of the master_discoverer to get 
    the list of discovered ROS master. Based on this list the L{SyncThread} for
    synchronization will be created.
    @see: L{master_discovery_fkie.interface_finder.get_listmaster_service()}
    '''
    service_names = interface_finder.get_listmaster_service(self.getMasteruri(), False)
    for service_name in service_names:
      rospy.loginfo("service 'list_masters' found on %s", service_name)
      self.__lock.acquire()
      try:
#        rospy.wait_for_service(service_name)
        try:
          discoverMasters = rospy.ServiceProxy(service_name, DiscoverMasters)
          resp = discoverMasters()
          masters = []
          for m in resp.masters:
            if not m.name in self.ignore: # do not sync to the master, if it is in ignore list
              masters.append(m.name)
              self.updateMaster(m.name, m.uri, m.timestamp, m.discoverer_name, m.monitoruri)
          for key in set(self.masters.keys()) - set(masters):
            self.removeMaster(self.masters[key].name)
        except rospy.ServiceException, e:
          rospy.logwarn("ERROR Service call 'list_masters' failed: %s", str(e))
      except:
        import traceback
        rospy.logwarn("ERROR while initial list masters: %s", traceback.format_exc())
      finally:
        self.__lock.release()
    self.update_timer = threading.Timer(15.0, self.retrieveMasters)
    self.update_timer.start()


  def updateMaster(self, mastername, masteruri, timestamp, discoverer_name, monitoruri):
    '''
    Updates the timestamp of the given ROS master, or creates a new SyncThread to
    synchronize the local master with given ROS master.
    @param mastername: the name of the remote ROS master to update or synchronize.
    @type mastername: C{str}
    @param masteruri: the URI of the remote ROS master.
    @type masteruri: C{str}
    @param timestamp: the timestamp of the remote ROS master.
    @type timestamp: L{float64}
    @param discoverer_name: the name of the remote master_discoverer node
    @type discoverer_name: C{str}
    @param monitoruri: the URI of the RPC interface of the remote master_discoverer node.
    @type monitoruri: C{str}
    '''
    self.__lock.acquire()
    try:
      if (masteruri != self.materuri) and not mastername in self.ignore: # do not sync to the master, if it is in ignore list
#        print "--update:", ros_master.uri, mastername
        if (mastername in self.masters):
          self.masters[mastername].update(mastername, masteruri, discoverer_name, monitoruri, timestamp)
        else:
#          print "add a sync thread to:", mastername, ros_master.uri
          self.masters[mastername] = SyncThread(mastername, masteruri, discoverer_name, monitoruri, 0.0)
    except:
      import traceback
      rospy.logwarn("ERROR while update master[%s]: %s", str(mastername), traceback.format_exc())
    finally:
      self.__lock.release()

  def removeMaster(self, ros_master_name):
    '''
    Removes the master with given name from the synchronization list.
    @param ros_master_name: the name of the ROS master to remove.
    @type ros_master_name: C{str}
    '''
    self.__lock.acquire()
    try:
      if (ros_master_name in self.masters):
        m = self.masters.pop(ros_master_name)
        m.stop()
        del m
    except Exception:
      import traceback
      rospy.logwarn("ERROR while removing master[%s]: %s", ros_master_name, traceback.format_exc())
    finally:
      self.__lock.release()

  def finish(self):
    '''
    Removes all remote masters and unregister their topics and services.
    '''
    rospy.logdebug("Stop synchronization")
    self.__lock.acquire()
    if not self.update_timer is None:
      self.update_timer.cancel()
    for key in self.masters.keys():
      m = self.masters[key]
      m.stop()
      del m
    if hasattr(self, "sub_changes"):
      for key, item in self.sub_changes.items():
        item.unregister()
    self.__lock.release()
    
  def rosservice_get_sync_info(self, req):
    '''
    Callback for the ROS service to get the info to synchronized nodes.
    '''
    masters = list()
    self.__lock.acquire()
    try:
      for (mastername, s) in self.masters.iteritems():
        (nodes, service) = s.getSyncInfo()
        masters.append(SyncMasterInfo(s.masterInfo.uri, nodes, service))
    except:
      import traceback
      traceback.print_exc()
    finally:
      self.__lock.release()
      return GetSyncInfoResponse(masters)

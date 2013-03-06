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
import copy
import sys
import socket
import time

import roslib; roslib.load_manifest('master_discovery_fkie')
import rospy
import roslib.network

from master_discovery_fkie.msg import *
from master_discovery_fkie.srv import *
from master_monitor import MasterMonitor, MasterConnectionException
from udp import McastSocket


class DiscoveredMaster(object):
  '''
  The class stores all information about the remote ROS master and the all
  received heartbeat messages of the remote node. On first contact a theaded 
  connection to remote discoverer will be established to get additional 
  information about the ROS master.
  '''
  def __init__(self, monitoruri, heartbeat_rate=1., timestamp=0.0, callback_master_state=None):
    '''
    Initialize method for the DiscoveredMaster class.
    @param monitoruri: The URI of the remote RPC server, which moniter the ROS master
    @type monitoruri:  C{str}
    @param heartbeat_rate: The remote rate, which is used to send the heartbeat messages. 
    @type heartbeat_rate:  C{float} (Default: C{1.})
    @param timestamp: The timestamp of the state of the remoter ROS master
    @type timestamp:  C{float} (Default: c{0})
    @param callback_master_state: the callback method to publish the changes of the ROS masters
    @type callback_master_state: C{<method>(master_discovery_fkie/MasterState)}  (Default: C{None})
    '''
    self.masteruri = None
    self.mastername = None
    self.timestamp = timestamp
    self.discoverername = None
    self.monitoruri = monitoruri
    self.heartbeat_rate = heartbeat_rate
    self.heartbeats = list()
    self.last_heartbeat_ts = time.time()
    self.online = False
    self.callback_master_state = callback_master_state
    # create a thread to retrieve additional information about the remote ROS master
    self._retrieveThread = threading.Thread(target = self.__retrieveMasterinfo)
    self._retrieveThread.setDaemon(True)
    self._retrieveThread.start()

  def addHeartbeat(self, timestamp, rate):
    '''
    Adds a new heartbeat measurement. If it is a new timestamp a ROS message 
    about the change of this ROS master will be published into ROS network.
    @param timestamp: The new timestamp of the ROS master state
    @type timestamp:  C{float}
    @param rate: The remote rate, which is used to send the heartbeat messages. 
    @type rate:  C{float}
    '''
    cur_time = time.time()
    self.heartbeats.append(cur_time)
    self.last_heartbeat_ts = cur_time
    # reset the list, if the heartbeat is changed
    if self.heartbeat_rate != rate:
      self.heartbeat_rate = rate
      self.heartbeats = list()
    # publish new master state, if the timestamp is changed 
    if (self.timestamp != timestamp or not self.online):
      self.timestamp = timestamp
      if not (self.masteruri is None):
        #set the state to 'online'
        self.online = True
        if not (self.callback_master_state is None):
          self.callback_master_state(MasterState(MasterState.STATE_CHANGED, 
                                                 ROSMaster(str(self.mastername), 
                                                           self.masteruri, 
                                                           self.timestamp, 
                                                           self.online, 
                                                           self.discoverername, 
                                                           self.monitoruri)))

  def removeHeartbeats(self, timestamp):
    '''
    Removes all hearbeat measurements, which are older as the given timestamp.
    @param timestamp: heartbeats older this timestamp will be removed.
    @type timestamp:  C{float}
    @return: the count of removed heartbeats
    @rtype: C{int}
    '''
    do_remove = True
    removed = 0
    while do_remove:
      if len(self.heartbeats) > 0 and self.heartbeats[0] < timestamp:
        del self.heartbeats[0]
        removed = removed + 1
      else:
        do_remove = False
    return removed

  def setOffline(self):
    '''
    Sets this master to offline and publish the new state to the ROS network.
    '''
    if not (self.callback_master_state is None) and self.online:
      self.callback_master_state(MasterState(MasterState.STATE_CHANGED, 
                                             ROSMaster(str(self.mastername), 
                                                       self.masteruri, 
                                                       self.timestamp, 
                                                       self.online, 
                                                       self.discoverername, 
                                                       self.monitoruri)))
    self.online = False

  def __retrieveMasterinfo(self):
    '''
    Connects to the remote RPC server of the discoverer node and gets the 
    information about the Master URI, name of the service, and other. The 
    getMasterInfo() method will be used. On problems the connection will be 
    reestablished until the information will be get successful.
    '''
    if not (self.monitoruri is None):
      while self._retrieveThread.is_alive() and not rospy.is_shutdown() and (self.mastername is None):
        try:
#          print "get Info about master", self.monitoruri
          remote_monitor = xmlrpclib.ServerProxy(self.monitoruri)
          timestamp, masteruri, mastername, nodename, monitoruri = remote_monitor.masterContacts()
        except:
          import traceback
          rospy.logwarn("socket error: %s", traceback.format_exc())
          time.sleep(1)
        else:
          if float(timestamp) != 0:
            self.masteruri = masteruri
            self.mastername = mastername
            self.discoverername = nodename
#            self.monitoruri = monitoruri
            self.timestamp = float(timestamp)
            self.online = True
            #publish new node 
            if not (self.callback_master_state is None):
              self.callback_master_state(MasterState(MasterState.STATE_NEW, 
                                                     ROSMaster(str(self.mastername), 
                                                               self.masteruri, 
                                                               self.timestamp, 
                                                               self.online, 
                                                               self.discoverername, 
                                                               self.monitoruri)))
          else:
            time.sleep(1)



class Discoverer(threading.Thread):
  '''
  The class to publish the current state of the ROS master.
  '''

  VERSION = 1
  '''@ivar: the version of the packet format described by L{HEARTBEAT_FMT}'''
  '''
  Version 1: 'cBBiiH'
    one character 'R'
    unsigned char: version of the hearbeat message
    unsigned char: rate of the heartbeat message in HZ*10. Maximal rate: 25.5 Hz -> value 255
    int: secs of the ROS Master state
    int: nsecs of the ROS Master state
    unsigned short: the port number of the RPC Server of the remote ROS-Core monitor
  '''
  HEARTBEAT_FMT = 'cBBiiH'
  ''' @ivar: packet format description, see: U{http://docs.python.org/library/struct.html} '''
  HEARTBEAT_HZ = 2
  ''' @ivar: the send rate of the heartbeat packets in hz (Default: 2 Hz)'''
  MEASUREMENT_INTERVALS = 5
  ''' @ivar: the count of intervals (1 sec) used for a quality calculation. If 
  HEARTBEAT_HZ is smaller then 1, MEASUREMENT_INTERVALS will be divided by HEARTBEAT_HZ value. 
  (Default: 5 sec are used to determine the link qaulity)'''
  TIMEOUT_FACTOR = 1.4
  ''' @ivar: the timeout is defined by calculated measurement duration multiplied by TIMEOUT_FAKTOR. ''' 
  ROSMASTER_HZ = 1
  ''' @ivar: the test rate of ROS master state in Hz (Default: 1 Hz). '''
  REMOVE_AFTER = 300
  ''' @ivar: remove an offline host after this time in [sec] (Default: 300 sec). '''
  
  def __init__(self, mcast_port, mcast_group, monitor_port):
    '''
    Initialize method for the Discoverer class
    @param mcast_port: The port used to publish and receive the multicast messages.
    @type mcast_port:  int
    @param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.
    @type mcast_group:  str
    @param monitor_port: The port of the RPC Server, used to get more information about the ROS master.
    @type monitor_port:  int
    '''
    threading.Thread.__init__(self)
    self.do_finish = False
    self.__lock = threading.RLock()
    # the list with all ROS master neighbors
    self.masters = dict() # (ip, DiscoveredMaster)
    
    self.static_hosts = []
    if rospy.has_param('~rosmaster_hz'):
      Discoverer.ROSMASTER_HZ = rospy.get_param('~rosmaster_hz')
    if rospy.has_param('~heartbeat_hz'):
      Discoverer.HEARTBEAT_HZ = rospy.get_param('~heartbeat_hz')
    if rospy.has_param('~measurement_intervals'):
      Discoverer.MEASUREMENT_INTERVALS = rospy.get_param('~measurement_intervals')
    if rospy.has_param('~timeout_factor'):
      Discoverer.TIMEOUT_FACTOR = rospy.get_param('~timeout_factor')
    if rospy.has_param('~remove_after'):
      Discoverer.REMOVE_AFTER = rospy.get_param('~remove_after')
    if rospy.has_param('~static_hosts'):
      self.static_hosts[len(self.static_hosts):] = rospy.get_param('~static_hosts')

    rospy.loginfo("Static hosts: " + str(self.static_hosts))

    self.current_check_hz = Discoverer.HEARTBEAT_HZ
    # initialize the ROS publishers
    self.pubchanges = rospy.Publisher("~changes", MasterState)
    self.pubstats = rospy.Publisher("~linkstats", LinkStatesStamped)
    # initialize the ROS services
    rospy.Service('~list_masters', DiscoverMasters, self.rosservice_list_masters)

    # test the reachability of the ROS master 
    local_addr = roslib.network.get_local_address()
    if (local_addr in ['localhost', '127.0.0.1']):
      sys.exit("'%s' is not reachable for other systems. Change the ROS_MASTER_URI!", local_addr)
    
    self.mcast_port = mcast_port
    self.mcast_group = mcast_group
    rospy.loginfo("Start broadcasting at ('%s', %d)", mcast_group, mcast_port)
    self._init_mcast_socket(True)
#    # create the multicast socket and join the multicast group
#    self.msocket = msocket = McastSocket(mcast_port, mcast_group)
##    msocket.settimeout(3.0)
#    if not msocket.hasEnabledMulticastIface():
#      sys.exit("No enabled multicast interfaces available!\nAdd multicast support e.g. sudo ifconfig eth0 multicast")
#
    # create a thread to handle the received multicast messages
    self._recvThread = threading.Thread(target = self.recv_loop)
    self._recvThread.setDaemon(True)
    self._recvThread.start()
    
    # create a thread to monitor the ROS master state
    self.master_monitor = MasterMonitor(monitor_port)
    self._masterMonitorThread = threading.Thread(target = self.checkROSMaster_loop)
    self._masterMonitorThread.setDaemon(True)
    self._masterMonitorThread.start()

    # create a timer monitor the offline ROS master and calculate the link qualities
    try:
      self._statsTimer = threading.Timer(1, self.timed_stats_calculation)
      self._statsTimer.start()
    except:
      rospy.logwarn("ROS Timer is not available! Statistic calculation and timeouts are deactivated!")
    # set the callback to finish all running threads
    rospy.on_shutdown(self.finish)

  def _init_mcast_socket(self, doexit_on_error=False):
    rospy.loginfo("Init multicast socket")
    # create the multicast socket and join the multicast group
    self.msocket = msocket = McastSocket(self.mcast_port, self.mcast_group)
#    msocket.settimeout(3.0)
    if not msocket.hasEnabledMulticastIface() and doexit_on_error:
      sys.exit("No enabled multicast interfaces available!\nAdd multicast support e.g. sudo ifconfig eth0 multicast")

  def finish(self, *arg):
    '''
    Callback called on exit of the ros node to publish the empty list of 
    ROSMasters.
    '''
    # publish all master as removed
    self.__lock.acquire(True)
    # tell other loops to finish
    self.do_finish = True
    # finish the RPC server and timer
    self.master_monitor.shutdown()
    for (k, v) in self.masters.iteritems():
      if not v.mastername is None:
        self.publish_masterstate(MasterState(MasterState.STATE_REMOVED, 
                                       ROSMaster(str(v.mastername), 
                                                 v.masteruri, 
                                                 v.timestamp, 
                                                 v.online, 
                                                 v.discoverername, 
                                                 v.monitoruri)))
    self.__lock.release()
    try:
      self._statsTimer.cancel()
    except:
      pass

  def run(self):
    '''
    The run method is used for periodic send small multicast messages. This 
    messages simulates the heartbeat and are used to detect other running
    nodes associated with ROS master.
    '''
    while (not rospy.is_shutdown()) and not self.do_finish:
      if not self.master_monitor.getMasteruri() is None:
        t = 0
        if not self.master_monitor.getCurrentState() is None:
          t = self.master_monitor.getCurrentState().timestamp
        msg = struct.pack(Discoverer.HEARTBEAT_FMT,'R', Discoverer.VERSION, int(Discoverer.HEARTBEAT_HZ*10), int(t), int((t-(int(t))) * 1000000000), self.master_monitor.rpcport)
        try:
          self.msocket.send2group(msg)
          for a in self.static_hosts:
            try:
              self.msocket.send2addr(msg, a)
            except socket.gaierror as e:
              rospy.logwarn("send to static host: " + str(a) + " failed: " + str(e))
        except Exception as e:
          rospy.logwarn(e)
          self._init_mcast_socket()
      time.sleep(1.0/Discoverer.HEARTBEAT_HZ)
    msg = struct.pack(Discoverer.HEARTBEAT_FMT,'R', Discoverer.VERSION, int(Discoverer.HEARTBEAT_HZ*10), -1, -1, self.master_monitor.rpcport)
    self.msocket.send2group(msg)
    for a in self.static_hosts:
      rospy.loginfo("send Discoverer.HEARTBEAT_FMT to static host: " + str(a))
      self.msocket.send2addr(msg, a)
    self.msocket.close()

  def checkROSMaster_loop(self):
    '''
    The method test periodically the state of the ROS master. The new state will
    be published as heartbeat messages.
    '''
    import os
    try_count = 0
    while (not rospy.is_shutdown()) and not self.do_finish:
      try:
        cputimes = os.times()
        cputime_init = cputimes[0] + cputimes[1]
        if self.master_monitor.checkState():
          # publish the new state ?
          pass
        # adapt the check rate to the CPU usage time
        cputimes = os.times()
        cputime = cputimes[0] + cputimes[1] - cputime_init
        if self.current_check_hz*cputime > 0.20:
          self.current_check_hz = float(self.current_check_hz)/2.0
        elif self.current_check_hz*cputime < 0.10 and self.current_check_hz < Discoverer.HEARTBEAT_HZ:
          self.current_check_hz = float(self.current_check_hz)*2.0
#        print "self.current_check_hz:", self.current_check_hz
        try_count = 0
      except MasterConnectionException, e:
        try_count = try_count + 1
        if try_count == 5:
          rospy.logerr("Communication with ROS Master failed: %s", e)
#          rospy.signal_shutdown("ROS Master not reachable")
#          time.sleep(3)
      # remove offline hosts
      self.__lock.acquire(True)
      current_time = time.time()
      to_remove = []
      for (k, v) in self.masters.iteritems():
        if Discoverer.REMOVE_AFTER > 0 and current_time - v.last_heartbeat_ts > Discoverer.REMOVE_AFTER:
          to_remove.append(k)
          if not v.mastername is None:
            self.publish_masterstate(MasterState(MasterState.STATE_REMOVED, 
                                           ROSMaster(str(v.mastername), 
                                                     v.masteruri, 
                                                     v.timestamp, 
                                                     v.online, 
                                                     v.discoverername, 
                                                     v.monitoruri)))
      for r in to_remove:
        del self.masters[r]
      self.__lock.release()
#      print "update rate", self.current_check_hz
      time.sleep(1.0/self.current_check_hz)

  def recv_loop(self):
    '''
    This method handles the received multicast messages.
    '''
    while self.msocket and (not rospy.is_shutdown()) and not self.do_finish:
      try:
        (msg, address) = self.msocket.recvfrom(1024)
      except socket.timeout:
#        rospy.logwarn("TIMOUT ignored")
        pass
      except socket.error:
        import traceback
        rospy.logwarn("socket error: %s", traceback.format_exc())
      else:
        try:
          (version, msg_tuple) = self.msg2masterState(msg)
          if (version == Discoverer.VERSION):
            (r, version, rate, secs, nsecs, monitor_port) = msg_tuple
            master_key = (address, monitor_port)
            # remove master if sec and nsec are -1
            if secs == -1:
              self.__lock.acquire(True)
              if self.masters.has_key(master_key):
                master = self.masters[master_key]
                if not master.mastername is None:
                  self.publish_masterstate(MasterState(MasterState.STATE_REMOVED, 
                                                 ROSMaster(str(master.mastername), 
                                                           master.masteruri, 
                                                           master.timestamp, 
                                                           False, 
                                                           master.discoverername, 
                                                           master.monitoruri)))
                del self.masters[master_key]
              self.__lock.release()
            # update the timestamp of existing master
            elif self.masters.has_key(master_key):
              self.__lock.acquire(True)
              self.masters[master_key].addHeartbeat(float(secs)+float(nsecs)/1000000000.0, float(rate)/10.0)
              self.__lock.release()
            # or create a new master
            else:
#                print "create new masterstate", ''.join(['http://', address[0],':',str(monitor_port)])
              self.__lock.acquire(True)
              self.masters[master_key] = DiscoveredMaster(monitoruri=''.join(['http://', address[0],':',str(monitor_port)]), 
                                                          heartbeat_rate=float(rate)/10.0,
                                                          timestamp=float(secs)+float(nsecs)/1000000000.0,
                                                          callback_master_state=self.publish_masterstate)
              self.__lock.release()
        except Exception, e:
          rospy.logwarn("Error while decode message: %s", str(e))

  @classmethod
  def msg2masterState(cls, msg):
    '''
    @return: parses the hearbeat message and return a tuple of
            version and values corresponding with current version of message.
            @see L{Discoverer.HEARTBEAT_FMT}
    @raise Exception on invalid message
    @rtype: C{(unsigned char, tuple corresponding to L{Discoverer.HEARTBEAT_FMT})}
    '''
    if len(msg) > 2:
      (r,) = struct.unpack('c', msg[0])
      (version,) = struct.unpack('B', msg[1])
      if (version == Discoverer.VERSION):
        if (r == 'R'):
          if len(msg) == struct.calcsize(Discoverer.HEARTBEAT_FMT):
            return (version, struct.unpack(Discoverer.HEARTBEAT_FMT, msg))
        else:
          raise Exception(' '.join(["wrong initial discovery message char", str(r), "received from", str(address)]))
      elif (version > Discoverer.VERSION):
        raise Exception(' '.join(["newer heartbeat version", str(version), "(own:", str(Discoverer.VERSION), ") detected, please update your master_discovery"]))
      elif (version < Discoverer.VERSION):
        raise Exception(' '.join(["old heartbeat version", str(version), "detected (current:", str(Discoverer.VERSION),"), please update master_discovery on", str(address)]))
      else:
        raise Exception(' '.join(["heartbeat version", str(Discoverer.VERSION), "expected, received:", str(version)]))
    raise Exception("massage is to small")

  def timed_stats_calculation(self):
    '''
    This method will be called by a timer and has two jobs:
     1. set the masters offline, if no heartbeat messages are received a long time
     2. calculate the quality of known links
    @see: L{float}
    '''
    result = LinkStatesStamped()
    current_time = time.time()
    result.header.stamp.secs = int(current_time)
    result.header.stamp.nsecs = int((current_time - result.header.stamp.secs) * 1000000000)
    try:
      self.__lock.acquire(True)
  #    self.__lock.release()
      for (k, v) in self.masters.iteritems():
        quality = -1.0
   #     self.__lock.acquire(True)
        if not (v.mastername is None):
          rate = v.heartbeat_rate
          measurement_duration = Discoverer.MEASUREMENT_INTERVALS
          if rate < 1.:
            measurement_duration = Discoverer.MEASUREMENT_INTERVALS / rate
          # remove all heartbeats, which are to old
          ts_oldest = current_time - measurement_duration
          removed_ts = v.removeHeartbeats(ts_oldest)
          # sets the master offline if the last received heartbeat is to old
          if current_time - v.last_heartbeat_ts > (measurement_duration * Discoverer.TIMEOUT_FACTOR):
            v.setOffline()
          # calculate the quality for inly online masters
          if v.online:
            beats_count = len(v.heartbeats)
            expected_count = rate * measurement_duration
    #        print "beats_count:",beats_count, ", expected_count:",expected_count
            if expected_count > 0:
              quality = float(beats_count) / float(expected_count) * 100.0
              if quality > 100.0:
                quality = 100.0
            result.links.append(LinkState(v.mastername, quality))
    finally:
      self.__lock.release()
    #publish the results
    self.publish_stats(result)
    try:
      if not rospy.is_shutdown():
        self._statsTimer = threading.Timer(1, self.timed_stats_calculation)
        self._statsTimer.start()
    except:
      pass

  def publish_masterstate(self, master_state):
    '''
    Publishes the given state to the ROS network. This method is thread safe.
    @param master_state: the master state to publish
    @type master_state:  L{master_discovery_fkie.MasterState}
    '''
    if self.__lock.acquire(True):
      try:
        self.pubchanges.publish(master_state)
      except:
        import traceback
        traceback.print_exc()
      finally:
        self.__lock.release()
    

  def publish_stats(self, stats):
    '''
    Publishes the link quality states to the ROS network.This method is thread safe.
    @param stats: the link quality states to publish
    @type stats:  L{master_discovery_fkie.LinkStatesStamped}
    '''
    if self.__lock.acquire(True):
      try:
        self.pubstats.publish(stats)
      except:
        import traceback
        traceback.print_exc()
      finally:
        self.__lock.release()

  def rosservice_list_masters(self, req):
    '''
    Callback for the ROS service to get the current list of the known ROS masters.
    '''
    masters = list()
    self.__lock.acquire(True)
    try:
      for (k, v) in self.masters.iteritems():
        if not v.mastername is None:
          masters.append(ROSMaster(str(v.mastername), 
                                   v.masteruri, 
                                   v.timestamp, 
                                   v.online, 
                                   v.discoverername, 
                                   v.monitoruri))
    except:
      import traceback
      traceback.print_exc()
    finally:
      self.__lock.release()
      return DiscoverMastersResponse(masters)

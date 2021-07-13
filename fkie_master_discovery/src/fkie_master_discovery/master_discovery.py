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

try:
    import queue
except ImportError:
    import Queue as queue  # python 2 compatibility
import errno
import rospy
import socket
import std_srvs.srv
import struct
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

from rosgraph.network import get_local_addresses, get_local_address
from .common import get_hostname
from .master_monitor import MasterMonitor, MasterConnectionException
from .udp import DiscoverSocket, QueueReceiveItem, SEND_ERRORS


try:  # to avoid the problems with autodoc on ros.org/wiki site
    from fkie_multimaster_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster  # , SyncMasterInfo, SyncTopicInfo
    from fkie_multimaster_msgs.srv import DiscoverMasters, DiscoverMastersResponse
except:
    pass


class DiscoveredMaster(object):
    '''
    The class stores all information about the remote ROS master and the all
    received heartbeat messages of the remote node. On first contact a theaded
    connection to remote discoverer will be established to get additional
    information about the ROS master.

    :param monitoruri: The URI of the remote RPC server, which moniter the ROS master

    :type monitoruri:  str

    :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

    :type heartbeat_rate:  float (Default: `1.``)

    :param timestamp: The timestamp of the state of the remoter ROS master

    :type timestamp:  float (Default: ``0``)

    :param timestamp_local: The timestamp of the state of the remoter ROS master,
                            without the changes maked while a synchronization.

    :type timestamp_local:  float (Default: ``0``)

    :param callback_master_state: the callback method to publish the changes of the ROS masters

    :type callback_master_state: `fkie_master_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/MasterState.html>`_}  (Default: ``None``)
    '''

    MIN_HZ_FOR_QUALILTY = 0.3

    ERR_RESOLVE_NAME = 1
    ERR_SOCKET = 2

    def __init__(self, monitoruri, is_local=False, heartbeat_rate=1.,
                 timestamp=0.0, timestamp_local=0.0, callback_master_state=None):
        '''
        Initialize method for the DiscoveredMaster class.

        :param monitoruri: The URI of the remote RPC server, which moniter the ROS master

        :type monitoruri:  str

        :param is_local: is the URI of the remote RPC server local or not

        :type is_local:  bool

        :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

        :type heartbeat_rate:  float (Default: `1.``)

        :param timestamp: The timestamp of the state of the remoter ROS master

        :type timestamp:  float (Default: ``0``)

        :param timestamp_local: The timestamp of the state of the remoter ROS master, without the changes maked while a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param callback_master_state: the callback method to publish the changes of the ROS masters

        :type callback_master_state: `fkie_master_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/MasterState.html>`_}  (Default: ``None``)
        '''
        self.__lock = threading.RLock()
        self.masteruri = None
        self.mastername = None
        self.timestamp = timestamp
        self.timestamp_local = timestamp_local
        self.discoverername = None
        self.monitoruri = monitoruri
        self.is_local = is_local
        self.heartbeat_rate = heartbeat_rate
        self.heartbeats = list()
        self.requests = list()
        self.last_heartbeat_ts = time.time()
        self.creation_ts = time.time()
        self.online = False
        self.callback_master_state = callback_master_state
        self.ts_last_request = 0
        self._errors = dict()  # ERR_*, msg
        self.monitor_hostname = get_hostname(monitoruri)
        self.master_hostname = None
        self.masteruriaddr = None
        self._on_finish = False
        # create a thread to retrieve additional information about the remote ROS master
        self._get_into_timer = threading.Timer(0.1, self._get_info_threaded)
        self._get_into_timer.start()

    def finish(self):
        self._on_finish = True
        try:
            self._get_into_timer.cancel()
        except Exception:
            pass

    def add_heartbeat(self, timestamp, timestamp_local, rate):
        '''
        Adds a new heartbeat measurement. If it is a new timestamp a ROS message
        about the change of this ROS master will be published into ROS network.

        :param timestamp: The new timestamp of the ROS master state

        :type timestamp:  float

        :param timestamp_local: The timestamp of the state of the remoter ROS
                          master, without the changes maked while a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param rate: The remote rate, which is used to send the heartbeat messages.
                     If the rate is zero the heartbeat is ignored.

        :type rate:  float

        :return: ``True`` on changes

        :rtype: bool
        '''
        result = False
        cur_time = time.time()
        self.last_heartbeat_ts = cur_time
        self.ts_last_request = 0
        self.requests = list()
        # publish new master state, if the timestamp is changed
        if (self.timestamp != timestamp or not self.online or self.timestamp_local != timestamp_local):
            self.timestamp = timestamp
            self.timestamp_local = timestamp_local
            if self.masteruri is not None:
                # set the state to 'online'
                self.online = True
                if self.callback_master_state is not None:
                    self.callback_master_state(MasterState(MasterState.STATE_CHANGED,
                                                           ROSMaster(str(self.mastername),
                                                                     self.masteruri,
                                                                     rospy.Time(self.timestamp),
                                                                     rospy.Time(self.timestamp_local),
                                                                     self.online,
                                                                     self.discoverername,
                                                                     self.monitoruri)))
                    result = True
        if rate >= DiscoveredMaster.MIN_HZ_FOR_QUALILTY:
            # reset the list, if the heartbeat is changed
            if self.heartbeat_rate != rate:
                self.heartbeat_rate = rate
                self.heartbeats = list()
            self.heartbeats.append(cur_time)
        return result

    def add_request(self, timestamp):
        '''
        Stores the count of requests. This count is used to determine offline state
        of remote master.

        :param timestamp: time the request is sent.

        :type timestamp:  float
        '''
        self.ts_last_request = timestamp
        self.requests.append(timestamp)
        rospy.logdebug("Unanswered requests [%d] for %s: %s" % (len(self.requests), str(self.masteruri), str(self.requests)))

    def requests_count(self):
        return len(self.requests)

    def remove_heartbeats(self, timestamp):
        '''
        Removes all heartbeat measurements, which are older as the given timestamp.

        :param timestamp: heartbeats older this timestamp will be removed.

        :type timestamp:  float

        :return: the count of removed heartbeats

        :rtype: int
        '''
        do_remove = True
        # remove the requests
        while do_remove:
            if len(self.requests) > 0 and self.requests[0] < timestamp:
                del self.requests[0]
            else:
                do_remove = False
        do_remove = True
        removed = 0
        while do_remove:
            if len(self.heartbeats) > 0 and self.heartbeats[0] < timestamp:
                del self.heartbeats[0]
                removed = removed + 1
            else:
                do_remove = False
        return removed

    def set_offline(self):
        '''
        Sets this master to offline and publish the new state to the ROS network.
        '''
        if self.callback_master_state is not None and self.online:
            rospy.loginfo('Set host to offline: %s' % self.mastername)
            self.callback_master_state(MasterState(MasterState.STATE_CHANGED,
                                                   ROSMaster(str(self.mastername),
                                                             self.masteruri,
                                                             rospy.Time(self.timestamp),
                                                             rospy.Time(self.timestamp_local),
                                                             False,
                                                             self.discoverername,
                                                             self.monitoruri)))
        self.online = False

    def get_quality(self, interval=5, offline_after=1.4):
        '''
        Calculates the link quality to this master.
        '''
        quality = -1.0
        if self.mastername is not None and self.heartbeat_rate >= self.MIN_HZ_FOR_QUALILTY:
            current_time = time.time()
            measurement_duration = interval
            if self.heartbeat_rate < 1.:
                measurement_duration = measurement_duration / self.heartbeat_rate
            # reduce the mesurement duration on start of the master
            if measurement_duration > current_time - self.creation_ts:
                measurement_duration = current_time - self.creation_ts
            # remove all heartbeats, which are to old
            ts_oldest = current_time - measurement_duration
            self.remove_heartbeats(ts_oldest)
            # sets the master offline if the last received heartbeat is to old
            if current_time - self.last_heartbeat_ts > (measurement_duration * offline_after):
                self.set_offline()
            # calculate the quality for online masters only
            if self.online:
                beats_count = len(self.heartbeats)
                expected_count = int(self.heartbeat_rate * measurement_duration + len(self.requests))
                if expected_count > 0:
                    quality = float(beats_count) / float(expected_count) * 100.0
                    if quality > 100.0:
                        quality = 100.0
        return quality

    @property
    def errors(self):
        '''
        Copies the errors and returns as dictionary.
        :return: a dictionry of error type (ERR_*) and a message
        '''
        result = dict()
        with self.__lock:
            for key, val in self._errors.items():
                result[key] = val
        return result

    def _add_error(self, error_id, msg):
        with self.__lock:
            if id not in self._errors:
                self._errors[error_id] = msg

    def _del_error(self, error_id):
        try:
            with self.__lock:
                del self._errors[error_id]
        except:
            pass

    def __start_get_info_timer(self, timetosleep):
        self._get_into_timer = threading.Timer(timetosleep, self._get_info_threaded)
        self._get_into_timer.start()

    def _get_info_threaded(self):
        thread = threading.Thread(target=self._retrieve_masterinfo)
        thread.setDaemon(True)
        thread.start()

    def _retrieve_masterinfo(self):
        '''
        Connects to the remote RPC server of the discoverer node and gets the
        information about the Master URI, name of the service, and other. The
        ``getMasterInfo()`` method will be used. On problems the connection will be
        reestablished until the information will be get successful.
        '''
        if self.monitoruri is not None and not self._on_finish:
            timetosleep = 5.
            if not rospy.is_shutdown() and self.mastername is None:
                try:
                    rospy.logdebug("Get additional connection info from %s" % self.monitoruri)
                    remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
                    socket.setdefaulttimeout(10)
                    timestamp, masteruri, mastername, nodename, monitoruri = remote_monitor.masterContacts()
                    self._del_error(self.ERR_SOCKET)
                    rospy.logdebug("Got [%s, %s, %s, %s] from %s" % (timestamp, masteruri, mastername, nodename, monitoruri))
                    timetosleep = 0
                except socket.error as errobj:
                    msg = "can't retrieve connection information using XMLRPC from [%s], socket error: %s" % (self.monitoruri, str(errobj))
                    rospy.logwarn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                    if errobj.errno in [errno.EHOSTUNREACH]:
                        timetosleep = 30
                except:
                    msg = "can't retrieve connection information using XMLRPC from [%s]: %s" % (self.monitoruri, traceback.format_exc())
                    rospy.logwarn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                else:
                    if float(timestamp) != 0:
                        self.masteruri = masteruri
                        self.mastername = mastername
                        self.discoverername = nodename
#            self.monitoruri = monitoruri
                        self.timestamp = float(timestamp)
                        self.online = True
                        # resolve the masteruri. Print an error if not reachable
                        try:
                            self.master_hostname = get_hostname(self.masteruri)
                            self.masteruriaddr = socket.gethostbyname(self.master_hostname)
                            self._del_error(self.ERR_RESOLVE_NAME)
                        except socket.gaierror:
                            msg = "Master discovered with not known hostname ROS_MASTER_URI:='%s'. Fix your network settings!" % str(self.masteruri)
                            rospy.logwarn(msg)
                            self._add_error(self.ERR_RESOLVE_NAME, msg)
                        except:
                            msg = "resolve error [%s]: %s" % (self.monitoruri, traceback.format_exc())
                            rospy.logwarn(msg)
                            self._add_error(self.ERR_SOCKET, msg)
                        else:
                            # publish new node
                            if self.callback_master_state is not None:
                                rospy.loginfo("Added master with ROS_MASTER_URI=%s" % (self.masteruri))
                                self.callback_master_state(MasterState(MasterState.STATE_NEW,
                                                                       ROSMaster(str(self.mastername),
                                                                                 self.masteruri,
                                                                                 rospy.Time(self.timestamp),
                                                                                 rospy.Time(self.timestamp),
                                                                                 self.online,
                                                                                 self.discoverername,
                                                                                 self.monitoruri)))
                                timetosleep = 0
                            else:
                                msg = "calback is None, should not happen...remove master %s" % self.monitoruri
                                rospy.logwarn(msg)
                                self._add_error(self.ERR_SOCKET, msg)
                    else:
                        msg = "Got timestamp=0 from %s, retry... " % self.monitoruri
                        rospy.logwarn(msg)
                        self._add_error(self.ERR_SOCKET, msg)
                finally:
                    socket.setdefaulttimeout(None)
                if not self._on_finish and timetosleep > 0:
                    self.__start_get_info_timer(timetosleep)


class Discoverer(object):
    '''
    The class to publish the current state of the ROS master.

    Discovering is done by heartbeats:
      Each master discovery node sends to a multicast group periodically messages
      with current state. If the frequency is less than 0.3 the detected changes
      on ROS master are published immediately.
      The current state is described by timestamp of last change. The frequency of
      heartbeats can be changed by `~heartbeat_hz` parameter.

      If heartbeats are disabled (`~heartbeat_hz` is zero) each master discovery
      node sends on start three notification messages and requests.

      If for a host no more heartbeat are received while `ACTIVE_REQUEST_AFTER (60 sec)`
      a request to this host will be sent as an unicast message. After five
      unanswered requests the host state will be changed to `offline`.
      After `REMOVE_AFTER (300 sec)` the host will be removed.

    :param mcast_port: The port used to publish and receive the multicast messages.

    :type mcast_port:  int

    :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

    :type mcast_group:  str

    :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

    :type monitor_port:  int
    '''

    VERSION = 2
    '''the version of the packet format described by ``HEARTBEAT_FMT``

      :Version 1: 'cBBiiH'

      ::

        one character 'R'
        unsigned char: version of the heartbeat message
        unsigned char: rate of the heartbeat message in HZ*10. Maximal rate: 25.5 Hz -> value 255
        int: secs of the ROS Master state
        int: nsecs of the ROS Master state
        unsigned short: the port number of the RPC Server of the remote ROS-Core monitor

      :Version 2: 'cBBiiHii'

      ::

        ``Version 1``
        int: secs of the ROS Master state (only local changes). Changes while sync will be ignored.
        int: nsecs of the ROS Master state (only local changes). Changes while sync will be ignored.

      :Version 3: 'cBBiiHii'

      ::

        ``Version 2``
        if the timestamp of ROS Master state is zero, the reply as unicast
        message will be send to the sender.

  '''
    HEARTBEAT_FMT = 'cBBiiHii'
    ''' packet format description, see: http://docs.python.org/library/struct.html '''
    HEARTBEAT_HZ = 0.02
    ''' the send rate of the heartbeat packets in hz. Zero disables the heartbeats. (Default: 0.02 Hz)
      Only values between 0.1 and 25.5 are used to detemine the link quality.
  '''
    MEASUREMENT_INTERVALS = 5
    ''' the count of intervals (1 sec) used for a quality calculation. If
      `HEARTBEAT_HZ` is smaller then 1, `MEASUREMENT_INTERVALS` will be divided
      by `HEARTBEAT_HZ` value.
      (Default: 5 sec are used to determine the link qaulity)'''
    TIMEOUT_FACTOR = 10
    ''' the timeout is defined by calculated measurement duration multiplied by `TIMEOUT_FAKTOR`. '''
    ROSMASTER_HZ = 1
    ''' the test rate of ROS master state in Hz (Default: 1 Hz). '''
    REMOVE_AFTER = 300
    ''' remove an offline host after this time in [sec] (Default: 300 sec). '''

    ACTIVE_REQUEST_AFTER = 60
    ''' send an update request, if after this time no heartbeats are received [sec] (Default: 60 sec). '''

    INIT_NOTIFICATION_COUNT = 3
    ''' the count of heartbeats and update request to send at the start (Default: 3 sec).
      It will be send with 1Hz. Only used if `HEARTBEAT_HZ` is zero. '''

    OFFLINE_AFTER_REQUEST_COUNT = 5
    ''' After this unanswered count of requests for update the remote master is set
      to offline state (Default: 5 sec).
      The requests are send after `ACTIVE_REQUEST_AFTER` with `ROSMASTER_HZ`. '''

    CHANGE_NOTIFICATION_COUNT = 3
    ''' After the ROS master was changed the new state will be sent for
      `CHANGE_NOTIFICATION_COUNT` times (Default: 3 sec). The new state will be
      sent with `ROSMASTER_HZ` and only if `HEARTBEAT_HZ` is zero. '''

    NETPACKET_SIZE = 68

    def __init__(self, mcast_port, mcast_group, monitor_port, rpc_addr=''):
        '''
        Initialize method for the Discoverer class

        :param mcast_port: The port used to publish and receive the multicast messages.

        :type mcast_port:  int

        :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

        :type mcast_group:  str

        :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

        :type monitor_port:  int
        '''
#    threading.Thread.__init__(self)
        self.do_finish = False
        self._services_initialized = False
        self.__lock = threading.RLock()
        # the list with all ROS master neighbors
        self.masters = dict()  # (ip, DiscoveredMaster)
        # this parameter stores the state of the remote nodes. If the state is changed
        # the cache for contacts of remote nodes will be cleared.
        self._changed = False
        self._last_datetime = time.time()
        self.ROSMASTER_HZ = rospy.get_param('~rosmaster_hz', Discoverer.ROSMASTER_HZ)
        self.HEARTBEAT_HZ = rospy.get_param('~heartbeat_hz', Discoverer.HEARTBEAT_HZ)
        self.MEASUREMENT_INTERVALS = rospy.get_param('~measurement_intervals', Discoverer.MEASUREMENT_INTERVALS)
        self.TIMEOUT_FACTOR = rospy.get_param('~timeout_factor', Discoverer.TIMEOUT_FACTOR)
        self.REMOVE_AFTER = rospy.get_param('~remove_after', Discoverer.REMOVE_AFTER)
        self.ACTIVE_REQUEST_AFTER = rospy.get_param('~active_request_after', Discoverer.ACTIVE_REQUEST_AFTER)
        if self.ACTIVE_REQUEST_AFTER <= 0:
            rospy.logwarn("active_request_after [sec]: %s <= 0 set to 60" % self.ACTIVE_REQUEST_AFTER)
            self.ACTIVE_REQUEST_AFTER = 60
        self.robots = rospy.get_param('~robot_hosts', [])
        self.CHANGE_NOTIFICATION_COUNT = rospy.get_param('~change_notification_count', Discoverer.CHANGE_NOTIFICATION_COUNT)
        self._current_change_notification_count = 0
        self._send_mcast = rospy.get_param('~send_mcast', True)
        self._listen_mcast = rospy.get_param('~listen_mcast', True)
        # for cases with more then one master_discovery on the same host and
        # heartbeat rate is less then 0.1. In this case we have to send a multicast
        # request reply, because we are bind to the same port. Unicast replies are
        # not forward to the same port only once.
        self._addresses = dict()  # {address : (int) ocurres}
        # some parameter checks and info outputs
        if not self._send_mcast:
            rospy.logwarn("Send multicast is disabled.")
        if not self._listen_mcast:
            rospy.logwarn("Listen to multicast is disabled.")
        if not self._send_mcast and not self.robots:
            rospy.logwarn("This master_discovery is invisible because it sends no heart beat messages! Set ~send_mcast/~listen_mcast to True or add hosts to ~robot_hosts.")
        if not self._listen_mcast or not self._send_mcast:
            self.robots.append('localhost')
        self.robots = list(set(self.robots))
        rospy.loginfo("Check the ROS Master[Hz]: " + str(self.ROSMASTER_HZ))
        if self.HEARTBEAT_HZ <= 0.:
            rospy.logwarn("Heart beat [Hz]: %s is increased to 0.02" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 0.02
        if self.HEARTBEAT_HZ > 25.5:
            rospy.logwarn("Heart beat [Hz]: %s is decreased to 25.5" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 25.5
        else:
            rospy.loginfo("Heart beat [Hz]: %s" % (self.HEARTBEAT_HZ))
        rospy.loginfo("Active request after [sec]: %s" % self.ACTIVE_REQUEST_AFTER)
        rospy.loginfo("Remove after [sec]: %s" % self.REMOVE_AFTER)
        if self.REMOVE_AFTER <= self.ACTIVE_REQUEST_AFTER:
            rospy.logwarn("'Active request after' should be less than 'remove after' to avoid removing masters from list!")
        rospy.loginfo("Robot hosts: " + str(self.robots))
        if self.HEARTBEAT_HZ > 0.:
            count_packets = len(self.robots) + (1 if self._send_mcast else 0)
            netload = self.HEARTBEAT_HZ * self.NETPACKET_SIZE * count_packets
            rospy.loginfo("Approx. mininum avg. network load: %.2f bytes/s" % netload)
        self.current_check_hz = self.ROSMASTER_HZ
        self.pubstats = rospy.Publisher("~linkstats", LinkStatesStamped, queue_size=1)
        # test the reachability of the ROS master
        local_addr = get_local_address()
        if (local_addr in ['localhost', '127.0.0.1']):
            rospy.logwarn("'%s' is not reachable for other systems. Change the ROS_MASTER_URI!" % local_addr)
        self.mcast_port = mcast_port
        self.mcast_group = mcast_group
        self._ts_received_mcast_request = 0
        # initialize the ROS publishers
        self.pubchanges = rospy.Publisher("~changes", MasterState, queue_size=10)
        # create a thread to monitor the ROS master state
        mgroup = DiscoverSocket.normalize_mgroup(mcast_group)
        is_ip6 = self._is_ipv6_group(mgroup)
        self.master_monitor = MasterMonitor(monitor_port, ipv6=is_ip6, rpc_addr=rpc_addr)
        # create timer to check for ros master changes
        self._timer_ros_changes = threading.Timer(0.1, self.checkROSMaster_loop)
        # init socket for discovering. Exit on errors!
        self._init_socket(True)
        # create a timer monitor the offline ROS master and calculate the link qualities
        self._timer_stats = threading.Timer(1, self.timed_stats_calculation)
        # create timer and paramter for heartbeat notifications
        self._init_notifications = 0
        # disable parameter, if HEARTBEAT_HZ is active (> zero)
        if self.HEARTBEAT_HZ > DiscoveredMaster.MIN_HZ_FOR_QUALILTY:
            # send init requests in mixed szenario: self._init_notifications = self.INIT_NOTIFICATION_COUNT
            self._current_change_notification_count = self.CHANGE_NOTIFICATION_COUNT
        self._timer_heartbeat = threading.Timer(1.0, self.send_heartbeat)
        # set the callback to finish all running threads
        rospy.on_shutdown(self.on_shutdown)
        self._recv_tread = threading.Thread(target=self._recv_loop_from_queue)

    def start(self):
        self._recv_tread.start()
        self._timer_ros_changes.start()
        self._timer_stats.start()
        self._timer_heartbeat.start()

    def _is_ipv6_group(self, addr):
        try:
            socket.inet_pton(socket.AF_INET6, addr)
            return True
        except:
            pass
        return False

    def _init_socket(self, doexit_on_error=False):
        # create discovery socket
        # if multicast messages are disabled only unicast socket is created
        # unicast socket is also created if ~interface is defined
        self.socket = DiscoverSocket(self.mcast_port, self.mcast_group, send_mcast=self._send_mcast, listen_mcast=self._listen_mcast)
        if self._send_mcast or self._listen_mcast:
            if not self.socket.hasEnabledMulticastIface() and doexit_on_error:
                sys.exit("No enabled multicast interfaces available!\nAdd multicast support e.g. sudo ifconfig eth0 multicast or disable multicast by settings 'send_mcast' and 'listen_mcast' to False.\nExit")
        # set callback for received UDP messages
#        self.socket.set_message_callback(self.recv_udp_msg)

    def _stop_timers(self):
        try:
            self._timer_ros_changes.cancel()
        except:
            pass
        try:
            self._timer_heartbeat.cancel()
        except:
            pass
        try:
            self._timer_stats.cancel()
        except:
            pass

    def on_shutdown(self, *arg):
        with self.__lock:
            # tell other loops to finish
            self.do_finish = True
            # finish the RPC server and timer
            self.master_monitor.shutdown()
            for (_, master) in self.masters.items():
                if master.mastername is not None:
                    self.publish_masterstate(MasterState(MasterState.STATE_REMOVED,
                                                         ROSMaster(str(master.mastername),
                                                                   master.masteruri,
                                                                   rospy.Time(master.timestamp),
                                                                   rospy.Time(master.timestamp_local),
                                                                   master.online,
                                                                   master.discoverername,
                                                                   master.monitoruri)))
                master.finish()
            # send notification that the master is going off
            msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                              int(self.HEARTBEAT_HZ * 10), -1, -1,
                              self.master_monitor.rpcport, -1, -1)
            self._publish_current_state(msg=msg)
            time.sleep(0.2)

    def finish(self):
        '''
        Callback called on exit of the ros node and publish the empty list of
        ROSMasters.
        '''
        # publish all master as removed
        rospy.logdebug("Finish master discovery")
        self._stop_timers()
        self.socket.close()
        self._killme_timer = threading.Timer(19., self._killme)
        self._killme_timer.setDaemon(True)
        self._killme_timer.start()

    def _killme(self):
        import os
        import signal
        os.kill(os.getpid(), signal.SIGKILL)

    def send_heartbeat(self, timer=True):
        '''
        Sends current state as heartbeat messages to defined multicast group. If the
        Discoverer.HEARTBEAT_HZ is greather then zero a timer will be started to
        send heartbeat messages periodically. This message will also send on start
        of the discoverer.
        '''
        with self.__lock:
            # publish the current state
            invalid_uri = (self.master_monitor.getMasteruri() is None)
            invalid_state = (self.master_monitor.getCurrentState() is None)
            if not (invalid_uri or invalid_state or rospy.is_shutdown() or self.do_finish):
                self._publish_current_state()
                # send update requests to group
                if timer and self._listen_mcast and self._init_notifications < self.INIT_NOTIFICATION_COUNT:
                    self._init_notifications += 1
                    rospy.logdebug("Send requests while init %d/%d" % (self._init_notifications, self.INIT_NOTIFICATION_COUNT))
                    self._request_state()
            if timer and not self.do_finish:
                if (self.HEARTBEAT_HZ > 0. or self._init_notifications < self.INIT_NOTIFICATION_COUNT):
                    sleeptime = 1.0 / self.HEARTBEAT_HZ if self.HEARTBEAT_HZ > 0. else 1.0
                    rospy.logdebug("Set timer to send heartbeat in %.2f sec" % sleeptime)
                    self._timer_heartbeat = threading.Timer(sleeptime, self.send_heartbeat)
                    self._timer_heartbeat.start()

    def _publish_current_state(self, address=None, msg=None):
        try:
            if self.socket.SOKET_ERRORS_NEEDS_RECONNECT:
                self._init_socket()
                time.sleep(3)
            if msg is None:
                msg = self._create_current_state_msg()
            if msg is not None:
                if address is not None:
                    rospy.logdebug('Send current state to addresses %s' % (address))
                    self.socket.send_queued(msg, address)
                elif self._send_mcast:
                    rospy.logdebug('Send current state to group %s:%s' % (self.mcast_group, self.mcast_port))
                    self.socket.send_queued(msg)
                    if self.robots:
                        rospy.logdebug('Send current state as unicast to all robot hosts %s' % self.robots)
                        self.socket.send_queued(msg, self.robots)
                else:
                    dests = list(set(list(self._addresses.keys()) + self.robots))
                    if dests:
                        rospy.logdebug('Send current state as unicast to all known hosts: %s' % dests)
                        self.socket.send_queued(msg, dests)
        except Exception as e:
            rospy.logwarn("Publish current state %s failed: %s" % (address, e))

    def _request_state(self, addresses=None, masters=[]):
        try:
            msg = self._create_request_update_msg()
            current_time = time.time()
            for master in masters:
                master.add_request(current_time)
            if addresses is not None:
                rospy.logdebug('Send a unicast request for update to %s' % addresses)
                self.socket.send_queued(msg, addresses)
            elif self._send_mcast:
                if current_time - self._ts_received_mcast_request > 1. / self.current_check_hz:
                    rospy.logdebug('Send request to mcast group %s:%s' % (self.mcast_group, self.mcast_port))
                    # do not send a multicast request if one was received in last time
                    self.socket.send_queued(msg)
                else:
                    rospy.logdebug('Skipped send request to mcast group %s:%s. Last send was %.2fsec ago,  allowed %.2f' % (self.mcast_group, self.mcast_port, current_time - self._ts_received_mcast_request, 1. / self.current_check_hz))
            if self.robots and addresses is None:
                rospy.logdebug('Send request as unicast to all robot hosts %s' % self.robots)
                self.socket.send_queued(msg, self.robots)
        except Exception as e:
            print(traceback.format_exc())
            rospy.logwarn("Send with addresses '%s' failed: %s" % (addresses, e))

    def _create_current_state_msg(self):
        t = 0
        local_t = 0
        if not self.master_monitor.getCurrentState() is None:
            t = self.master_monitor.getCurrentState().timestamp
            local_t = self.master_monitor.getCurrentState().timestamp_local
            return struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                               int(self.HEARTBEAT_HZ * 10),
                               int(t), int((t - (int(t))) * 1000000000),
                               self.master_monitor.rpcport,
                               int(local_t), int((local_t - (int(local_t))) * 1000000000))
        return None

    def _create_request_update_msg(self):
        version = Discoverer.VERSION if Discoverer.VERSION > 2 else 3
        msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', version,
                          int(self.HEARTBEAT_HZ * 10), 0, 0,
                          self.master_monitor.rpcport, 0, 0)
        return msg

    def checkROSMaster_loop(self):
        '''
        The method test periodically the state of the ROS master. The new state will
        be published as heartbeat messages.
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.checkState()`
        '''
        import os
        try_count = 0
        if (not rospy.is_shutdown()) and not self.do_finish:
            try:
                cputimes = os.times()
                cputime_init = cputimes[0] + cputimes[1]
                self.update_master_errors()
                if self.master_monitor.checkState(self._changed):
                    # publish the new state if frequetly publishing is disabled
                    if not self.do_finish and self.HEARTBEAT_HZ < DiscoveredMaster.MIN_HZ_FOR_QUALILTY:
                        self.send_heartbeat(timer=False)
                        self._current_change_notification_count = 0
                with self.__lock:
                    self._changed = False
                # repeat the last change for `CHANGE_NOTIFICATION_COUNT` times
                if 0 < self._current_change_notification_count < self.CHANGE_NOTIFICATION_COUNT:
                    self._current_change_notification_count += 1
                    self.send_heartbeat(timer=False)
                # adapt the check rate to the CPU usage time
                cputimes = os.times()
                cputime = cputimes[0] + cputimes[1] - cputime_init
                if self.current_check_hz * cputime > 0.20:
                    self.current_check_hz = float(self.current_check_hz) / 2.0
                elif self.current_check_hz * cputime < 0.10 and float(self.current_check_hz) * 2.0 < self.ROSMASTER_HZ:
                    self.current_check_hz = float(self.current_check_hz) * 2.0
                try_count = 0
            except MasterConnectionException as conn_err:
                try_count = try_count + 1
                if try_count == 5:
                    rospy.logerr("Communication with ROS Master failed: %s", conn_err)
            # remove offline hosts or request updates
            self._remove_offline_hosts()
            # setup timer for next ROS master state check
            if not rospy.is_shutdown():
                self._timer_ros_changes = threading.Timer(1.0 / self.current_check_hz, self.checkROSMaster_loop)
                self._timer_ros_changes.start()

    def _remove_offline_hosts(self):
        with self.__lock:
            current_time = time.time()
            to_remove = []
            multi_address = []
            for (k, v) in self.masters.items():
                ts_since_last_hb = current_time - v.last_heartbeat_ts
                ts_since_last_request = current_time - max(v.ts_last_request, v.last_heartbeat_ts)
                if self.REMOVE_AFTER > 0 and ts_since_last_hb > self.REMOVE_AFTER:
                    to_remove.append(k)
                    if v.mastername is not None:
                        self.publish_masterstate(MasterState(MasterState.STATE_REMOVED,
                                                             ROSMaster(str(v.mastername),
                                                                       v.masteruri,
                                                                       rospy.Time(v.timestamp),
                                                                       rospy.Time(v.timestamp_local),
                                                                       v.online,
                                                                       v.discoverername,
                                                                       v.monitoruri)))
                        v.finish()
                # request updates
                elif ts_since_last_request > self.ACTIVE_REQUEST_AFTER or (v.requests_count() > 0 and v.online):
                    if v.requests_count() >= self.OFFLINE_AFTER_REQUEST_COUNT:
                        v.set_offline()
                    if self._is_multi_address(k[0][0]):
                        # in case more then one master_discovery runs on the same address
                        # we send also a multicast message, because no two masters can bind to
                        # one unicast address
                        multi_address.append(v)
                    else:
                        self._request_state(k[0][0], [v])
            if multi_address:
                self._request_state(masters=multi_address)
            for r in to_remove:
                rospy.loginfo("Remove master discovery: http://%s:%s" % (r[0][0], r[1]))
                self._rem_address(r[0][0])
                del self.masters[r]

    def _recv_loop_from_queue(self):
        while not self.do_finish:
            try:
                recv_item = self.socket.receive_queue.get(timeout=1)
                self.recv_udp_msg(recv_item.msg, recv_item.sender_addr, recv_item.via)
            except queue.Empty:
                pass

    def recv_udp_msg(self, msg, address, via):
        '''
        This method handles the received udp messages.
        '''
        if not rospy.is_shutdown() and not self.do_finish:
            with self.__lock:
                self._check_timejump()
                try:
                    if len(msg) == 0:
                        return
                    (version, msg_tuple) = self.msg2masterState(msg, address)
                    if (version in [2, 3]):
                        add_to_list = False
                        (firstc, version, rate, secs, nsecs, monitor_port, secs_l, nsecs_l) = msg_tuple
                        if firstc != b'R':
                            # ignore the message. it does not start with 'R'
                            return
                        master_key = (address, monitor_port)
                        if version >= 3 and secs == 0 and nsecs == 0:
                            # is it a request to update the state
                            # send the current master state to the sender address
                            # TODO: add a filter, if multicast messages are disabled?
                            if self.master_monitor.getCurrentState() is not None:
                                if via == QueueReceiveItem.MULTICAST:
                                    rospy.logdebug("Received a multicast request for a state update from %s" % address[0])
                                    self._ts_received_mcast_request = time.time()
                                    if self._send_mcast:
                                        self._publish_current_state()
                                    self._publish_current_state(address[0])
                                elif via in [QueueReceiveItem.LOOPBACK, QueueReceiveItem.UNICAST]:
                                    rospy.logdebug("Received a request for a state update from %s" % (address[0]))
                                    self._publish_current_state(address[0])
                            add_to_list = master_key not in self.masters
                        elif secs == -1 or secs_l == -1:
                            # remove master if sec and nsec are -1
                            rospy.logdebug("Received a LEAVE heartbeat from %s via %s socket" % (master_key[0], via))
                            if master_key in self.masters:
                                master = self.masters[master_key]
                                if master.mastername is not None:
                                    # the contact info of the master is valied, publish the change
                                    state_remove = MasterState(MasterState.STATE_REMOVED,
                                                               ROSMaster(str(master.mastername),
                                                                         master.masteruri,
                                                                         rospy.Time(master.timestamp),
                                                                         rospy.Time(master.timestamp_local),
                                                                         False,
                                                                         master.discoverername,
                                                                         master.monitoruri))
                                    master.finish()
                                    self.publish_masterstate(state_remove)
                                rospy.loginfo("Remove master discovery: http://%s:%s, with ROS_MASTER_URI=%s" % (address[0], monitor_port, master.masteruri))
                                self._rem_address(address[0])
                                del self.masters[master_key]
                        elif master_key in self.masters:
                            # update the timestamp of existing master
                            rospy.logdebug("Received a heartbeat from %s via %s socket" % (master_key[0], via))
                            changed = self.masters[master_key].add_heartbeat(float(secs) + float(nsecs) / 1000000000.0, float(secs_l) + float(nsecs_l) / 1000000000.0, float(rate) / 10.0,)
                            if not self._changed:
                                self._changed = changed
                        else:
                            rospy.logdebug("Received a NEW heartbeat from %s via %s socket" % (master_key[0], via))
                            # or create a new master
                            add_to_list = True
                        if add_to_list:
                            rospy.loginfo("Detected master discovery: http://%s:%s" % (address[0], monitor_port))
                            self._add_address(address[0])
                            is_local = address[0].startswith('127.') or address[0] in get_local_addresses()
                            self.masters[master_key] = DiscoveredMaster(monitoruri=''.join(['http://', address[0], ':', str(monitor_port)]),
                                                                        is_local=is_local,
                                                                        heartbeat_rate=float(rate) / 10.0,
                                                                        timestamp=float(secs) + float(nsecs) / 1000000000.0,
                                                                        timestamp_local=float(secs_l) + float(nsecs_l) / 1000000000.0,
                                                                        callback_master_state=self.publish_masterstate)
                            if via == QueueReceiveItem.LOOPBACK:
                                self._publish_current_state(address[0])
                except Exception as e:
                    rospy.logwarn("Error while decode message: %s", str(e))

    def _check_timejump(self):
        if self._last_datetime > time.time():
            self._stop_timers()
            self.checkROSMaster_loop()
            self.send_heartbeat(True)
            self.timed_stats_calculation()
        self._last_datetime = time.time()

    def _is_multi_address(self, address):
        return address in self._addresses and self._addresses[address] > 1

    def _add_address(self, address):
        if address in self._addresses:
            self._addresses[address] += 1
        else:
            self._addresses[address] = 1

    def _rem_address(self, address):
        if address in self._addresses:
            self._addresses[address] -= 1
        if self._addresses[address] == 0:
            del self._addresses[address]

    @classmethod
    def msg2masterState(cls, msg, address):
        '''
        :return: parses the heartbeat message and return a tuple of
                version and values corresponding with current version of message.
                :mod:`fkie_master_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`

        :raise: Exception on invalid message

        :rtype: (``unsigned char``, tuple corresponding to :mod:`fkie_master_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`)
        '''
        if len(msg) > 2:
            (r,) = struct.unpack('c', msg[0:1])
            (version,) = struct.unpack('B', msg[1:2])
            if (version in [Discoverer.VERSION, 2, 3]):
                if (r == b'R'):
                    struct_size = struct.calcsize(Discoverer.HEARTBEAT_FMT)
                    if len(msg) == struct_size:
                        return (version, struct.unpack(Discoverer.HEARTBEAT_FMT, msg))
                    else:
                        raise Exception("wrong message size; expected %d, got %d from %s" % (struct_size, len(msg), address))
                else:
                    raise Exception("wrong initial discovery message char %s received from %s" % (r, address))
            elif (version > Discoverer.VERSION):
                raise Exception("newer heartbeat version %s (own: %s) from %s detected, please update your master_discovery" % (version, Discoverer.VERSION, address))
            elif (version < Discoverer.VERSION):
                raise Exception("old heartbeat version %s detected (current: %s), please update master_discovery on %s" % (version, Discoverer.VERSION, address))
            else:
                raise Exception("heartbeat version %s expected, received: %s" % (Discoverer.VERSION, version))
        raise Exception("message is too small")

    def timed_stats_calculation(self):
        '''
        This method will be called by a timer and has two jobs:
         1. set the masters offline, if no heartbeat messages are received a long time
         2. calculate the quality of known links
        '''
        result = LinkStatesStamped()
        result.header.stamp = rospy.Time.from_sec(time.time())
        with self.__lock:
            for (_, v) in self.masters.items():
                quality = v.get_quality(self.MEASUREMENT_INTERVALS, self.TIMEOUT_FACTOR)
                if not (v.mastername is None) and v.online:
                    result.links.append(LinkState(v.mastername, quality, rospy.Time.from_sec(v.last_heartbeat_ts)))
                if v.is_local:
                    result.header.frame_id = v.mastername
        # publish the results
        self.publish_stats(result)
        try:
            if not rospy.is_shutdown():
                self._timer_stats = threading.Timer(1, self.timed_stats_calculation)
                self._timer_stats.start()
        except:
            pass

    def publish_masterstate(self, master_state):
        '''
        Publishes the given state to the ROS network. This method is thread safe.

        :param master_state: the master state to publish

        :type master_state:  `fkie_master_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/MasterState.html>`_
        '''
        with self.__lock:
            try:
                self.pubchanges.publish(master_state)
                if not self._services_initialized:
                    # initialize the ROS services
                    self._services_initialized = True
                    rospy.Service('~list_masters', DiscoverMasters, self.rosservice_list_masters)
                    rospy.Service('~refresh', std_srvs.srv.Empty, self.rosservice_refresh)
            except:
                traceback.print_exc()

    def publish_stats(self, stats):
        '''
        Publishes the link quality states to the ROS network.This method is thread safe.

        :param stats: the link quality states to publish

        :type stats:  `fkie_master_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/LinkStatesStamped.html>`_
        '''
        if not rospy.is_shutdown():
            with self.__lock:
                try:
                    self.pubstats.publish(stats)
                except:
                    traceback.print_exc()

    def update_master_errors(self):
        result = []
        with self.__lock:
            try:
                current_errors = self.master_monitor.getMasterErrors()[1]
                for (_, v) in self.masters.items():
                    # add all errors to the responce
                    for _, msg in v.errors.items():
                        result.append(msg)
                    # test for resolved addr
                    if v.mastername is not None and not v.errors and v.masteruri != self.master_monitor.getMasteruri():
                        try:
                            if v.masteruriaddr != v.monitor_hostname:
                                msg = "Resolved host of ROS_MASTER_URI %s=%s and origin discovered IP=%s are different. Fix your network settings and restart master_discovery!" % (v.master_hostname, v.masteruriaddr, v.monitor_hostname)
                                if v.masteruriaddr is None or not v.masteruriaddr.startswith('127.'):
                                    local_addresses = ['localhost'] + get_local_addresses()
                                    # check 127/8 and local addresses
                                    if v.masteruriaddr not in local_addresses:
                                        if msg not in current_errors:
                                            rospy.logwarn(msg)
                                        result.append(msg)
                        except Exception as e:
                            result.append("%s" % e)
                            rospy.logwarn("Error while resolve address for %s: %s" % (v.masteruri, e))
                try:
                    for _addr, msg in SEND_ERRORS.items():
                        result.append('%s' % msg)
                except:
                    pass
            except Exception as e:
                result.append("%s" % e)
                rospy.logwarn("%s" % e)
        self.master_monitor.update_master_errors(result)

    def rosservice_list_masters(self, req):
        '''
        Callback for the ROS service to get the current list of the known ROS masters.
        '''
        masters = list()
        with self.__lock:
            try:
                for (_, v) in self.masters.items():
                    if v.mastername is not None:
                        masters.append(ROSMaster(str(v.mastername),
                                                 v.masteruri,
                                                 rospy.Time(v.timestamp),
                                                 rospy.Time(v.timestamp_local),
                                                 v.online,
                                                 v.discoverername,
                                                 v.monitoruri))
            except:
                traceback.print_exc()
        return DiscoverMastersResponse(masters)

    def rosservice_refresh(self, req):
        '''
        Callback for the ROS service to send an active unicast and multicast request
        to each known master discovery.
        '''
        with self.__lock:
            try:
                for (k, v) in self.masters.items():
                    if v.mastername is not None:
                        # send an active unicast request
                        self._request_state(k[0][0], [v])
                if self._listen_mcast:
                    self._request_state()
#        self._send_current_state()
            except:
                traceback.print_exc()
        return []

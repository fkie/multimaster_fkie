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



from python_qt_binding.QtCore import QObject, Signal
import socket
import threading
import time

import rospy

from fkie_master_discovery.master_monitor import MasterMonitor, MasterConnectionException
import fkie_master_discovery.interface_finder as interface_finder

from fkie_node_manager_daemon.common import utf8

try:
    import std_srvs.srv
    from fkie_multimaster_msgs.msg import LinkStatesStamped, MasterState, ROSMaster  # , LinkState, SyncMasterInfo, SyncTopicInfo
    from fkie_multimaster_msgs.srv import DiscoverMasters  # , GetSyncInfo
except ImportError as e:
    import sys
    sys.stderr.write("Can't import messages and services of fkie_multimaster_msgs. Is fkie_multimaster_msgs package compiled?")
    raise ImportError(utf8(e))


class MasterListService(QObject):
    '''
    A class to retrieve the ROS master list from a ROS service. The service
    will be determine using U{fkie_master_discovery.interface_finder.get_listmaster_service()<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#interface-finder-module>}

    '''
    masterlist_signal = Signal(str, str, list)
    '''@ivar: a signal with a list of the masters retrieved from the master_discovery service 'list_masters'.
  ParameterB{:} C{masteruri}, C{service name}, C{[U{fkie_master_discovery.msg.ROSMaster<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/ROSMaster.html>}, ...]}'''
    masterlist_err_signal = Signal(str, str)
    '''@ivar: this signal is emitted if an error while calling #list_masters'
  service of master_discovery is failed.
  ParameterB{:} C{masteruri}, C{error}'''

    def __init__(self):
        QObject.__init__(self)
        self.__serviceThreads = {}
        self.__refreshThreads = {}
        self._lock = threading.RLock()

    def stop(self):
        print("  Shutdown discovery listener...")
        for _, thread in self.__serviceThreads.items():
            thread.join(3)
        for _, thread in self.__refreshThreads.items():
            thread.join(3)
        print("  Discovery listener is off!")

    def retrieveMasterList(self, masteruri, wait=False, check_host=True):
        '''
        This method use the service 'list_masters' of the master_discovery to get
        the list of discovered ROS master. The retrieved list will be emitted as
        masterlist_signal.
        @param masteruri: the ROS master URI
        @type masteruri: C{str}
        @param wait: wait for the service
        @type wait: C{boolean}
        '''
        with self._lock:
            if masteruri not in self.__serviceThreads:
                upthread = MasterListThread(masteruri, wait, check_host)
                upthread.master_list_signal.connect(self._on_master_list)
                upthread.err_signal.connect(self._on_err)
                self.__serviceThreads[masteruri] = upthread
                upthread.start()

    def refresh(self, masteruri, wait=False, check_host=True):
        '''
        This method use the service 'refresh' of the master_discovery to refresh the
        discovered masters.
        @param masteruri: the ROS master URI
        @type masteruri: C{str}
        @param wait: wait for the service
        @type wait: C{boolean}
        '''
        with self._lock:
            if masteruri not in self.__refreshThreads:
                upthread = MasterRefreshThread(masteruri, wait, check_host)
                upthread.ok_signal.connect(self._on_ok)
                upthread.err_signal.connect(self._on_err)
                self.__refreshThreads[masteruri] = upthread
                upthread.start()

    def _on_master_list(self, masteruri, service_name, items):
        with self._lock:
            try:
                thread = self.__serviceThreads.pop(masteruri)
                del thread
            except KeyError:
                pass
        self.masterlist_signal.emit(masteruri, service_name, items)

    def _on_err(self, masteruri, msg, on_refresh):
        with self._lock:
            try:
                if on_refresh:
                    thread = self.__refreshThreads.pop(masteruri)
                    del thread
                else:
                    thread = self.__serviceThreads.pop(masteruri)
                    del thread
            except KeyError:
                pass
        self.masterlist_err_signal.emit(masteruri, msg)

    def _on_ok(self, masteruri):
        with self._lock:
            try:
                thread = self.__refreshThreads.pop(masteruri)
                del thread
            except KeyError:
                pass


class MasterListThread(QObject, threading.Thread):
    '''
    A thread to to retrieve the list of discovered ROS master from master_discovery
    service and publish it by sending a QT signal.
    '''
    master_list_signal = Signal(str, str, list)
    err_signal = Signal(str, str, bool)

    def __init__(self, masteruri, wait, check_host=True, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._masteruri = masteruri
        self._wait = wait
        self._check_host = check_host
        self.setDaemon(True)

    def run(self):
        '''
        '''
        if self._masteruri:
            found = False
            service_names = interface_finder.get_listmaster_service(self._masteruri, self._wait, check_host=self._check_host)
            err_msg = ''
            for service_name in service_names:
                rospy.logdebug("service 'list_masters' found on %s as %s", self._masteruri, service_name)
                if self._wait:
                    rospy.wait_for_service(service_name)
                socket.setdefaulttimeout(3)
                discoverMasters = rospy.ServiceProxy(service_name, DiscoverMasters)
                try:
                    resp = discoverMasters()
                except rospy.ServiceException as e:
                    err_msg = "Service call 'list_masters' failed: %s" % utf8(e)
                    rospy.logwarn(err_msg)
                    self.err_signal.emit(self._masteruri, "Service call '%s' failed: %s" % (service_name, err_msg), False)
                else:
                    if resp.masters:
                        self.master_list_signal.emit(self._masteruri, service_name, resp.masters)
                        found = True
                    else:
                        self.err_signal.emit(self._masteruri, "local 'master_discovery' reports empty master list, it seems he has a problem", False)
                finally:
                    socket.setdefaulttimeout(None)
            if not found:
                self.err_signal.emit(self._masteruri, "no service 'list_masters' found on %s" % self._masteruri, False)


class MasterRefreshThread(QObject, threading.Thread):
    '''
    A thread to call the refresh service of master discovery.
    '''
    ok_signal = Signal(str)
    err_signal = Signal(str, str, bool)

    def __init__(self, masteruri, wait, check_host=True, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._masteruri = masteruri
        self._wait = wait
        self._check_host = check_host
        self.setDaemon(True)

    def run(self):
        '''
        '''
        if self._masteruri:
            service_names = interface_finder.get_refresh_service(self._masteruri, self._wait, check_host=self._check_host)
            err_msg = ''
            for service_name in service_names:
                rospy.logdebug("service 'refresh' found on %s as %s", self._masteruri, service_name)
                if self._wait:
                    rospy.wait_for_service(service_name)
                socket.setdefaulttimeout(3)
                refreshMasters = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)
                try:
                    _ = refreshMasters()
                    self.ok_signal.emit(self._masteruri)
                except rospy.ServiceException as e:
                    rospy.logwarn("ERROR Service call 'refresh' failed: %s", utf8(e))
                    self.err_signal.emit(self._masteruri, "ERROR Service call 'refresh' failed: %s" % utf8(err_msg), True)
                finally:
                    socket.setdefaulttimeout(None)


class MasterStateTopic(QObject):
    '''
    A class to receive the ROS master state updates from a ROS topic. The topic
    will be determine using U{fkie_master_discovery.interface_finder.get_changes_topic()<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#interface-finder-module>}.
    '''
    state_signal = Signal(MasterState)
    ''':ivar state_signal: a signal to inform the receiver about new master state.
    Parameter: U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}'''

    def registerByROS(self, masteruri, wait=False):
        '''
        This method creates a ROS subscriber to received the notifications of ROS
        master updates. The retrieved messages will be emitted as state_signal.
        @param masteruri: the ROS master URI
        @type masteruri: C{str}
        @param wait: wait for the topic
        @type wait: C{boolean}
        @return: the topic name or an empty string
        '''
        result = ''
        topic_names = interface_finder.get_changes_topic(masteruri, wait)
        self.stop()
        if not hasattr(self, 'sub_changes'):
            self.sub_changes = []
            self.sub_names = []
        if set(self.sub_names) != set(topic_names):
            self.stop()
            self.sub_changes = []
            for topic_name in topic_names:
                rospy.loginfo("listen for updates on %s", topic_name)
                sub_changes = rospy.Subscriber(topic_name, MasterState, self.handlerMasterStateMsg)
                self.sub_changes.append(sub_changes)
                result = topic_name
            self.sub_names = topic_names
        return result

    def stop(self):
        '''
        Unregister the subscribed topics
        '''
        if hasattr(self, 'sub_changes'):
            for s in self.sub_changes:
                try:
                    s.unregister()
                except Exception as e:
                    rospy.logwarn("Error while unregister master state topic %s" % e)
            del self.sub_changes

    def handlerMasterStateMsg(self, msg):
        '''
        The method to handle the received MasterState messages. The received message
        will be emitted as state_signal.
        @param msg: the received message
        @type msg: U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}
        '''
        self.state_signal.emit(msg)


class MasterStatisticTopic(QObject):
    '''
    A class to receive the connections statistics from a ROS topic. The topic
    will be determine using U{fkie_master_discovery.interface_finder.get_stats_topic()<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#interface-finder-module>}
    '''
    stats_signal = Signal(LinkStatesStamped)
    '''@ivar: a signal with a list of link states to discovered ROS masters.
  Paramter: U{fkie_master_discovery.msg.LinkStatesStamped<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/LinkStatesStamped.html>}'''

    def registerByROS(self, masteruri, wait=False):
        '''
        This method creates a ROS subscriber to received the notifications of
        connection updates. The retrieved messages will be emitted as stats_signal.
        @param masteruri: the ROS master URI
        @type masteruri: str
        @param wait: wait for the topic
        @type wait: boolean
        @return: the topic name or an empty string
        '''
        result = ''
        self.stop()
        self.sub_stats = []
        topic_names = interface_finder.get_stats_topic(masteruri, wait)
        for topic_name in topic_names:
            rospy.loginfo("listen for connection statistics on %s", topic_name)
            sub_stats = rospy.Subscriber(topic_name, LinkStatesStamped, self.handlerMasterStatsMsg)
            self.sub_stats.append(sub_stats)
            result = topic_name
        return result

    def stop(self):
        '''
        Unregister the subscribed topics.
        '''
        if hasattr(self, 'sub_stats'):
            for s in self.sub_stats:
                s.unregister()
            del self.sub_stats

    def handlerMasterStatsMsg(self, msg):
        '''
        The method to handle the received LinkStatesStamped messages. The received
        message will be emitted as stats_signal.
        @param msg: the received message
        @type msg: U{fkie_master_discovery.msg.LinkStatesStamped<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/LinkStatesStamped.html>}
        '''
        self.stats_signal.emit(msg)


class OwnMasterMonitoring(QObject):
    '''
    A class to monitor the state of the master. Will be used, if no master
    discovering is available. On changes the 'state_signal' of type
    U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>} will be emitted.
    '''
    state_signal = Signal(MasterState)
    '''@ivar: a signal to inform the receiver about new master state.
    Parameter: U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}'''

    err_signal = Signal(str)
    '''@ivar: a signal to inform about an error.
  Parameter: L{str}'''

    ROSMASTER_HZ = 1
    '''@ivar: the rate to test ROS master for changes.'''

    def init(self, monitor_port):
        '''
        Creates the local monitoring. Call start() to run the local monitoring.
        @param monitor_port: the port of the XML-RPC Server created by monitoring class.
        @type monitor_port: C{int}
        '''
        self._master_monitor = MasterMonitor(monitor_port, False)
        self._do_pause = True
        self._do_finish = False
        self._masteruri = self._master_monitor.getMasteruri()
        self._local_addr = self._master_monitor.getMastername()
        self._masterMonitorThread = threading.Thread(target=self.mastermonitor_loop)
        self._masterMonitorThread.setDaemon(True)
        self._masterMonitorThread.start()
        self._last_error = (time.time(), None)

    def is_running(self):
        return not self._do_pause

    def stop(self):
        '''
        Stop the local master monitoring
        '''
        self._do_finish = True
        if self._master_monitor.is_running():
            print("  Shutdown the local master monitoring...")
            self._masterMonitorThread.join(15)
            self._master_monitor.shutdown()
            print("  Local master monitoring is off!")

    def mastermonitor_loop(self):
        '''
        The method test periodically the state of the ROS master. The new state will
        be published as 'state_signal'.
        '''
        import os
        current_check_hz = OwnMasterMonitoring.ROSMASTER_HZ
        while (not rospy.is_shutdown() and not self._do_finish):
            try:
                if not self._do_pause:
                    cputimes = os.times()
                    cputime_init = cputimes[0] + cputimes[1]
                    if self._master_monitor.checkState():
                        mon_state = self._master_monitor.getCurrentState()
                        # publish the new state
                        state = MasterState(MasterState.STATE_CHANGED,
                                            ROSMaster(utf8(self._local_addr),
                                                      utf8(self._masteruri),
                                                      rospy.Time(mon_state.timestamp),
                                                      rospy.Time(mon_state.timestamp_local),
                                                      True,
                                                      rospy.get_name(),
                                                      ''.join(['http://localhost:', utf8(self._master_monitor.rpcport)])))
                        self.state_signal.emit(state)
                    # adapt the check rate to the CPU usage time
                    cputimes = os.times()
                    cputime = cputimes[0] + cputimes[1] - cputime_init
                    if current_check_hz * cputime > 0.20:
                        current_check_hz = float(current_check_hz) / 2.0
                    elif current_check_hz * cputime < 0.10 and current_check_hz < OwnMasterMonitoring.ROSMASTER_HZ:
                        current_check_hz = float(current_check_hz) * 2.0
            except MasterConnectionException as mce:
                self._handle_exception("MasterConnectionException while master check loop", mce)
            except RuntimeError as ree:
                # will thrown on exit of the app while try to emit the signal
                self._handle_exception("RuntimeError while master check loop", ree)
            if not rospy.is_shutdown() and not self._do_finish:
                time.sleep(1.0 / current_check_hz)

    def _handle_exception(self, prefix, exception):
        text = '%s: %s' % (prefix, exception)
        if self._last_error[1] != text or time.time() - self._last_error[0] > 60:
            self._last_error = (time.time(), text)
            rospy.logwarn(text)
        self.err_signal.emit(text)

    def pause(self, state):
        '''
        Sets the local monitoring to pause.
        @param state: On/Off pause
        @type state: C{boolean}
        '''
        if not state and self._do_pause != state:
            self._master_monitor.reset()
        self._do_pause = state

    def isPaused(self):
        '''
        @return: True if the local monitoring of the Master state is paused.
        @rtype: C{boolean}
        '''
        return self._do_pause

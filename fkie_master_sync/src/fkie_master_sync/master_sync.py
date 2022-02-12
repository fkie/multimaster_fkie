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



import socket
import threading
import time
import uuid
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fkie_multimaster_msgs.msg import MasterState  # , LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo
from fkie_multimaster_msgs.srv import DiscoverMasters, GetSyncInfo, GetSyncInfoResponse
import rospy

from fkie_master_discovery.common import masteruri_from_master, resolve_url, read_interface, create_pattern, is_empty_pattern, get_hostname
from fkie_master_discovery.master_info import MasterInfo
import fkie_master_discovery.interface_finder as interface_finder

from .sync_thread import SyncThread


class Main(object):
    '''
    '''

    UPDATE_INTERVALL = 30

    def __init__(self):
        '''
        Creates a new instance. Find the topic of the master_discovery node using
        U{fkie_master_discovery.interface_finder.get_changes_topic()
        <http://docs.ros.org/api/fkie_master_discovery/html/modules.html#interface-finder-module>}.
        Also the parameter C{~ignore_hosts} will be analyzed to exclude hosts from sync.
        '''
        self.masters = {}
        # the connection to the local service master
        self.masteruri = masteruri_from_master()
        self.hostname = get_hostname(self.masteruri)
        self._localname = ''
        '''@ivar: the ROS master URI of the C{local} ROS master. '''
        self.__lock = threading.RLock()
        # load interface
        self._load_interface()
        # subscribe to changes notifier topics
        self._check_host = rospy.get_param('~check_host', True)
        topic_names = interface_finder.get_changes_topic(masteruri_from_master(), check_host=self._check_host)
        self.sub_changes = dict()
        '''@ivar: `dict` with topics {name: U{rospy.Subscriber<http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html>}} publishes the changes of the discovered ROS masters.'''
        for topic_name in topic_names:
            rospy.loginfo("listen for updates on %s", topic_name)
            self.sub_changes[topic_name] = rospy.Subscriber(topic_name, MasterState, self._rosmsg_callback_master_state)
        self.__timestamp_local = None
        self.__own_state = None
        self.update_timer = None
        self.resync_timer = None
        self.own_state_getter = None
        self._timer_update_diagnostics = None
        self._join_threads = dict()  # threads waiting for stopping the sync thread
        # initialize the ROS services
        rospy.Service('~get_sync_info', GetSyncInfo, self._rosservice_get_sync_info)
        rospy.on_shutdown(self.finish)
        self._current_diagnistic_level = None
        self.pub_diag = rospy.Publisher( "/diagnostics", DiagnosticArray, queue_size=10, latch=True)
        self.obtain_masters()

    def _rosmsg_callback_master_state(self, data):
        '''
        The method to handle the received MasterState messages. Based on this message
        new threads to synchronize with remote ROS master will be created, updated or
        removed.
        @param data: the received message
        @type data: U{fkie_master_discovery.MasterState
        <http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}
        '''
        with self.__lock:
            if not rospy.is_shutdown():
                if data.state in [MasterState.STATE_REMOVED]:
                    self.remove_master(data.master.name)
                elif data.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
                    m = data.master
                    self.update_master(m.name, m.uri, m.last_change.to_sec(), m.last_change_local.to_sec(), m.discoverer_name, m.monitoruri, m.online)

    def _callback_perform_resync(self):
        if self.resync_timer is not None:
            self.resync_timer.cancel()
        self.resync_timer = threading.Timer(0.1, self._perform_resync)
        self.resync_timer.start()

    def obtain_masters(self):
        '''
        This method use the service 'list_masters' of the master_discoverer to get
        the list of discovered ROS master. Based on this list the L{SyncThread} for
        synchronization will be created.
        @see: U{fkie_master_discovery.interface_finder.get_listmaster_service()
            <http://docs.ros.org/api/fkie_master_discovery/html/modules.html#interface-finder-module>}
        '''
        if not rospy.is_shutdown():
            service_names = interface_finder.get_listmaster_service(masteruri_from_master(), False, check_host=self._check_host)
            for service_name in service_names:
                try:
                    with self.__lock:
                        try:
                            socket.setdefaulttimeout(5)
                            discoverMasters = rospy.ServiceProxy(service_name, DiscoverMasters)
                            resp = discoverMasters()
                            masters = []
                            master_names = [m.name for m in resp.masters]
                            rospy.loginfo("ROS masters obtained from '%s': %s", service_name, master_names)
                            for m in resp.masters:
                                if self._can_sync(m.name):  # do not sync to the master, if it is in ignore list or not in filled sync list
                                    masters.append(m.name)
                                self.update_master(m.name, m.uri, m.last_change.to_sec(), m.last_change_local.to_sec(), m.discoverer_name, m.monitoruri, m.online)
                            for key in set(self.masters.keys()) - set(masters):
                                self.remove_master(self.masters[key].name)
                        except rospy.ServiceException as e:
                            rospy.logwarn("ERROR Service call 'list_masters' failed: %s", str(e))
                except:
                    import traceback
                    rospy.logwarn("ERROR while initial list masters: %s", traceback.format_exc())
                finally:
                    socket.setdefaulttimeout(None)
            self.update_timer = threading.Timer(self.UPDATE_INTERVALL, self.obtain_masters)
            self.update_timer.start()

    def update_master(self, mastername, masteruri, timestamp, timestamp_local, discoverer_name, monitoruri, online):
        '''
        Updates the timestamp of the given ROS master, or creates a new L{SyncThread} to
        synchronize the local master with given ROS master.
        @param mastername: the name of the remote ROS master to update or synchronize.
        @type mastername: C{str}
        @param masteruri: the URI of the remote ROS master.
        @type masteruri: C{str}
        @param timestamp: the timestamp of the remote ROS master.
        @type timestamp: C{float64}
        @param timestamp_local: the timestamp of the remote ROS master. (only local changes)
        @type timestamp_local: C{float64}
        @param discoverer_name: the name of the remote master_discoverer node
        @type discoverer_name: C{str}
        @param monitoruri: the URI of the RPC interface of the remote master_discoverer node.
        @type monitoruri: C{str}
        @param online: the current state on the master.
        @type online: C{bool}
        '''
        try:
            with self.__lock:
                if (masteruri != self.masteruri):
                    if self._can_sync(mastername):
                        # do not sync to the master, if it is in ignore list
                        if self.__resync_on_reconnect and mastername in self.masters:
                            self.masters[mastername].set_online(online, self.__resync_on_reconnect_timeout)
                        if online:
                            if mastername in self.masters:
                                    # updates only, if local changes are occured
                                self.masters[mastername].update(mastername, masteruri, discoverer_name, monitoruri, timestamp_local)
                            else:
                                self.masters[mastername] = SyncThread(mastername, masteruri, discoverer_name, monitoruri, 0.0, self.__sync_topics_on_demand, callback_resync=self._callback_perform_resync)
                                if self.__own_state is not None:
                                    self.masters[mastername].set_own_masterstate(MasterInfo.from_list(self.__own_state))
                                self.masters[mastername].update(mastername, masteruri, discoverer_name, monitoruri, timestamp_local)
                elif self.__timestamp_local != timestamp_local:  # self.__sync_topics_on_demand:
                    # get the master info from local discovery master and set it to all sync threads
                    self._localname = mastername
                    self.own_state_getter = threading.Thread(target=self.get_own_state, args=(monitoruri,))
                    self.own_state_getter.start()
                if self._timer_update_diagnostics is None or not self._timer_update_diagnostics.is_alive():
                    # check for topics type and checksum for all hosts. Not blocking!
                    self._timer_update_diagnostics = threading.Thread(target=self._update_diagnostics_state)
                    self._timer_update_diagnostics.start()
        except:
            import traceback
            rospy.logwarn("ERROR while update master[%s]: %s", str(mastername), traceback.format_exc())

    def get_own_state(self, monitoruri):
        '''
        Gets the master info from local master discovery and set it to all sync threads.
        This function is running in a thread!!!
        '''
        try:
            socket.setdefaulttimeout(3)
            own_monitor = xmlrpcclient.ServerProxy(monitoruri)
            self.__own_state = own_monitor.masterInfo()
            own_state = MasterInfo.from_list(self.__own_state)
            socket.setdefaulttimeout(None)
            with self.__lock:
                # update the state for all sync threads
                for (_, s) in self.masters.items():
                    s.set_own_masterstate(own_state, self.__sync_topics_on_demand)
                self.__timestamp_local = own_state.timestamp_local
        except:
            import traceback
            rospy.logwarn("ERROR while getting own state from '%s': %s", monitoruri, traceback.format_exc())
            socket.setdefaulttimeout(None)
            time.sleep(3)
            if self.own_state_getter is not None and not rospy.is_shutdown():
                self.own_state_getter = threading.Thread(target=self.get_own_state, args=(monitoruri,))
                self.own_state_getter.start()

    def remove_master(self, ros_master_name):
        '''
        Removes the master with given name from the synchronization list.
        @param ros_master_name: the name of the ROS master to remove.
        @type ros_master_name: C{str}
        '''
        try:
            with self.__lock:
                if ros_master_name in self.masters:
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
            rospy.loginfo("  Stop synchronization to `%s`" % sync_thread.name)
            sync_thread.stop()
            with self.__lock:
                del self._join_threads[ident]
            rospy.loginfo("  Finished synchronization to `%s`" % sync_thread.name)
            del sync_thread

    def finish(self, msg=''):
        '''
        Removes all remote masters and unregister their topics and services.
        '''
        rospy.loginfo("Stop synchronization...")
        with self.__lock:
            # stop update timer
            rospy.loginfo("  Stop timers...")
            if self.update_timer is not None:
                self.update_timer.cancel()
            if self.resync_timer is not None:
                self.resync_timer.cancel()
            # unregister from update topics
            rospy.loginfo("  Unregister from master discovery...")
            for (_, v) in self.sub_changes.items():
                v.unregister()
            self.own_state_getter = None
            # Stop all sync threads
            for key in self.masters.keys():
                rospy.loginfo("  Remove master: %s", key)
                self.remove_master(key)
        # wait for their ending
        while len(self._join_threads) > 0:
            rospy.loginfo("  Wait for ending of %s threads ...", str(len(self._join_threads)))
            time.sleep(1)
        rospy.loginfo("Synchronization is now off")

    def _perform_resync(self):
        self.resync_timer = None
        with self.__lock:
            for (_, s) in self.masters.items():
                s.perform_resync()

    def _rosservice_get_sync_info(self, req):
        '''
        Callback for the ROS service to get the info to synchronized nodes.
        '''
        masters = list()
        try:
            with self.__lock:
                for (_, s) in self.masters.items():
                    masters.append(s.get_sync_info())
        except:
            import traceback
            traceback.print_exc()
        finally:
            return GetSyncInfoResponse(masters)

    def _load_interface(self):
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
                if 'sync_topics_on_demand' in data:
                    self.__sync_topics_on_demand = data['sync_topics_on_demand']
            elif rospy.has_param('~sync_topics_on_demand'):
                self.__sync_topics_on_demand = rospy.get_param('~sync_topics_on_demand')
            rospy.loginfo("sync_topics_on_demand: %s", self.__sync_topics_on_demand)
            self.__resync_on_reconnect = rospy.get_param('~resync_on_reconnect', True)
            rospy.loginfo("resync_on_reconnect: %s", self.__resync_on_reconnect)
            self.__resync_on_reconnect_timeout = rospy.get_param('~resync_on_reconnect_timeout', 0)
            rospy.loginfo("resync_on_reconnect_timeout: %s", self.__resync_on_reconnect_timeout)
        except:
            import traceback
            # kill the ros node, to notify the user about the error
            rospy.logerr("Error on load interface: %s", traceback.format_exc())
            import os
            import signal
            os.kill(os.getpid(), signal.SIGKILL)

    def _can_sync(self, mastername):
        result = False
        if is_empty_pattern(self._re_ignore_hosts):
            if is_empty_pattern(self._re_sync_hosts):
                result = True
            elif self._re_sync_hosts.match(mastername) is not None:
                result = True
        elif self._re_ignore_hosts.match(mastername) is None:
            result = True
        elif not is_empty_pattern(self._re_sync_hosts):
            if self._re_sync_hosts.match(mastername) is not None:
                result = True
        return result

    def _update_diagnostics_state(self):
        md5_warnings = {}
        ttype_warnings = {}
        for mname, mth in self.masters.items():
            warnings = mth.get_md5warnigs()
            if warnings:
                md5_warnings[mname] = warnings
            twarnings = mth.get_topic_type_warnings()
            if twarnings:
                ttype_warnings[mname] = twarnings
        level = 0
        if md5_warnings or ttype_warnings:
            level = 1
        if self._current_diagnistic_level != level:
            da = DiagnosticArray()
            if md5_warnings or ttype_warnings:
                # add warnings for all hosts with topic types with different md5sum
                for mname, warnings in md5_warnings.items():
                    diag_state = DiagnosticStatus()
                    diag_state.level = level
                    diag_state.name = rospy.get_name()
                    diag_state.message = 'Wrong topic md5sum @ %s and %s' % (mname, self._localname)
                    diag_state.hardware_id = self.hostname
                    for (topicname, _node, _nodeuri), tmtype in warnings.items():
                        if isinstance(tmtype, tuple):
                            (md5sum, ttype) = tmtype
                            if md5sum is not None:
                                key = KeyValue()
                                key.key = topicname
                                key.value = str(ttype)
                                diag_state.values.append(key)
                    da.status.append(diag_state)
                # add warnings if a topic with different type is synchrinozied to local host
                for mname, warnings in ttype_warnings.items():
                    diag_state = DiagnosticStatus()
                    diag_state.level = level
                    diag_state.name = rospy.get_name()
                    diag_state.message = 'Wrong topics type @ %s and %s' % (mname, self._localname)
                    diag_state.hardware_id = self.hostname
                    for (topicname, _node, _nodeuri), tmtype in warnings.items():
                        ttype = tmtype
                        if isinstance(tmtype, tuple):
                            (md5sum, ttype) = tmtype
                        key = KeyValue()
                        key.key = topicname
                        key.value = str(ttype)
                        diag_state.values.append(key)
                    da.status.append(diag_state)
            else:
                # clear all warnings
                diag_state = DiagnosticStatus()
                diag_state.level = 0
                diag_state.name = rospy.get_name()
                diag_state.message = ""
                diag_state.hardware_id = self.hostname
                da.status.append(diag_state)
            da.header.stamp = rospy.Time.now()
            self.pub_diag.publish(da)
            self._current_diagnistic_level = level

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



import random
import roslib
import roslib.message
import socket
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

from fkie_multimaster_msgs.msg import SyncTopicInfo, SyncServiceInfo, SyncMasterInfo
import rospy

from fkie_master_discovery.common import masteruri_from_ros, get_hostname
from fkie_master_discovery.filter_interface import FilterInterface


class SyncThread(object):
    '''
    A thread to synchronize the local ROS master with a remote master. While the
    synchronization only the topic of the remote ROS master will be registered by
    the local ROS master. The remote ROS master will be keep unchanged.
    '''

    MAX_UPDATE_DELAY = 5  # times

    MSG_ANY_TYPE = '*'

    def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, sync_on_demand=False, callback_resync=None):
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
        self._online = True
        self._offline_ts = 0

        self.masteruri_local = masteruri_from_ros()
        self.hostname_local = get_hostname(self.masteruri_local)
        rospy.logdebug("SyncThread[%s]: create this sync thread, discoverer_name: %s", self.name, self.discoverer_name)
        # synchronization variables
        self.__lock_info = threading.RLock()
        self.__lock_intern = threading.RLock()
        self._use_filtered_method = None
        self._use_md5check_topics = None
        self._md5warnings = {}  # ditionary of {(topicname, node, nodeuri) : (topictype, md5sum)}
        self._topic_type_warnings = {}  # ditionary of {(topicname, node, nodeuri) : remote topictype}
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
        self.__callback_resync = callback_resync
        self.__has_remove_sync = False

        # setup the filter
        self._filter = FilterInterface()
        self._filter.load(self.name,
                          ['/rosout', self.discoverer_name, '/master_discovery', '/master_sync', '/node_manager', '/node_manager_daemon', '/zeroconf', '/param_sync'], [],
                          ['/rosout', '/rosout_agg', '/master_discovery/*', '/master_sync/*', '/zeroconf/*'], ['/'] if sync_on_demand else [],
                          ['/*get_loggers', '/*set_logger_level', '/master_discovery/*', '/master_sync/*', '/node_manager_daemon/*', '/zeroconf/*'], [],
                          # do not sync the bond message of the nodelets!!
                          ['bond/Status', 'fkie_multimaster_msgs/SyncTopicInfo', 'fkie_multimaster_msgs/SyncServiceInfo', 'fkie_multimaster_msgs/SyncMasterInfo', 'fkie_multimaster_msgs/MasterState'],
                          [], [],
                          [])

        # congestion avoidance: wait for random.random*2 sec. If an update request
        # is received try to cancel and restart the current timer. The timer can be
        # canceled for maximal MAX_UPDATE_DELAY times.
        self._update_timer = None
        self._delayed_update = 0
        self.__on_update = False

    def get_sync_info(self):
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
                for (t_n, _t_t, n_n, n_uri) in self.__publisher:
                    result_publisher.append(SyncTopicInfo(t_n, n_n, n_uri))
                    result_set.add(n_n)
                for (t_n, _t_t, n_n, n_uri) in self.__subscriber:
                    result_subscriber.append(SyncTopicInfo(t_n, n_n, n_uri))
                    result_set.add(n_n)
                for (s_n, s_uri, n_n, n_uri) in self.__services:
                    result_services.append(SyncServiceInfo(s_n, s_uri, n_n, n_uri))
                    result_set.add(n_n)
                self.__sync_info = SyncMasterInfo(self.uri, list(result_set), result_publisher, result_subscriber, result_services)
            return self.__sync_info

    def set_online(self, value, resync_on_reconnect_timeout=0.):
        if value:
            if not self._online:
                with self.__lock_intern:
                    self._online = True
                    offline_duration = time.time() - self._offline_ts
                    if offline_duration >= resync_on_reconnect_timeout:
                        rospy.loginfo("SyncThread[%s]: perform resync after the host was offline (unregister and register again to avoid connection losses to python topic. These does not suppot reconnection!)", self.name)
                        if self._update_timer is not None:
                            self._update_timer.cancel()
                        self._unreg_on_finish()
                        self.__unregistered = False
                        self.__publisher = []
                        self.__subscriber = []
                        self.__services = []
                        self.timestamp = 0.
                        self.timestamp_local = 0.
                        self.timestamp_remote = 0.
                    else:
                        rospy.loginfo("SyncThread[%s]: skip resync after the host was offline because of resync_on_reconnect_timeout=%.2f and the host was only %.2f sec offline", self.name, resync_on_reconnect_timeout, offline_duration)
        else:
            self._online = False
            self._offline_ts = time.time()

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
            self.timestamp_remote = timestamp
            if (self.timestamp_local != timestamp):
                rospy.logdebug("SyncThread[%s]: update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp, self.timestamp_local)
                self.name = name
                self.uri = uri
                self.discoverer_name = discoverer_name
                self.monitoruri = monitoruri
                self._request_update()

#    rospy.logdebug("SyncThread[%s]: update exit", self.name)

    def set_own_masterstate(self, own_state, sync_on_demand=False):
        '''
        Sets the state of the local ROS master state. If this state is not None, the topics on demand will be synchronized.
        @param own_state: the state of the local ROS master state
        @type own_state:  C{fkie_master_discovery/MasterInfo}
        @param sync_on_demand: if True, sync only topic, which are also local exists (Default: False)
        @type sync_on_demand:  bool
        '''
        with self.__lock_intern:
            timestamp_local = own_state.timestamp_local
            if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
                ownstate_ts = self.__own_state.timestamp_local if self.__own_state is not None else float('nan')
                rospy.logdebug("SyncThread[%s]: local state update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp_local, ownstate_ts)
                self.__own_state = own_state
                if sync_on_demand:
                    self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
                self._request_update()

    def stop(self):
        '''
        Stops running thread.
        '''
        rospy.logdebug("  SyncThread[%s]: stop request", self.name)
        with self.__lock_intern:
            if self._update_timer is not None:
                self._update_timer.cancel()
            self._unreg_on_finish()
        rospy.logdebug("  SyncThread[%s]: stop exit", self.name)

    def _request_update(self):
        with self.__lock_intern:
            r = random.random() * 2.
            # start update timer with a random waiting time to avoid a congestion picks on changes of ROS master state
            if self._update_timer is None or not self._update_timer.is_alive():
                del self._update_timer
                self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                self._update_timer.start()
            else:
                if self._delayed_update < self.MAX_UPDATE_DELAY:
                    # if the timer thread can be canceled start new one
                    self._update_timer.cancel()
                    # if callback (XMLRPC request) is already running the timer is not canceled -> test for `self.__on_update`
                    if not self._update_timer.is_alive() or not self.__on_update:
                        self._delayed_update += 1
                        self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                        self._update_timer.start()

    def _request_remote_state(self, handler):
        self._delayed_update = 0
        self.__on_update = True
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a filtered method
            if self._use_filtered_method is None:
                try:
                    self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
                except:
                    self._use_filtered_method = False
            remote_state = None
            # get the state informations
            rospy.loginfo("SyncThread[%s] Requesting remote state from '%s'", self.name, self.monitoruri)
            if self._use_filtered_method:
                remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
                remote_state = remote_monitor.masterInfo()
            if not self.__unregistered:
                handler(remote_state)
        except:
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            self.__on_update = False
            socket.setdefaulttimeout(None)

    def _apply_remote_state(self, remote_state):
        rospy.loginfo("SyncThread[%s] Applying remote state...", self.name)
        try:
            rospy.logdebug("SyncThread[%s]: remote state: %s" % (self.name, remote_state))
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
            own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
            own_master_multi = xmlrpcclient.MultiCall(own_master)
            # fill the multicall object
            handler = []
            # sync the publishers
            publisher = []
            publisher_to_register = []
            remove_sync_found = False
            for (topic, nodes) in publishers:
                for node in nodes:
                    if node == rospy.get_name():
                        self.__has_remove_sync = True
                        remove_sync_found = True
                        break
            for (topic, nodes) in publishers:
                for node in nodes:
                    topictype = self._get_topictype(topic, topicTypes)
                    nodeuri = self._get_nodeuri(node, nodeProviders, remote_masteruri)
                    if topictype and nodeuri and not self._do_ignore_ntp(node, topic, topictype):
                        # register the nodes only once
                        if not ((topic, topictype, node, nodeuri) in self.__publisher):
                            publisher_to_register.append((topic, topictype, node, nodeuri))
                        publisher.append((topic, topictype, node, nodeuri))
            # unregister not updated publishers
            for (topic, topictype, node, nodeuri) in set(self.__publisher) - set(publisher):
                own_master_multi.unregisterPublisher(node, topic, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare UNPUB %s[%s] %s",
                                self.name, node, nodeuri, topic)
                handler.append(('upub', topic, node, nodeuri))
                # delete from warning list
                with self.__lock_info:
                    if (topic, node, nodeuri) in self._md5warnings:
                        del self._md5warnings[(topic, node, nodeuri)]
            # register new publishers
            for (topic, topictype, node, nodeuri) in publisher_to_register:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare PUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('pub', topic, topictype, node, nodeuri))

            # sync the subscribers
            subscriber = []
            subscriber_to_register = []
            for (topic, nodes) in subscribers:
                for node in nodes:
                    topictype = self._get_topictype(topic, topicTypes)
                    nodeuri = self._get_nodeuri(node, nodeProviders, remote_masteruri)
                    # if remote topictype is None, try to set to the local topic type
#          if not topictype and not self.__own_state is None:
#            if topic in self.__own_state.topics:
#              topictype = self.__own_state.topics[topic].type
                    if not topictype:
                        topictype = self.MSG_ANY_TYPE
                    if topictype and nodeuri and not self._do_ignore_nts(node, topic, topictype):
                        # register the node as subscriber in local ROS master
                        if not ((topic, node, nodeuri) in self.__subscriber):
                            subscriber_to_register.append((topic, topictype, node, nodeuri))
                        subscriber.append((topic, topictype, node, nodeuri))
            # unregister not updated topics
            for (topic, topictype, node, nodeuri) in set(self.__subscriber) - set(subscriber):
                own_master_multi.unregisterSubscriber(node, topic, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare UNSUB %s[%s] %s",
                            self.name, node, nodeuri, topic)
                handler.append(('usub', topic, node, nodeuri))
                # delete from warning list
                with self.__lock_info:
                    if (topic, node, nodeuri) in self._md5warnings:
                        del self._md5warnings[(topic, node, nodeuri)]
            # register new subscriber
            for (topic, topictype, node, nodeuri) in subscriber_to_register:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare SUB %s[%s] %s[%s]",
                            self.name, node, nodeuri, topic, topictype)
                handler.append(('sub', topic, topictype, node, nodeuri))
            # check for conflicts with local types before register remote topics
            with self.__lock_info:
                self._check_local_topic_types(publisher_to_register + subscriber_to_register)

            # sync the services
            services = []
            services_to_register = []
            for (service, nodes) in rservices:
                for node in nodes:
                    serviceuri = self._get_serviceuri(service, serviceProviders, remote_masteruri)
                    nodeuri = self._get_nodeuri(node, nodeProviders, remote_masteruri)
                    if serviceuri and nodeuri and not self._do_ignore_ns(node, service):
                        # register the node as publisher in local ROS master
                        if not ((service, serviceuri, node, nodeuri) in self.__services):
                            services_to_register.append((service, serviceuri, node, nodeuri))
                        services.append((service, serviceuri, node, nodeuri))
            # unregister not updated services
            for (service, serviceuri, node, nodeuri) in set(self.__services) - set(services):
                own_master_multi.unregisterService(node, service, serviceuri)
                rospy.logdebug("SyncThread[%s]: prepare UNSRV %s[%s] %s[%s]",
                            self.name, node, nodeuri, service, serviceuri)
                handler.append(('usrv', service, serviceuri, node, nodeuri))
            # register new services
            for (service, serviceuri, node, nodeuri) in services_to_register:
                own_master_multi.registerService(node, service, serviceuri, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare SRV %s[%s] %s[%s]",
                            self.name, node, nodeuri, service, serviceuri)
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
                self._check_multical_result(result, handler)
                # set the last synchronization time
                self.timestamp = stamp
                self.timestamp_local = stamp_local
                rospy.logdebug("SyncThread[%s]: current timestamp %.9f, local %.9f", self.name, stamp, stamp_local)
                if self.timestamp_remote > stamp_local:
                    rospy.logdebug("SyncThread[%s]: invoke next update, remote ts: %.9f", self.name, self.timestamp_remote)
                    self._update_timer = threading.Timer(random.random() * 2., self._request_remote_state, args=(self._apply_remote_state,))
                    self._update_timer.start()
            # check md5sum for topics
            with self.__lock_info:
                self._check_md5sums(publisher_to_register + subscriber_to_register)
            # check if remote master_sync was stopped
            if self.__has_remove_sync and not remove_sync_found:
                # resync
                if self.__callback_resync is not None:
                    self.__callback_resync()
                self.__has_remove_sync = False
        except:
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            socket.setdefaulttimeout(None)
        rospy.loginfo("SyncThread[%s] remote state applied.", self.name)

    def _check_multical_result(self, mresult, handler):
        if not self.__unregistered:
            # analyze the results of the registration call
            # HACK param to reduce publisher creation, see line 372
            publiser_to_update = {}
            for h, (code, statusMessage, r) in zip(handler, mresult):
                try:
                    if h[0] == 'sub':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: topic subscription error: %s (%s), %s %s, node: %s", self.name, h[1], h[2], str(code), str(statusMessage), h[3])
                        else:
                            rospy.logdebug("SyncThread[%s]: topic subscribed: %s, %s %s, node: %s", self.name, h[1], str(code), str(statusMessage), h[3])
                    if h[0] == 'sub' and code == 1 and len(r) > 0:
                        if not self._do_ignore_ntp(h[3], h[1], h[2]):
                            # topic, nodeuri, node : list of publisher uris
                            publiser_to_update[(h[1], h[4], h[3])] = r
                    elif h[0] == 'pub':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: topic advertise error: %s (%s), %s %s", self.name, h[1], h[2], str(code), str(statusMessage))
                        else:
                            rospy.logdebug("SyncThread[%s]: topic advertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usub':
                        rospy.logdebug("SyncThread[%s]: topic unsubscribed: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'upub':
                        rospy.logdebug("SyncThread[%s]: topic unadvertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'srv':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: service registration error: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                        else:
                            rospy.logdebug("SyncThread[%s]: service registered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usrv':
                        rospy.logdebug("SyncThread[%s]: service unregistered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                except:
                    rospy.logerr("SyncThread[%s] ERROR while analyzing the results of the registration call [%s]: %s", self.name, h[1], traceback.format_exc())
            # hack:
            # update publisher since they are not updated while registration of a subscriber
            # https://github.com/ros/ros_comm/blob/9162b32a42b5569ae42a94aa6426aafcb63021ae/tools/rosmaster/src/rosmaster/master_api.py#L195
            for (sub_topic, api, node), pub_uris in publiser_to_update.items():
                msg = "SyncThread[%s] publisherUpdate[%s] -> node: %s [%s], publisher uris: %s" % (self.name, sub_topic, api, node, pub_uris)
                try:
                    pub_client = xmlrpcclient.ServerProxy(api)
                    ret = pub_client.publisherUpdate('/master', sub_topic, pub_uris)
                    msg_suffix = "result=%s" % ret
                    rospy.logdebug("%s: %s", msg, msg_suffix)
                except Exception as ex:
                    msg_suffix = "exception=%s" % ex
                    rospy.logwarn("%s: %s", msg, msg_suffix)

    def perform_resync(self):
        # # create a multicall object
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        # fill the multicall object
        handler = []
        with self.__lock_info:
            # reregister subcriptions
            for (topic, topictype, node, nodeuri) in self.__subscriber:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare RESUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('sub', topic, topictype, node, nodeuri))
            # reregister publishers
            for (topic, topictype, node, nodeuri) in self.__publisher:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare REPUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('pub', topic, topictype, node, nodeuri))
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _check_md5sums(self, topics_to_register):
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a getTopicsMd5sum method
            if self._use_md5check_topics is None:
                try:
                    self._use_md5check_topics = 'getTopicsMd5sum' in remote_monitor.system.listMethods()
                except:
                    self._use_md5check_topics = False
            if self._use_md5check_topics:
                rospy.loginfo("SyncThread[%s] Requesting remote md5sums '%s'", self.name, self.monitoruri)
                topic_types = [topictype for _topic, topictype, _node, _nodeuri in topics_to_register]
                remote_md5sums_topics = remote_monitor.getTopicsMd5sum(topic_types)
                for rttype, rtmd5sum in remote_md5sums_topics:
                    try:
                        lmd5sum = None
                        msg_class = roslib.message.get_message_class(rttype)
                        if msg_class is not None:
                            lmd5sum = msg_class._md5sum
                        if lmd5sum != rtmd5sum:
                            for topicname, topictype, node, nodeuri in topics_to_register:
                                if topictype == rttype:
                                    if (topicname, node, nodeuri) not in self._md5warnings:
                                        if lmd5sum is None:
                                            rospy.logwarn("Unknown message type %s for topic: %s, local host: %s, remote host: %s" % (rttype, topicname, self.hostname_local, self.name))
                                        else:
                                            rospy.logwarn("Different checksum detected for topic: %s, type: %s, local host: %s, remote host: %s" % (topicname, rttype, self.hostname_local, self.name))
                                        self._md5warnings[(topicname, node, nodeuri)] = (topictype, lmd5sum)
                    except Exception as err:
                        import traceback
                        rospy.logwarn(err)
                        rospy.logwarn(traceback.format_exc())
        except:
            import traceback
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            socket.setdefaulttimeout(None)

    def _check_local_topic_types(self, topics_to_register):
        try:
            if self.__own_state is not None:
                for topicname, topictype, node, nodeuri in topics_to_register:
                    try:
                        if topicname in self.__own_state.topics:
                            own_topictype = self.__own_state.topics[topicname].type
                            if own_topictype not in ['*', None] and topictype not in ['*', None] :
                                if topictype != own_topictype:
                                    if (topicname, node, nodeuri) not in self._topic_type_warnings:
                                        rospy.logwarn("Different topic types detected for topic: %s, own type: %s remote type: %s, local host: %s, remote host: %s" % (topicname, own_topictype, topictype, self.hostname_local, self.name))
                                        self._topic_type_warnings[(topicname, node, nodeuri)] = "local: %s, remote: %s" % (own_topictype, topictype)
                    except Exception as err:
                        import traceback
                        rospy.logwarn(err)
                        rospy.logwarn(traceback.format_exc())
        except:
            import traceback
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            socket.setdefaulttimeout(None)


    def get_md5warnigs(self):
        with self.__lock_info:
            return dict(self._md5warnings)

    def get_topic_type_warnings(self):
        with self.__lock_info:
            return dict(self._topic_type_warnings)

    def _unreg_on_finish(self):
        with self.__lock_info:
            self.__unregistered = True
            try:
                rospy.logdebug("    SyncThread[%s] clear all registrations", self.name)
                socket.setdefaulttimeout(5)
                own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                # end routine if the master was removed
                for topic, _topictype, node, uri in self.__subscriber:
                    rospy.logdebug("    SyncThread[%s]   unsibscribe %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterSubscriber(node, topic, uri)
                    # TODO: unregister a remote subscriber while local publisher is still there
                    # Note: the connection between running components after unregistration is stil there!
                for topic, _topictype, node, uri in self.__publisher:
                    rospy.logdebug("    SyncThread[%s]   unadvertise %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterPublisher(node, topic, uri)
                for service, serviceuri, node, uri in self.__services:
                    rospy.logdebug("    SyncThread[%s]   unregister service %s [%s]" % (self.name, service, node))
                    own_master_multi.unregisterService(node, service, serviceuri)
                rospy.logdebug("    SyncThread[%s] execute a MultiCall", self.name)
                _ = own_master_multi()
                rospy.logdebug("    SyncThread[%s] finished", self.name)
            except:
                rospy.logerr("SyncThread[%s] ERROR while ending: %s", self.name, traceback.format_exc())
            socket.setdefaulttimeout(None)

    def _do_ignore_ntp(self, node, topic, topictype):
        if node == rospy.get_name():
            return True
        return self._filter.is_ignored_publisher(node, topic, topictype)

    def _do_ignore_nts(self, node, topic, topictype):
        if node == rospy.get_name():
            return True
        return self._filter.is_ignored_subscriber(node, topic, topictype)

    def _do_ignore_ns(self, node, service):
        if node == rospy.get_name():
            return True
        return self._filter.is_ignored_service(node, service)

    def _get_topictype(self, topic, topic_types):
        for (topicname, topic_type) in topic_types:
            if (topicname == topic):
                return topic_type.replace('None', '')
        return None

    def _get_nodeuri(self, node, nodes, remote_masteruri):
        for (nodename, uri, masteruri, pid, local) in nodes:
            if (nodename == node) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                # the node was registered originally to another ROS master -> do sync
                if masteruri != self.masteruri_local:
                    return uri
        return None

    def _get_serviceuri(self, service, nodes, remote_masteruri):
        for (servicename, uri, masteruri, _topic_type, local) in nodes:
            if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                if masteruri != self.masteruri_local:
                    return uri
        return None

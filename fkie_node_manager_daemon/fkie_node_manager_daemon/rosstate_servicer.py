# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List

import os
import json
from types import SimpleNamespace
import asyncio
from autobahn import wamp

from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.runtime_interface import RosProvider
from fkie_multimaster_msgs.crossbar.runtime_interface import RosNode
from fkie_multimaster_msgs.crossbar.runtime_interface import RosTopic
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchContent
from fkie_multimaster_msgs.logging.logging import Log
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import DiscoveredState
from fkie_multimaster_msgs.msg import Endpoint

import fkie_node_manager_daemon as nmd
from .url import NMD_DEFAULT_PORT
from .url import port_from_uri


class RosStateServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = NMD_DEFAULT_PORT, test_env=False):
        nmd.ros_node.get_logger().info("Create ros_state servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env)
        self._endpoints = {}  # uri : Endpoint
        self._ros_state = None  # DiscoveredState
        self.topic_name_state = '%s/%s/rosstate' % (
            nmd.settings.NM_DISCOVERY_NAMESPACE, nmd.settings.NM_DISCOVERY_NAME)
        self.topic_name_endpoint = '%s/daemons' % (
            nmd.settings.NM_DISCOVERY_NAMESPACE)
        qos_profile = QoSProfile(depth=100,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 # history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        nmd.ros_node.get_logger().info('listen for discovered items on %s' %
                                       self.topic_name_state)
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            DiscoveredState, self.topic_name_state, self._on_msg_state, qos_profile=qos_profile)
        nmd.ros_node.get_logger().info('listen for endpoint items on %s' %
                                       self.topic_name_endpoint)
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_profile)

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        for uri, endpoint in endpoints.items():
            provider = RosProvider(
                name=endpoint.ros_name, host=endpoint.name, port=port_from_uri(endpoint.uri))
            result.append(provider)
        return result

    def _crossbar_publish_masters(self):
        result = self._endpoints_to_provider(self._endpoints)
        try:
            self.publish('ros.provider.list',
                         json.dumps(result, cls=SelfEncoder))
        except Exception as cpe:
            pass

    def get_publisher_count(self):
        if hasattr(self, 'topic_name_endpoint') and self.topic_name_endpoint is not None:
            return nmd.ros_node.count_publishers(self.topic_name_endpoint)
        return -1

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        if hasattr(self, 'sub_discovered_state') and self.sub_discovered_state is not None:
            nmd.ros_node.destroy_subscription(self.sub_discovered_state)
            del self.sub_discovered_state
        if hasattr(self, 'sub_endpoints') and self.sub_endpoints is not None:
            nmd.ros_node.destroy_subscription(self.sub_endpoints)
            del self.sub_endpoints

    def _on_msg_state(self, msg: DiscoveredState):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.DiscoveredState<XXX>
        '''
        nmd.ros_node.get_logger().info('new message on %s' % self.topic_name_state)
        self._ros_state = msg
        try:
            self.publish('ros.system.changed', "")
        except Exception:
            pass

    def _on_msg_endpoint(self, msg: Endpoint):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.Endpoint<XXX>
        '''
        nmd.ros_node.get_logger().info('new message on %s' % self.topic_name_endpoint)
        if msg.on_shutdown:
            if msg.uri in self._endpoints:
                del self._endpoints[msg.uri]
        else:
            self._endpoints[msg.uri] = msg
        self._crossbar_publish_masters()

    @wamp.register('ros.provider.get_list')
    def crossbar_get_provider_list(self) -> str:
        nmd.ros_node.get_logger().info('Request to [ros.provider.get_list]')
        return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    @wamp.register('ros.nodes.get_list')
    def crossbar_get_node_list(self) -> str:
        nmd.ros_node.get_logger().info('Request to [ros.nodes.get_list]')
        node_list: List[RosNode] = self.to_crossbar()
        return json.dumps(node_list, cls=SelfEncoder)

    @wamp.register('ros.launch.get_list')
    def getList(self) -> List[LaunchContent]:
        Log.debug('Request to [ros.launch.get_list] TO BE IMPLEMENTED')
        reply = []
        return json.dumps(reply, cls=SelfEncoder)

    def _guid_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid.data.tolist())

    def to_crossbar(self) -> List[RosNode]:
        result = []
        if self._ros_state is not None:
            topic_by_id = {}
            topic_objs = {}
            service_by_id = {}
            service_objs = {}
            for rp in self._ros_state.participants:
                for te in rp.topic_entities:
                    t_guid = self._guid_to_str(te.guid)
                    if te.name.startswith('rt/'):
                        if (te.name[2:], te.ttype) not in topic_objs:
                            tp = RosTopic(te.name[2:], te.ttype)
                            topic_objs[(te.name[2:], te.ttype)] = tp
                            topic_by_id[t_guid] = tp
                        else:
                            #topic_objs[(te.name[2:], te.ttype)].guids.append(t_guid)
                            topic_by_id[t_guid] = topic_objs[(te.name[2:], te.ttype)]
                    elif te.name.startswith('rr/'):
                        pass
                for rn in rp.node_entities:
                    n_guid = self._guid_to_str(rp.guid)
                    ros_node = RosNode(n_guid, rn.name)
                    ros_node.name = os.path.join(rn.ns, rn.name)
                    ros_node.namespace = rn.ns
                    for ntp in rn.publisher:
                        try:
                            tp = topic_by_id[self._guid_to_str(ntp)]
                            tp.publisher.append(n_guid)
                            ros_node.publishers.append(tp)
                        except KeyError:
                            pass
                    for nts in rn.subscriber:
                        try:
                            ts = topic_by_id[self._guid_to_str(nts)]
                            ts.subscriber.append(n_guid)
                            ros_node.subscribers.append(ts)
                        except KeyError:
                            pass
                    result.append(ros_node)
        return result

#  class RosNode:
#     def __init__(self, id: str, name: str) -> None:
#         self.id = id
#         self.name = get_node_name(name)
#         self.namespace = get_namespace(name)
#         self.status = 'running'
#         self.pid = -1
#         self.node_API_URI = ''
#         self.masteruri = ''
#         self.location = 'local'
#         self.publishers: List[RosTopic] = []
#         self.subscribers: List[RosTopic] = []
#         self.services: List[RosService] = []
#         self.screens: List[str] = []
#         self.parameters: List[RosParameter] = []

# class RosTopic:
#     def __init__(self, name: str, msgtype: str) -> None:
#         self.name = name
#         self.msgtype = msgtype
#         self.publisher: List[str] = []
#         self.subscriber: List[str] = []
        # try:
        #     iffilter = filter_interface
        #     ros_nodes = dict()
        #     # filter the topics
        #     for name, topic in self.topics.items():
        #         ros_topic = RosTopic(name, topic.type)
        #         for n in topic.publisherNodes:
        #             if not iffilter.is_ignored_publisher(n, name, topic.type):
        #                 ros_topic.publisher.append(n)
        #                 node = ros_nodes.get(n, RosNode(n, n))
        #                 node.publishers.append(ros_topic)
        #                 ros_nodes[n] = node
        #         for n in topic.subscriberNodes:
        #             if not iffilter.is_ignored_subscriber(n, name, topic.type):
        #                 ros_topic.subscriber.append(n)
        #                 node = ros_nodes.get(n, RosNode(n, n))
        #                 node.subscribers.append(ros_topic)
        #                 ros_nodes[n] = node
        #     # filter the services
        #     for name, service in self.services.items():
        #         ros_service = RosService(name, service.type)
        #         for sp in service.serviceProvider:
        #             if not iffilter.is_ignored_service(sp, name):
        #                 ros_service.provider.append(sp)
        #                 node = ros_nodes.get(sp, RosNode(sp, sp))
        #                 node.services.append(ros_service)
        #                 ros_nodes[sp] = node
        #         ros_service.service_API_URI = service.uri
        #         ros_service.masteruri = service.masteruri
        #         ros_service.location = 'local' if service.isLocal else 'remote'

        #     result = []
        #     # creates the nodes list
        #     for name, node in self.nodes.items():
        #         ros_node = ros_nodes.get(name, RosNode(name, name))
        #         ros_node.node_API_URI = node.uri
        #         ros_node.masteruri = node.masteruri
        #         ros_node.pid = node.pid
        #         ros_node.location = 'local' if node.isLocal else 'remote'

        #         # Add active screens for a given node
        #         screens = screen.get_active_screens(name)
        #         for session_name, _ in screens.items():
        #             ros_node.screens.append(session_name)
        #         result.append(ros_node)
        # except Exception:
        #     import traceback
        #     print(traceback.format_exc())
        # return result
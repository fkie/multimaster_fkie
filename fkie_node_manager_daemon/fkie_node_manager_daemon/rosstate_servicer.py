
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
import threading
import time

from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.runtime_interface import RosProvider
from fkie_multimaster_msgs.crossbar.runtime_interface import RosNode
from fkie_multimaster_msgs.crossbar.runtime_interface import RosTopic
from fkie_multimaster_msgs.crossbar.runtime_interface import RosService
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchContent
from fkie_multimaster_msgs.defines import NM_DISCOVERY_NAME
from fkie_multimaster_msgs.defines import NM_NAMESPACE
from fkie_multimaster_msgs.defines import NMD_DEFAULT_PORT
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system.url import get_port
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import DiscoveredState
from fkie_multimaster_msgs.msg import Endpoint

import fkie_node_manager_daemon as nmd


class RosStateServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = NMD_DEFAULT_PORT, test_env=False):
        nmd.ros_node.get_logger().info("Create ros_state servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env)
        self._endpoints = {}  # uri : Endpoint
        self._ros_state = None  # DiscoveredState
        self.topic_name_state = '%s/%s/rosstate' % (
            NM_NAMESPACE, NM_DISCOVERY_NAME)
        self.topic_name_endpoint = '%s/daemons' % (
            NM_NAMESPACE)
        qos_profile = QoSProfile(depth=100,
                                 # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 # history=QoSHistoryPolicy.KEEP_LAST,
                                 # reliability=QoSReliabilityPolicy.RELIABLE)
                                 )
        nmd.ros_node.get_logger().info('listen for discovered items on %s' %
                                       self.topic_name_state)
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            DiscoveredState, self.topic_name_state, self._on_msg_state, qos_profile=qos_profile)
        nmd.ros_node.get_logger().info('listen for endpoint items on %s' %
                                       self.topic_name_endpoint)
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_profile)
        self._checkDiscoveryNodeThread = threading.Thread(
            target=self._check_discovery_node, daemon=True)
        self._checkDiscoveryNodeThread.start()

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        my_provider_name = nmd.launcher.server.name
        for uri, endpoint in endpoints.items():
            origin = my_provider_name == endpoint.ros_name
            provider = RosProvider(
                name=endpoint.ros_name, host=endpoint.name, port=get_port(endpoint.uri), origin=origin)
            result.append(provider)
        return result

    def _crossbar_publish_masters(self):
        result = self._endpoints_to_provider(self._endpoints)
        self.publish_to('ros.provider.list', result)

    def get_publisher_count(self):
        if hasattr(self, 'topic_name_endpoint') and self.topic_name_endpoint is not None:
            return nmd.ros_node.count_publishers(self.topic_name_endpoint)
        return -1

    def _check_discovery_node(self):
        while not self._on_shutdown:
            if self._ros_state is not None:
                if nmd.ros_node.count_publishers(self.topic_name_state) == 0:
                    self._ros_state = None
                    self.publish_to('ros.discovery.ready', {'status': False})
            time.sleep(1)

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
        self.publish_to('ros.discovery.ready', {'status': False})
        self.publish_to('ros.daemon.ready', {'status': False})

    def _on_msg_state(self, msg: DiscoveredState):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.DiscoveredState<XXX>
        '''
        nmd.ros_node.get_logger().info('new message on %s' % self.topic_name_state)
        if self._ros_state is None:
            self.publish_to('ros.discovery.ready', {'status': True})
        self._ros_state = msg
        self.publish_to('ros.nodes.changed', {"timestamp": time.time()})

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

    @wamp.register('ros.nodes.stop_node')
    def stop_node(self, name: str) -> bool:
        Log.info(f"Request to stop node '{name}'")
        return nmd.launcher.server.screen_servicer.kill_node(name.split('|')[-1])

    def _guid_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid.data.tolist())

    def get_type(self, dds_type: str) -> str:
        result = dds_type
        if result:
            result = result.replace('::', '/')
            result = result.replace('/dds_', '')
            # result = result.replace('/msg/dds_', '')
            # result = result.replace('/srv/dds_', '')
            result = result.rstrip('_')
        return result

    def get_service_type(self, dds_service_type: str) -> str:
        result = dds_service_type
        for suffix in ['_Response_', '_Request_']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)+1]
                break
        return self.get_type(result)

    def get_service_name(self, dds_service_name: str) -> str:
        result = dds_service_name
        for suffix in ['Reply', 'Request']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)]
                break
        return self.get_type(result)

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
                            topic_by_id[t_guid] = topic_objs[(
                                te.name[2:], te.ttype)]
                    elif te.name[:2] in ['rr', 'rq', 'rs']:
                        srv_type = self.get_service_type(te.ttype)
                        # TODO: distinction between Reply/Request? Currently it is removed.
                        srv_name = self.get_service_name(te.name[2:])
                        if (srv_name, srv_type) not in service_objs:
                            srv = RosService(srv_name, srv_type)
                            service_objs[(srv_name, srv_type)] = srv
                            service_by_id[t_guid] = srv
                        else:
                            service_by_id[t_guid] = service_objs[(
                                srv_name, srv_type)]
                parent_node = None
                for rn in rp.node_entities:
                    n_guid = self._guid_to_str(rp.guid)
                    full_name = os.path.join(rn.ns, rn.name)
                    Log.info(
                        f"add node: {n_guid}|{full_name}, {rp.enclave}, {rp.unicast_locators}")
                    ros_node = RosNode(f"{n_guid}|{full_name}", rn.name)
                    ros_node.name = full_name
                    ros_node.namespace = rn.ns
                    for ntp in rn.publisher:
                        gid = self._guid_to_str(ntp)
                        try:
                            tp = topic_by_id[gid]
                            tp.publisher.append(n_guid)
                            ros_node.publishers.append(tp)
                        except KeyError:
                            try:
                                srv = service_by_id[gid]
                                srv.provider.append(n_guid)
                                ros_node.services.append(srv)
                            except KeyError:
                                pass
                    for nts in rn.subscriber:
                        gid = self._guid_to_str(nts)
                        try:
                            ts = topic_by_id[gid]
                            ts.subscriber.append(n_guid)
                            ros_node.subscribers.append(ts)
                        except KeyError:
                            try:
                                srv = service_by_id[gid]
                                srv.provider.append(n_guid)
                                ros_node.services.append(srv)
                            except KeyError:
                                pass
                    if parent_node is not None:
                        ros_node.parent_id = parent_node.id
                    else:
                        parent_node = ros_node
                    result.append(ros_node)
        return result

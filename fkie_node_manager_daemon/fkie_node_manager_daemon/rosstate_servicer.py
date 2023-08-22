
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

from typing import Dict
from typing import List
from numbers import Number
from typing import Text
from typing import Union

import os
import json
import asyncio
from autobahn import wamp
import threading
import time

from composition_interfaces.srv import ListNodes
from composition_interfaces.srv import UnloadNode

from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.crossbar.runtime_interface import RosProvider
from fkie_multimaster_pylib.crossbar.runtime_interface import RosNode
from fkie_multimaster_pylib.crossbar.runtime_interface import RosTopic
from fkie_multimaster_pylib.crossbar.runtime_interface import RosService
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchContent
from fkie_multimaster_pylib.defines import NM_DISCOVERY_NAME
from fkie_multimaster_pylib.defines import NM_NAMESPACE
from fkie_multimaster_pylib.defines import NMD_DEFAULT_PORT
from fkie_multimaster_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.names import ns_join
from fkie_multimaster_pylib.system import screen
from fkie_multimaster_pylib.system.host import get_hostname
from fkie_multimaster_pylib.system.url import get_port
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import DiscoveredState
from fkie_multimaster_msgs.msg import ParticipantEntitiesInfo
from fkie_multimaster_msgs.msg import Endpoint

import fkie_node_manager_daemon as nmd


class RosStateServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = NMD_DEFAULT_PORT, test_env=False):
        Log.info("Create ros_state servicer")
        CrossbarBaseSession.__init__(
            self, loop, realm, port, test_env=test_env)
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._ros_state: Dict[str, ParticipantEntitiesInfo] = {}
        self._ros_node_list: List[RosNode] = None
        self.topic_name_state = f"{NM_NAMESPACE}/{NM_DISCOVERY_NAME}/rosstate"
        self.topic_name_endpoint = f"{NM_NAMESPACE}/daemons"
        self.topic_state_publisher_count = 0
        self._ts_state_updated = 0
        self._ts_state_notified = 0
        self._rate_check_discovery_node = 1.0
        self._thread_check_discovery_node = None

    def start(self):
        qos_state_profile = QoSProfile(depth=100
                                       # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                       # history=QoSHistoryPolicy.KEEP_LAST,
                                       # reliability=QoSReliabilityPolicy.RELIABLE
                                       )
        qos_endpoint_profile = QoSProfile(depth=100,
                                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                          # history=QoSHistoryPolicy.KEEP_LAST,
                                          reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info(f"{self.__class__.__name__}: listen for discovered items on {self.topic_name_state}")
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            DiscoveredState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info(f"{self.__class__.__name__}: listen for endpoint items on {self.topic_name_endpoint}")
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        self._thread_check_discovery_node = threading.Thread(
            target=self._check_discovery_node, daemon=True)
        self._thread_check_discovery_node.start()

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        for uri, endpoint in endpoints.items():
            origin = self.uri == uri
            provider = RosProvider(
                name=endpoint.name, host=get_hostname(endpoint.uri), port=get_port(endpoint.uri), origin=origin, hostnames=[get_hostname(endpoint.uri)])
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
            if self.topic_state_publisher_count:
                if nmd.ros_node.count_publishers(self.topic_name_state) == 0:
                    self.publish_to('ros.discovery.ready', {'status': False})
                    self.topic_state_publisher_count = 0
                    self._ts_state_updated = time.time()
                if self._ts_state_updated > self._ts_state_notified:
                    if time.time() - self._ts_state_notified > self._rate_check_discovery_node:
                        self.publish_to('ros.nodes.changed', {
                                        "timestamp": self._ts_state_updated})
                        nmd.launcher.server.screen_servicer.system_change()
                        self._ts_state_notified = self._ts_state_updated
            time.sleep(1.0 / self._rate_check_discovery_node)

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        self._on_shutdown = True
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
        self._ros_node_list = None
        if not self.topic_state_publisher_count:
            self.publish_to('ros.discovery.ready', {'status': True})
            self.topic_state_publisher_count = nmd.ros_node.count_publishers(
                self.topic_name_state)
        self._ros_state = {}
        for participant in msg.participants:
            guid = self._guid_to_str(participant.guid)
            self._ros_state[guid] = participant
        # notify crossbar clients, but not to often
        self._ts_state_updated = time.time()
        if self._ts_state_updated - self._ts_state_notified > self._rate_check_discovery_node:
            self.publish_to('ros.nodes.changed', {
                            "timestamp": self._ts_state_updated})
            nmd.launcher.server.screen_servicer.system_change()
            self._ts_state_notified = self._ts_state_updated

    def _on_msg_endpoint(self, msg: Endpoint):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.Endpoint<XXX>
        '''
        Log.info(f"{self.__class__.__name__}: new message on {self.topic_name_endpoint}")
        is_new = False
        if msg.on_shutdown:
            if msg.uri in self._endpoints:
                is_new = True
                del self._endpoints[msg.uri]
        elif msg.uri in self._endpoints:
            other = self._endpoints[msg.uri]
            is_new = msg.name != other.name
            is_new |= msg.ros_name != other.ros_name
            is_new |= msg.ros_domain_id != other.ros_domain_id
            is_new |= msg.pid != other.pid
        else:
            is_new = True
        if is_new:
            self._endpoints[msg.uri] = msg
            self._crossbar_publish_masters()

    @wamp.register('ros.provider.get_list')
    def crossbar_get_provider_list(self) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.provider.get_list]")
        return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    @wamp.register('ros.nodes.get_list')
    def crossbar_get_node_list(self) -> str:
        Log.info(f"{self.__class__.__name__}: Request to [ros.nodes.get_list]")
        node_list: List[RosNode] = self._get_ros_node_list()

        return json.dumps(node_list, cls=SelfEncoder)

    @wamp.register('ros.nodes.stop_node')
    def stop_node(self, name: str) -> bool:
        Log.info(f"{self.__class__.__name__}: Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        unloaded = False
        result = json.dumps({'result': False, 'message': f'{name} not found'}, cls=SelfEncoder)
        if node is not None:
            if node.parent_id:
                self.stop_composed_node(node)
                unloaded = True
            else:
                result = nmd.launcher.server.screen_servicer.kill_node(os.path.join(node.namespace, node.name))
        if unloaded:
            result = json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
        nmd.launcher.server.screen_servicer.system_change()
        return result

    @wamp.register('ros.subscriber.stop')
    def stop_subscriber(self, topic_name: str) -> bool:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.subscriber.stop]: {str(topic_name)}")
        ns, name = ros2_subscriber_nodename_tuple(topic_name)
        return self.stop_node(os.path.join(ns, name))

    def stop_composed_node(self, node: RosNode) -> None:
        # try to unload node from container
        node_name = ns_join(node.namespace, node.name)
        Log.info(
            f"{self.__class__.__name__}: -> unload '{node_name}' from '{node.parent_id}'")
        container_node: RosNode = self.get_ros_node_by_id(node.parent_id)
        if container_node is not None:
            container_name = ns_join(
                container_node.namespace, container_node.name)
            try:
                node_unique_id = self.get_composed_node_id(
                    container_name, node_name)
            except Exception as err:
                print(f"{self.__class__.__name__}: unload ERR {err}")
                return json.dumps({'result': False, 'message': str(err)}, cls=SelfEncoder)

            service_unload_node = f'{container_name}/_container/unload_node'
            Log.info(
                f"{self.__class__.__name__}:-> unload '{node_name}' with id '{node_unique_id}' from '{service_unload_node}'")

            request = UnloadNode.Request()
            request.unique_id = node_unique_id
            response = nmd.launcher.call_service(
                service_unload_node, UnloadNode, request)
            if not response.success:
                return json.dumps({'result': False, 'message': response.error_message}, cls=SelfEncoder)
        else:
            return json.dumps({'result': False, 'message': '{node.parent_id} not found!'}, cls=SelfEncoder)

    def get_composed_node_id(self, container_name: str, node_name: str) -> Number:
        service_list_nodes = f'{container_name}/_container/list_nodes'
        Log.debug(f"{self.__class__.__name__}: list nodes from '{service_list_nodes}'")
        request_list = ListNodes.Request()
        response_list = nmd.launcher.call_service(
            service_list_nodes, ListNodes, request_list)
        for name, unique_id in zip(response_list.full_node_names, response_list.unique_ids):
            if name == node_name:
                return unique_id
        return -1

    def _get_ros_node_list(self) -> List[RosNode]:
        if self._ros_node_list is None:
            self._ros_node_list = self.to_crossbar()
        return self._ros_node_list

    def get_ros_node(self, node_name: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_name == ns_join(node.namespace, node.name):
                return node
        return None

    def get_ros_node_by_id(self, node_id: str) -> Union[RosNode, None]:
        node_list: List[RosNode] = self._get_ros_node_list()
        for node in node_list:
            if node_id == node.id:
                return node
        return None

    def _guid_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid.data.tolist()[0:12])

    def _guid_arr_to_str(self, guid):
        return '.'.join('{:02X}'.format(c) for c in guid)

    @classmethod
    def get_message_type(cls, dds_type: Text) -> Text:
        result = dds_type
        if result:
            result = result.replace('::', '/')
            result = result.replace('/dds_', '')
            # result = result.replace('/msg/dds_', '')
            # result = result.replace('/srv/dds_', '')
            result = result.rstrip('_')
        return result

    @classmethod
    def get_service_type(cls, dds_service_type: Text) -> Text:
        result = dds_service_type
        for suffix in ['_Response_', '_Request_']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)+1]
                break
        return cls.get_message_type(result)

    @classmethod
    def get_service_name(cls, dds_service_name: Text) -> Text:
        result = dds_service_name
        for suffix in ['Reply', 'Request']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)]
                break
        return cls.get_message_type(result)

    def to_crossbar(self) -> List[RosNode]:
        node_dict = {}
        topic_by_id = {}
        topic_objs = {}
        service_by_id = {}
        service_objs = {}
        Log.debug(f"{self.__class__.__name__}: create graph for crossbar")

        def _get_node_from(node_ns, node_name, node_guid):
            key = (node_ns, node_name, node_guid)
            if key not in node_dict:
                full_name = os.path.join(node_ns, node_name)
                Log.debug(
                    f"{self.__class__.__name__}:   create node: {full_name}")
                ros_node = RosNode(node_guid, full_name)
                # search node with same guid, we asume it is the manager
                for (_ns, _name, _guid), _node in node_dict.items():
                    if node_guid == _guid and _node.parent_id is None:
                        ros_node.parent_id = _node.id

                node_dict[key] = ros_node
                if node_guid in self._ros_state:
                    participant = self._ros_state[node_guid]
                    ros_node.location = participant.unicast_locators
                    Log.debug(
                        f"{self.__class__.__name__}:     set unicast locators: {participant.unicast_locators}")
                    ros_node.namespace = node_ns
                    ros_node.enclave = participant.enclave
                # Add active screens for a given node
                screens = screen.get_active_screens(full_name)
                for session_name, _ in screens.items():
                    Log.debug(
                        f"{self.__class__.__name__}:     append screen: {session_name}")
                    ros_node.screens.append(session_name)
                ros_node.system_node = os.path.basename(
                    full_name).startswith('_') or full_name in ['/rosout']

                return node_dict[key], True
            return node_dict[key], False

        def _get_topic_from(topic_name, topic_type):
            t_guid = self._guid_arr_to_str(pub_info.endpoint_gid)
            if topic_name.startswith('rt/'):
                if (topic_name[2:], topic_type) not in topic_objs:
                    topic_type_res = self.get_message_type(topic_type)
                    Log.debug(
                        f"{self.__class__.__name__}:   create topic {topic_name[2:]} ({topic_type_res})")
                    tp = RosTopic(topic_name[2:], topic_type_res)
                    topic_objs[(topic_name[2:], topic_type)] = tp
                    topic_by_id[t_guid] = tp
                else:
                    topic_by_id[t_guid] = topic_objs[(
                        topic_name[2:], topic_type)]
                return topic_by_id[t_guid], True
            elif topic_name[:2] in ['rr', 'rq', 'rs']:
                srv_type = self.get_service_type(topic_type)
                # TODO: distinction between Reply/Request? Currently it is removed.
                srv_name = self.get_service_name(topic_name[2:])
                if (srv_name, srv_type) not in service_objs:
                    srv_type_res = self.get_service_type(srv_type)
                    Log.debug(
                        f"{self.__class__.__name__}:   create service {srv_name} ({srv_type_res})")
                    srv = RosService(srv_name, srv_type_res)
                    service_objs[(srv_name, srv_type)] = srv
                    service_by_id[t_guid] = srv
                else:
                    service_by_id[t_guid] = service_objs[(
                        srv_name, srv_type)]
                return service_by_id[t_guid], False
        result = []

        topic_list = nmd.ros_node.get_topic_names_and_types(True)
        for topic_name, topic_types in topic_list:
            pub_infos = nmd.ros_node.get_publishers_info_by_topic(
                topic_name, True)
            if pub_infos:
                for pub_info in pub_infos:
                    if '_NODE_NAME_UNKNOWN_' in pub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in pub_info.node_namespace:
                        continue
                    n_guid = self._guid_arr_to_str(pub_info.endpoint_gid[0:12])
                    ros_node, isnew = _get_node_from(
                        pub_info.node_namespace, pub_info.node_name, n_guid)
                    for topic_type in topic_types:
                        tp, istopic = _get_topic_from(topic_name, topic_type)
                        # add tp.qos_profil
                        if istopic:
                            discover_state_publisher = False
                            endpoint_publisher = False
                            Log.debug(
                                f"{self.__class__.__name__}:      add publisher {n_guid} {pub_info.node_namespace}/{pub_info.node_name}")
                            tp.publisher.append(n_guid)
                            ros_node.publishers.append(tp)
                            discover_state_publisher = 'fkie_multimaster_msgs/msg/DiscoveredState' in topic_type
                            endpoint_publisher = 'fkie_multimaster_msgs/msg/Endpoint' in topic_type
                            ros_node.system_node = ros_node.system_node or discover_state_publisher or endpoint_publisher
                        else:
                            if n_guid not in tp.provider:
                                Log.debug(
                                    f"{self.__class__.__name__}:      add provider {n_guid} {sub_info.node_namespace}/{sub_info.node_name}")
                                tp.provider.append(n_guid)
                            ros_node.services.append(tp)

                    if isnew:
                        result.append(ros_node)
            sub_infos = nmd.ros_node.get_subscriptions_info_by_topic(
                topic_name, True)

            if sub_infos:
                for sub_info in sub_infos:
                    if '_NODE_NAME_UNKNOWN_' in sub_info.node_name or '_NODE_NAMESPACE_UNKNOWN_' in sub_info.node_namespace:
                        continue
                    n_guid = self._guid_arr_to_str(sub_info.endpoint_gid[0:12])
                    ros_node, isnew = _get_node_from(
                        sub_info.node_namespace, sub_info.node_name, n_guid)
                    for topic_type in topic_types:
                        tp, istopic = _get_topic_from(topic_name, topic_type)
                        # add tp.qos_profil
                        if istopic:
                            Log.debug(
                                f"{self.__class__.__name__}:      add subscriber {n_guid} {sub_info.node_namespace}/{sub_info.node_name}")
                            tp.subscriber.append(n_guid)
                            ros_node.subscribers.append(tp)
                        else:
                            if n_guid not in tp.provider:
                                Log.debug(
                                    f"{self.__class__.__name__}:      add provider {n_guid} {sub_info.node_namespace}/{sub_info.node_name}")
                                tp.provider.append(n_guid)
                            ros_node.services.append(tp)
                    if isnew:
                        result.append(ros_node)
        return result

    @classmethod
    def get_message_type(cls, dds_type: Text) -> Text:
        result = dds_type
        if result:
            result = result.replace('::', '/')
            result = result.replace('/dds_', '')
            # result = result.replace('/msg/dds_', '')
            # result = result.replace('/srv/dds_', '')
            result = result.rstrip('_')
        return result

    @classmethod
    def get_service_type(cls, dds_service_type: Text) -> Text:
        result = dds_service_type
        for suffix in ['_Response_', '_Request_']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)+1]
                break
        return cls.get_message_type(result)

    @classmethod
    def get_service_name(cls, dds_service_name: Text) -> Text:
        result = dds_service_name
        for suffix in ['Reply', 'Request']:
            if result[-len(suffix):] == suffix:
                result = result[:-len(suffix)]
                break
        return cls.get_message_type(result)

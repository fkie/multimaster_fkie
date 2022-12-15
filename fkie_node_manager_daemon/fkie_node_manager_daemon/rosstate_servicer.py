
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
from typing import Union

import os
import json
import asyncio
from autobahn import wamp
import threading
import time

from composition_interfaces.srv import ListNodes
from composition_interfaces.srv import UnloadNode

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
from fkie_multimaster_msgs.names import ns_join
from fkie_multimaster_msgs.system import screen
from fkie_multimaster_msgs.system.host import get_hostname
from fkie_multimaster_msgs.system.url import get_port
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import DiscoveredState
from fkie_multimaster_msgs.msg import ParticipantEntitiesInfo
from fkie_multimaster_msgs.msg import Endpoint

import fkie_node_manager_daemon as nmd


class RosStateServicer(CrossbarBaseSession):

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = NMD_DEFAULT_PORT, test_env=False):
        Log.info("Create ros_state servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env)
        self._endpoints: Dict[str, Endpoint] = {}  # uri : Endpoint
        self._ros_state: Dict[str, ParticipantEntitiesInfo] = {}
        self._ros_node_list: List[RosNode] = None
        self.topic_name_state = '%s/%s/rosstate' % (
            NM_NAMESPACE, NM_DISCOVERY_NAME)
        self.topic_name_endpoint = '%s/daemons' % (
            NM_NAMESPACE)
        qos_state_profile = QoSProfile(depth=100,
                                       # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                       # history=QoSHistoryPolicy.KEEP_LAST,
                                       # reliability=QoSReliabilityPolicy.RELIABLE)
                                       )
        qos_endpoint_profile = QoSProfile(depth=100,
                                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                          # history=QoSHistoryPolicy.KEEP_LAST,
                                          reliability=QoSReliabilityPolicy.RELIABLE)
        Log.info('listen for discovered items on %s' %
                 self.topic_name_state)
        self.sub_discovered_state = nmd.ros_node.create_subscription(
            DiscoveredState, self.topic_name_state, self._on_msg_state, qos_profile=qos_state_profile)
        Log.info('listen for endpoint items on %s' %
                 self.topic_name_endpoint)
        self.sub_endpoints = nmd.ros_node.create_subscription(
            Endpoint, self.topic_name_endpoint, self._on_msg_endpoint, qos_profile=qos_endpoint_profile)
        self._checkDiscoveryNodeThread = threading.Thread(
            target=self._check_discovery_node, daemon=True)
        self._checkDiscoveryNodeThread.start()

    def _endpoints_to_provider(self, endpoints) -> List[RosProvider]:
        result = []
        for uri, endpoint in endpoints.items():
            origin = self.uri == uri
            provider = RosProvider(
                name=endpoint.name, host=get_hostname(endpoint.uri), port=get_port(endpoint.uri), origin=origin)
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
            if self._ros_state:
                if nmd.ros_node.count_publishers(self.topic_name_state) == 0:
                    self._ros_state = {}
                    self.publish_to('ros.discovery.ready', {'status': False})
            elif nmd.ros_node.count_publishers(self.topic_name_state) > 0:
                nmd.launcher.server.pub_endpoint.publish(
                    nmd.launcher.server._endpoint_msg)
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
        Log.info('new message on %s' % self.topic_name_state)
        self._ros_node_list = None
        if not self._ros_state:
            if msg.full_state:
                Log.info('  full update')
                self._ros_state = {}
                self.publish_to('ros.discovery.ready', {'status': True})
                for participant in msg.participants:
                    guid = self._guid_to_str(participant.guid)
                    self._ros_state[guid] = participant
                self.publish_to('ros.nodes.changed', {"timestamp": time.time()})
                nmd.launcher.server.screen_servicer.system_change()
        else:
            if msg.full_state:
                # update all nodes on full state
                Log.info('  full update')
                self._ros_state = {}
            else:
                Log.info('  partially update')
            for participant in msg.participants:
                guid = self._guid_to_str(participant.guid)
                Log.info(f"    add participant {guid}")
                self._ros_state[guid] = participant
            for gid in msg.removed_participants:
                try:
                    r_gid = self._guid_to_str(gid)
                    Log.info(f"    removed participant {r_gid}")
                    del self._ros_state[r_gid]
                except Exception as err:
                    Log.warn(f"error while remove participant: {err}")
            self.publish_to('ros.nodes.changed', {"timestamp": time.time()})
            nmd.launcher.server.screen_servicer.system_change()

    def _on_msg_endpoint(self, msg: Endpoint):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.Endpoint<XXX>
        '''
        Log.info('new message on %s' % self.topic_name_endpoint)
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
        Log.info('Request to [ros.provider.get_list]')
        return json.dumps(self._endpoints_to_provider(self._endpoints), cls=SelfEncoder)

    @wamp.register('ros.nodes.get_list')
    def crossbar_get_node_list(self) -> str:
        Log.info('Request to [ros.nodes.get_list]')
        node_list: List[RosNode] = self._get_ros_node_list()

        return json.dumps(node_list, cls=SelfEncoder)

    @wamp.register('ros.nodes.stop_node')
    def stop_node(self, name: str) -> bool:
        Log.info(f"Request to stop node '{name}'")
        node: RosNode = self.get_ros_node(name)
        if node is None:
            node = self.get_ros_node_by_id(name)
        unloaded = False
        if node is not None:
            if node.parent_id:
                self.stop_composed_node(node)
                unloaded = True
        if unloaded:
            return json.dumps({'result': True, 'message': ''}, cls=SelfEncoder)
        nmd.launcher.server.screen_servicer.system_change()
        return nmd.launcher.server.screen_servicer.kill_node(name.split('|')[-1])

    def stop_composed_node(self, node: RosNode) -> None:
        # try to unload node from container
        node_name = ns_join(node.namespace, node.name)
        Log.info(
            f"-> unload '{node_name}' from '{node.parent_id}'")
        container_node: RosNode = self.get_ros_node_by_id(node.parent_id)
        if container_node is not None:
            container_name = ns_join(
                container_node.namespace, container_node.name)
            try:
                node_unique_id = self.get_composed_node_id(
                    container_name, node_name)
            except Exception as err:
                print("unload ERR", err)
                return json.dumps({'result': False, 'message': str(err)}, cls=SelfEncoder)

            service_unload_node = f'{container_name}/_container/unload_node'
            Log.info(
                f"-> unload '{node_name}' with id '{node_unique_id}' from '{service_unload_node}'")

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
        Log.debug(f"-> list nodes from '{service_list_nodes}'")
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
        if self._ros_state:
            topic_by_id = {}
            topic_objs = {}
            service_by_id = {}
            service_objs = {}
            for p_guid, participant in self._ros_state.items():
                # location = ''
                # for loc in rp.unicast_locators:
                #     if '127.0.0.' in loc or 'SHM' in loc:
                #         location = 'local'
                # if not location:
                #     location = rp.unicast_locators
                for te in participant.topic_entities:
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
                for rn in participant.node_entities:
                    full_name = os.path.join(rn.ns, rn.name)
                    Log.info(
                        f"add node: {p_guid}|{full_name}, {participant.enclave}, {participant.unicast_locators}")
                    ros_node = RosNode(f"{p_guid}|{full_name}", rn.name)
                    discover_state_publisher = False
                    endpoint_publisher = False
                    ros_node.location = participant.unicast_locators
                    ros_node.name = full_name
                    ros_node.namespace = rn.ns
                    for ntp in rn.publisher:
                        gid = self._guid_to_str(ntp)
                        try:
                            tp = topic_by_id[gid]
                            tp.publisher.append(p_guid)
                            ros_node.publishers.append(tp)
                            discover_state_publisher = tp.msgtype == 'fkie_multimaster_msgs::msg::dds_::DiscoveredState_'
                            endpoint_publisher = tp.msgtype == 'fkie_multimaster_msgs::msg::dds_::Endpoint_'
                        except KeyError:
                            try:
                                srv = service_by_id[gid]
                                srv.provider.append(p_guid)
                                ros_node.services.append(srv)
                            except KeyError:
                                pass
                    for nts in rn.subscriber:
                        gid = self._guid_to_str(nts)
                        try:
                            ts = topic_by_id[gid]
                            ts.subscriber.append(p_guid)
                            ros_node.subscribers.append(ts)
                        except KeyError:
                            try:
                                srv = service_by_id[gid]
                                srv.provider.append(p_guid)
                                ros_node.services.append(srv)
                            except KeyError:
                                pass
                    if parent_node is not None:
                        ros_node.parent_id = parent_node.id
                    else:
                        parent_node = ros_node
                    # Add active screens for a given node
                    screens = screen.get_active_screens(full_name)
                    for session_name, _ in screens.items():
                        print("APPEND SCREEN:", session_name)
                        ros_node.screens.append(session_name)
                    ros_node.system_node = discover_state_publisher or endpoint_publisher or os.path.basename(
                        full_name).startswith('_') or full_name in ['/rosout']
                    result.append(ros_node)
        return result


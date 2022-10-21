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

from typing import Text
from concurrent import futures
import os
import rclpy
import time
import threading

# crossbar-io dependencies
import asyncio

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import Endpoint
from fkie_multimaster_msgs.srv import LoadLaunch, Task
from fkie_multimaster_msgs.names import ns_join
import fkie_node_manager_daemon as nmd

#import fkie_node_manager_daemon.security as security
from .common import interpret_path
from .host import get_host_name
from .file_servicer import FileServicer
from .launch_servicer import LaunchServicer
from .monitor_servicer import MonitorServicer
from .rosstate_servicer import RosStateServicer
from .screen_servicer import ScreenServicer
from .settings_servicer import SettingsServicer
from .url import nmduri, nmdport
#from .version_servicer import VersionServicer
from .parameter_servicer import ParameterServicer


class Server:

    def __init__(self, rosnode, *, default_domain_id=-1):
        self.rosnode = rosnode
        self.crossbar_port = nmdport()
        self.crossbar_realm = "ros"
        self.crossbar_loop = asyncio.get_event_loop()
        self.ros_domain_id = default_domain_id
        if self.ros_domain_id > 0:
            rosnode.get_logger().warn('default ROS_DOMAIN_ID=%d overwritten to %d' %
                                      (0, self.ros_domain_id))
        self.name = get_host_name()
        self.uri = ''
        self.settings_servicer = SettingsServicer()
        self.monitor_servicer = MonitorServicer(
            self.settings_servicer.settings)
        self.file_servicer = FileServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port)
        self.screen_servicer = ScreenServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port)
        self.rosstate_servicer = RosStateServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port)
        self.parameter_servicer = ParameterServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port)
        self.launch_servicer = LaunchServicer(
            self.crossbar_loop, self.crossbar_realm, self.crossbar_port, ros_domain_id=self.ros_domain_id)

        # self.launch_servicer = LaunchServicer(ros_domain_id=self.ros_domain_id, self.monitor_servicer, self.crossbar_loop, self.crossbar_realm, self.crossbar_port)
        #self.launch_servicer = LaunchServicer(ros_domain_id=self.ros_domain_id)
        rosnode.create_service(LoadLaunch, '~/start_launch',
                               self._rosservice_start_launch)
        rosnode.create_service(LoadLaunch, '~/load_launch',
                               self._rosservice_load_launch)
        rosnode.create_service(Task, '~/run', self._rosservice_start_node)
        qos_profile = QoSProfile(depth=10,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 # history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_endpoint = rosnode.create_publisher(
            Endpoint, 'daemons', qos_profile=qos_profile)
        self.rosname = ns_join(
            nmd.ros_node.get_namespace(), nmd.ros_node.get_name())

    def __del__(self):
        self.crossbar_loop.stop()
        self.launch_servicer = None
        self.monitor_servicer = None
        self.screen_servicer = None
        self.settings_servicer = None
        self.rosstate_servicer = None
        self.parameter_servicer = None

    def start(self, uri: Text, displayed_name: Text = '') -> bool:
        if displayed_name:
            self.name = displayed_name
        nmd.ros_node.get_logger().info("Connect to crossbar server on %s" % uri)
        # self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        port = self.crossbar_port
        # security.init_keys(nmd.ros_node)
        # # create credentials
        # has_credentials = security.has_keys()
        # if has_credentials:
        #     try:
        #         # we use client authentification
        #         server_credentials = grpc.ssl_server_credentials((security.get_keys(),), root_certificates=security.get_ca_cert(), require_client_auth=True)
        #         port = self.server.add_secure_port(uri, server_credentials)
        #         while port == 0 and rclpy.ok():
        #             nmd.ros_node.get_logger().warn("can not add secure channel to '%s', try again..." % uri)
        #             time.sleep(2.)
        #             port = self.server.add_secure_port(uri, server_credentials)
        #     except Exception as err:
        #         nmd.ros_node.get_logger().error('Error while create secure channel: %s' % err)
        #         return False
        # elif not security.is_strict_mode():
        #     # create insecure channel
        #     port = self.server.add_insecure_port(uri)
        #     while port == 0 and rclpy.ok():
        #         nmd.ros_node.get_logger().warn("can not add insecure channel to '%s', try again..." % uri)
        #         time.sleep(2.)
        #         port = self.server.add_insecure_port(uri)
        # else:
        #     return False
        # if port > 0:
        # fgrpc.add_FileServiceServicer_to_server(FileServicer(), self.server)
        # lgrpc.add_LaunchServiceServicer_to_server(self.launch_servicer, self.server)
        # mgrpc.add_MonitorServiceServicer_to_server(self.monitor_servicer, self.server)
        # sgrpc.add_ScreenServiceServicer_to_server(ScreenServicer(), self.server)
        # stgrpc.add_SettingsServiceServicer_to_server(self.settings_servicer, self.server)
        # vgrpc.add_VersionServiceServicer_to_server(VersionServicer(), self.server)
        # self.server.start()
        # nmd.ros_node.get_logger().info("Server at '%s' started using %s channel!" % (uri, 'secure' if has_credentials else 'insecure'))
        # else:
        # return False
        # update name if port is not a default one
        self.insecure_port = port
        if nmdport() != port:
            self.name = "%s_%d" % (self.name, port)
        self.uri = self.rosstate_servicer.uri  # nmduri('grpc://%s' % uri)
        endpoint_msg = Endpoint(name=get_host_name(), uri=self.uri, ros_name=get_host_name(
        ), ros_domain_id=self.ros_domain_id, on_shutdown=False, pid=os.getpid())
        self.pub_endpoint.publish(endpoint_msg)
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(self.crossbar_loop,), daemon=True)
        self._crossbarThread.start()
        return True

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def shutdown(self):
        endpoint_msg = Endpoint(name=get_host_name(), uri=self.uri, ros_name=get_host_name(
        ), ros_domain_id=self.ros_domain_id, on_shutdown=True, pid=os.getpid())
        self.pub_endpoint.publish(endpoint_msg)
        self.screen_servicer.stop()
        self.launch_servicer.stop()
        self.file_servicer.stop()
        self.monitor_servicer.stop()
        self.rosstate_servicer.stop()
        self.rosnode.destroy_publisher(self.pub_endpoint)

    def load_launch_file(self, path, autostart=False):
        pass
        #self.launch_servicer.load_launch_file(interpret_path(path), autostart)

    def _rosservice_start_launch(self, request):
        nmd.ros_node.get_logger().info("Service request to load and start %s" % request.path)
        #self.launch_servicer.load_launch_file(interpret_path(request.path), True)
        return LoadLaunch.Response()

    def _rosservice_load_launch(self, request):
        nmd.ros_node.get_logger().info("Service request to load %s" % request.path)
        self.launch_servicer.load_launch_file(
            interpret_path(request.path), False)
        return LoadLaunch.Response()

    def _rosservice_start_node(self, req):
        '''
        Callback for the ROS service to start a node.
        '''
        self.launch_servicer.start_node(req.node)
        return []

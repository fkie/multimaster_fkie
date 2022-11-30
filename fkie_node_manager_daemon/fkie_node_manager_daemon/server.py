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
import os
import threading
import time

# crossbar-io dependencies
import asyncio
import json

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import Endpoint
from fkie_multimaster_msgs.srv import LoadLaunch, Task
from fkie_multimaster_msgs.launch import xml
from fkie_multimaster_msgs.names import ns_join
from fkie_multimaster_msgs.crossbar import server
from fkie_multimaster_msgs.system.host import get_host_name
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.logging.logging import Log
import fkie_node_manager_daemon as nmd

from fkie_node_manager_daemon.file_servicer import FileServicer
from fkie_node_manager_daemon.launch_servicer import LaunchServicer
from fkie_node_manager_daemon.monitor_servicer import MonitorServicer
from fkie_node_manager_daemon.parameter_servicer import ParameterServicer
from fkie_node_manager_daemon.rosstate_servicer import RosStateServicer
from fkie_node_manager_daemon.screen_servicer import ScreenServicer


class Server:

    def __init__(self, rosnode, *, default_domain_id=-1):
        self.rosnode = rosnode
        self.crossbar_port = server.port()
        self.crossbar_realm = "ros"
        self.crossbar_loop = asyncio.get_event_loop()
        self.ros_domain_id = default_domain_id
        if self.ros_domain_id > 0:
            rosnode.get_logger().warn('default ROS_DOMAIN_ID=%d overwritten to %d' %
                                      (0, self.ros_domain_id))
        self.name = get_host_name()
        self.uri = ''
        # self.monitor_servicer = MonitorServicer(
        #     self.settings_servicer.settings)
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
        self.rosstate_servicer = None
        self.parameter_servicer = None

    def start(self, uri: Text, displayed_name: Text = '') -> bool:
        if displayed_name:
            self.name = displayed_name
        nmd.ros_node.get_logger().info("Connect to crossbar server on %s" % uri)
        port = self.crossbar_port
        # update name if port is not a default one
        self.insecure_port = port
        if server.port() != port:
            self.name = "%s_%d" % (self.name, port)
        self.uri = self.rosstate_servicer.uri.replace(
            'localhost', get_host_name())
        endpoint_msg = Endpoint(name=get_host_name(), uri=self.uri, ros_name=ns_join(nmd.ros_node.get_namespace(), nmd.ros_node.get_name(
        )), ros_domain_id=self.ros_domain_id, on_shutdown=False, pid=os.getpid())
        self.pub_endpoint.publish(endpoint_msg)
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(self.crossbar_loop,), daemon=True)
        self._crossbarThread.start()
        self._crossbarNotificationThread = threading.Thread(
            target=self._crossbar_notify_if_regsitered, daemon=True)
        self._crossbarNotificationThread.start()
        return True

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _crossbar_notify_if_regsitered(self):
        registration_finished = False
        while not registration_finished:
            registration_finished = True
            registration_finished &= self.launch_servicer.crossbar_registered
            registration_finished &= self.screen_servicer.crossbar_registered
            registration_finished &= self.rosstate_servicer.crossbar_registered
            registration_finished &= self.parameter_servicer.crossbar_registered
            time.sleep(0.5)
        self._crossbar_send_status(True)

    def _crossbar_send_status(self, status: bool):
        # try to send notification to crossbar subscribers
        self.rosstate_servicer.publish_to(
            'ros.daemon.ready', {'status': status})

    def shutdown(self):
        WAIT_TIMEOUT = 3
        self._crossbar_send_status(False)
        endpoint_msg = Endpoint(name=get_host_name(), uri=self.uri, ros_name=get_host_name(
        ), ros_domain_id=self.ros_domain_id, on_shutdown=True, pid=os.getpid())
        self.pub_endpoint.publish(endpoint_msg)
        self.screen_servicer.stop()
        self.launch_servicer.stop()
        self.file_servicer.stop()
        # self.monitor_servicer.stop()
        self.rosstate_servicer.stop()
        self.screen_servicer.stop()
        self.parameter_servicer.stop()
        shutdown_task = self.crossbar_loop.create_task(
            self.crossbar_loop.shutdown_asyncgens())
        self.rosnode.destroy_publisher(self.pub_endpoint)
        sleep_time = 0.5
        while not shutdown_task.done() or self.parameter_servicer.crossbar_connected:
            time.sleep(sleep_time)
            sleep_time += 0.5
            if sleep_time > WAIT_TIMEOUT:
                break

    def load_launch_file(self, path, autostart=False):
        pass
        #self.launch_servicer.load_launch_file(xml.interpret_path(path), autostart)

    def _rosservice_start_launch(self, request):
        nmd.ros_node.get_logger().info("Service request to load and start %s" % request.path)
        #self.launch_servicer.load_launch_file(xml.interpret_path(request.path), True)
        return LoadLaunch.Response()

    def _rosservice_load_launch(self, request):
        nmd.ros_node.get_logger().info("Service request to load %s" % request.path)
        self.launch_servicer.load_launch_file(
            xml.interpret_path(request.path), False)
        return LoadLaunch.Response()

    def _rosservice_start_node(self, req):
        '''
        Callback for the ROS service to start a node.
        '''
        self.launch_servicer.start_node(req.node)
        return []

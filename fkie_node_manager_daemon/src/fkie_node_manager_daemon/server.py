# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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


from concurrent import futures
import grpc
import json
import rospy
import os
import threading
import time

from fkie_multimaster_msgs.srv import ListNodes
from fkie_multimaster_msgs.srv import ListNodesResponse
from fkie_multimaster_msgs.srv import LoadLaunch
from fkie_multimaster_msgs.srv import LoadLaunchResponse
from fkie_multimaster_msgs.srv import Task
import fkie_multimaster_msgs.grpc.file_pb2_grpc as fgrpc
import fkie_multimaster_msgs.grpc.launch_pb2_grpc as lgrpc
import fkie_multimaster_msgs.grpc.monitor_pb2_grpc as mgrpc
import fkie_multimaster_msgs.grpc.screen_pb2_grpc as sgrpc
import fkie_multimaster_msgs.grpc.settings_pb2_grpc as stgrpc
import fkie_multimaster_msgs.grpc.version_pb2_grpc as vgrpc
from fkie_multimaster_pylib.defines import GRPC_SERVER_PORT_OFFSET
from fkie_multimaster_pylib.launch import xml
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.crossbar import server
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder


# crossbar-io dependencies
import asyncio

from .file_servicer import FileServicer
from .launch_servicer import LaunchServicer
from .monitor_servicer import MonitorServicer
from .screen_servicer import ScreenServicer
from .settings_servicer import SettingsServicer
from .version_servicer import VersionServicer
from .parameter_servicer import ParameterServicer


class GrpcServer:
    def __init__(self, test_env=False):
        self.crossbar_port = server.port()
        self.crossbar_realm = "ros"
        self.crossbar_loop = asyncio.get_event_loop()
        self.server = None
        self._test_env = test_env
        self.settings_servicer = SettingsServicer()
        self._grpc_verbosity = self.settings_servicer.settings.param(
            "global/grpc_verbosity", "INFO"
        )
        self._grpc_poll_strategy = self.settings_servicer.settings.param(
            "global/grpc_poll_strategy", ""
        )
        self.settings_servicer.settings.add_reload_listener(self._update_grpc_parameter)
        self.monitor_servicer = MonitorServicer(
            self.settings_servicer.settings,
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.launch_servicer = LaunchServicer(
            self.monitor_servicer,
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.parameter_servicer = ParameterServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.file_servicer = FileServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.screen_servicer = ScreenServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.version_servicer = VersionServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )

        rospy.Service("~start_launch", LoadLaunch, self._rosservice_start_launch)
        rospy.Service("~load_launch", LoadLaunch, self._rosservice_load_launch)
        rospy.Service("~run", Task, self._rosservice_start_node)
        rospy.Service("~list_nodes", ListNodes, self._rosservice_list_nodes)

    def __del__(self):
        self.version_servicer = None
        self.launch_servicer = None
        self.monitor_servicer = None
        self.settings_servicer = None
        self.parameter_servicer = None
        self.file_servicer = None
        self.screen_servicer = None
        self.crossbar_loop.stop()
        self.server.stop(3)

    def _update_grpc_parameter(self, settings):
        old_verbosity = self._grpc_verbosity
        old_strategy = self._grpc_poll_strategy
        self._grpc_verbosity = settings.param("global/grpc_verbosity", "INFO")
        os.environ["GRPC_VERBOSITY"] = self._grpc_verbosity
        Log.info("use GRPC_VERBOSITY=%s" % self._grpc_verbosity)
        self._grpc_poll_strategy = settings.param("global/grpc_poll_strategy", "")
        if self._grpc_poll_strategy:
            os.environ["GRPC_ENABLE_FORK_SUPPORT"] = "1"
            os.environ["GRPC_POLL_STRATEGY"] = self._grpc_poll_strategy
            Log.info("use GRPC_POLL_STRATEGY=%s" % self._grpc_poll_strategy)
        else:
            try:
                del os.environ["GRPC_POLL_STRATEGY"]
            except Exception:
                pass
            os.environ["GRPC_ENABLE_FORK_SUPPORT"] = "0"
        if (
            old_verbosity != self._grpc_verbosity
            or old_strategy != self._grpc_poll_strategy
        ):
            Log.info(
                "gRPC verbosity or poll strategy changed: trigger restart grpc server on %s"
                % self._launch_url
            )
            restart_timer = threading.Timer(1.0, self.restart)
            restart_timer.start()

    def restart(self):
        Log.info("Invoke restart...")
        self.crossbar_loop.stop()
        self.shutdown()
        del self.server
        self.monitor_servicer = MonitorServicer(self.settings_servicer.settings)
        self.launch_servicer = LaunchServicer(
            self.monitor_servicer,
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.parameter_servicer = ParameterServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.file_servicer = FileServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.screen_servicer = ScreenServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.version_servicer = VersionServicer(
            self.crossbar_loop,
            self.crossbar_realm,
            self.crossbar_port,
            test_env=self._test_env,
        )
        self.start(self._launch_url)

    def start(self, url="[::]:12311"):
        self._launch_url = url
        Log.info("Start grpc server on %s" % url)
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        # create credentials
        # read in key and certificate
        #         with open('/home/tiderko/grpc_cert/server.key', 'rb') as f:
        #             private_key = f.read()
        #         with open('/home/tiderko/grpc_cert/server.crt', 'rb') as f:
        #             certificate_chain = f.read()
        #         # create server credentials
        #         server_credentials = grpc.ssl_server_credentials(((private_key, certificate_chain,),))
        #         print("port: ", self.server.add_secure_port(url, server_credentials))
        insecure_port = self.server.add_insecure_port(url)
        while insecure_port == 0 and not rospy.is_shutdown():
            Log.warn("can not add insecure channel to '%s', try again..." % url)
            time.sleep(2.0)
            insecure_port = self.server.add_insecure_port(url)
        if insecure_port > 0:
            fgrpc.add_FileServiceServicer_to_server(self.file_servicer, self.server)
            lgrpc.add_LaunchServiceServicer_to_server(self.launch_servicer, self.server)
            mgrpc.add_MonitorServiceServicer_to_server(
                self.monitor_servicer, self.server
            )
            sgrpc.add_ScreenServiceServicer_to_server(self.screen_servicer, self.server)
            stgrpc.add_SettingsServiceServicer_to_server(
                self.settings_servicer, self.server
            )
            vgrpc.add_VersionServiceServicer_to_server(
                self.version_servicer, self.server
            )
            self.server.start()
            Log.info("Server at '%s' started!" % url)
        # Log.info(f"Connect to crossbar server @ ws://localhost:{self.crossbar_port}/ws, realm: {self.crossbar_realm}")
        self._crossbarThread = threading.Thread(
            target=self.run_crossbar_forever, args=(self.crossbar_loop,), daemon=True
        )
        self._crossbarThread.start()
        self._crossbarNotificationThread = threading.Thread(
            target=self._crossbar_notify_if_regsitered, daemon=True
        )
        self._crossbarNotificationThread.start()

    def run_crossbar_forever(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _crossbar_notify_if_regsitered(self):
        registration_finished = False
        while not registration_finished:
            registration_finished = True
            registration_finished &= self.launch_servicer.crossbar_registered
            registration_finished &= self.screen_servicer.crossbar_registered
            registration_finished &= self.file_servicer.crossbar_registered
            registration_finished &= self.parameter_servicer.crossbar_registered
            registration_finished &= self.version_servicer.crossbar_registered
            time.sleep(0.5)
        self._crossbar_send_status(True)

    def _crossbar_send_status(self, status: bool):
        # try to send notification to crossbar subscribers
        self.launch_servicer.publish_to("ros.daemon.ready", {"status": status})

    def shutdown(self):
        WAIT_TIMEOUT = 3
        self._crossbar_send_status(False)
        shutdown_task = self.crossbar_loop.create_task(
            self.crossbar_loop.shutdown_asyncgens()
        )
        self.version_servicer.stop()
        self.launch_servicer.stop()
        self.monitor_servicer.stop()
        self.parameter_servicer.shutdown()
        self.file_servicer.shutdown()
        self.screen_servicer.stop()
        self.server.stop(WAIT_TIMEOUT)
        sleep_time = 0.5
        while not shutdown_task.done() or self.screen_servicer.crossbar_connected:
            time.sleep(sleep_time)
            sleep_time += 0.5
            if sleep_time > WAIT_TIMEOUT:
                print("break")
                break

    def load_launch_file(self, path, autostart=False):
        self.launch_servicer.load_launch_file(xml.interpret_path(path), autostart)

    def _rosservice_start_launch(self, request):
        Log.info("Service request to load and start %s" % request.path)
        self.launch_servicer.load_launch_file(xml.interpret_path(request.path), True)
        return LoadLaunchResponse()

    def _rosservice_load_launch(self, request):
        Log.info("Service request to load %s" % request.path)
        self.launch_servicer.load_launch_file(xml.interpret_path(request.path), False)
        return LoadLaunchResponse()

    def _rosservice_start_node(self, req):
        """
        Callback for the ROS service to start a node.
        """
        self.launch_servicer.start_node_by_name(req.node)
        return []

    def _rosservice_list_nodes(self, req):
        """
        Callback for the ROS service to list all nodes loaded by all launch files.
        """
        return ListNodesResponse(self.launch_servicer.list_nodes())

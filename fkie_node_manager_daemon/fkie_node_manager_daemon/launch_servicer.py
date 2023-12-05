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


import os
import re
import rclpy
import shlex
import sys
import traceback

from launch.launch_context import LaunchContext
import asyncio

import json
from types import SimpleNamespace
import asyncio
from autobahn import wamp

from typing import Dict
from typing import List
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from watchdog.events import FileSystemEvent
import rosidl_parser.definition
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py import set_message_fields

from fkie_multimaster_pylib import ros_pkg
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.crossbar.runtime_interface import SubscriberNode
from fkie_multimaster_pylib.crossbar.runtime_interface import RosNode
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchArgument
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchCallService
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchFile
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchLoadRequest
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchLoadReply
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchContent
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchAssociations
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchNode
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchNodeInfo
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchNodeReply
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchInterpretPathRequest
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchInterpretPathReply
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchIncludedFilesRequest
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchIncludedFile
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchMessageStruct
from fkie_multimaster_pylib.crossbar.launch_interface import LaunchPublishMessage
from fkie_multimaster_pylib.defines import NM_NAMESPACE
from fkie_multimaster_pylib.defines import ros2_subscriber_nodename_tuple
from fkie_multimaster_pylib.defines import SEARCH_IN_EXT
from fkie_multimaster_pylib.launch import xml
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.names import ns_join
from fkie_multimaster_pylib.system import exceptions
from fkie_multimaster_pylib.system import screen
from fkie_multimaster_pylib.system.host import is_local
from fkie_multimaster_pylib.system.url import equal_uri
from fkie_multimaster_pylib.system.supervised_popen import SupervisedPopen

from . import launcher
from .launch_config import LaunchConfig
from .launch_validator import LaunchValidator

import fkie_node_manager_daemon as nmd


class CfgId(object):
    '''
    Identification object for a loaded launch file. You can load the same launch file for different ROS-Master!
    '''

    def __init__(self, path: str, daemonuri: str = ''):
        '''
        :param str path: absolute path of the launch file.
        :param str daemonuri: daemon where to launch the configuration
        '''
        self.path = path
        self.daemonuri = daemonuri
        self._local = is_local(daemonuri)

    def __str__(self):
        return "%s%s" % (self.daemonuri, self.path)

    def __repr__(self):
        return "%s%s" % (self.daemonuri, self.path)

    def __hash__(self, *args, **kwargs):
        return hash("%s%s" % (self.daemonuri, self.path))

    def __eq__(self, other):
        '''
        Compares the path of the item.
        '''
        if isinstance(other, tuple):
            return self.path == other[0] and self.equal_hosts(other[1])
        elif other is not None:
            return self.path == other.path and self.equal_hosts(other.daemonuri)
        return False

    def __ne__(self, other):
        return not (self == other)

    def equal_hosts(self, daemonuri: str):
        '''
        Compares the daemonuri names of this instance with other host.

        :param str daemonuri: uri of other daemon
        '''
        if not daemonuri:
            if self._local:
                return True
        if equal_uri(self.daemonuri, daemonuri):
            return True
        return False


class LaunchServicer(CrossbarBaseSession, LoggingEventHandler):
    '''
    Handles GRPC-requests defined in `launch.proto`.
    '''

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, ros_domain_id=-1):
        Log.info("Create ROS2 launch servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        LoggingEventHandler.__init__(self)
        self._watchdog_observer = Observer()
        self.__launch_context = LaunchContext(argv=sys.argv[1:])
        self.__launch_context._set_asyncio_loop(asyncio.get_event_loop())
        self.ros_domain_id = ros_domain_id
        self.xml_validator = LaunchValidator()
        self._observed_dirs = {}  # path: watchdog.observers.api.ObservedWatch
        self._included_files = []
        self._included_dirs = []
        self._is_running = True
        self._peers = {}
        self._loaded_files: Dict[CfgId, LaunchConfig] = dict()
        self._observed_launch_files: Dict[str, List[str]] = {}
        self._watchdog_observer.start()

    def _terminated(self):
        Log.info(f"{self.__class__.__name__}: terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            Log.info(
                f"{self.__class__.__name__}: Add callback to peer context @{context.peer()}")
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        '''
        Stop watchdog and cancel the autostart of the nodes.
        '''
        self._is_running = False
        self.shutdown()
        self._watchdog_observer.stop()

    def on_any_event(self, event: FileSystemEvent):
        if event.src_path in self._included_files:
            affected_launch_files = []
            for launch_path, path_list in self._observed_launch_files.items():
                if event.src_path in path_list:
                    affected_launch_files.append(launch_path)
            change_event = {"eventType": event.event_type,
                            "srcPath": event.src_path,
                            "affected": affected_launch_files}
            Log.debug(
                f"{self.__class__.__name__}: observed change {event.event_type} on {event.src_path}")
            self.publish_to('ros.path.changed', change_event)

    def _add_file_to_observe(self, path: str):
        directory = os.path.dirname(path)
        Log.debug(f"{self.__class__.__name__}:observe path: {path}")
        if directory not in self._observed_dirs:
            Log.debug(
                f"{self.__class__.__name__}: add directory to observer: {directory}")
            watch = self._watchdog_observer.schedule(self, directory)
            self._observed_dirs[directory] = watch
        self._included_files.append(path)
        self._included_dirs.append(directory)

    def _remove_file_from_observe(self, path: str):
        Log.debug(f"{self.__class__.__name__}: stop observe path: {path}")
        try:
            directory = os.path.dirname(path)
            self._included_files.remove(path)
            self._included_dirs.remove(directory)
            if directory not in self._included_dirs:
                if directory in self._observed_dirs:
                    Log.debug(
                        f"{self.__class__.__name__}: remove directory from observer: {directory}")
                    self._watchdog_observer.unschedule(
                        self._observed_dirs[directory])
                    del self._observed_dirs[directory]
        except ValueError:
            pass

    def _add_launch_to_observer(self, launch_config: LaunchConfig):
        added = []
        try:
            self._add_file_to_observe(launch_config.filename)
            added.append(launch_config.filename)
            for inc_discription in launch_config._included_files:
                self._add_file_to_observe(inc_discription._get_launch_file())
                added.append(inc_discription._get_launch_file())
        except Exception as e:
            Log.error(
                f"{self.__class__.__name__}: _add_launch_to_observer {launch_config.filename}: \n {e}")
        self._observed_launch_files[launch_config.filename] = added

    def _remove_launch_from_observer(self, launch_config: LaunchConfig):
        try:
            for path in self._observed_launch_files[launch_config.filename]:
                self._remove_file_from_observe(path)
            del self._observed_launch_files[launch_config.filename]
        except Exception as e:
            Log.error(
                f"{self.__class__.__name__}: _add_launch_to_observer {launch_config.filename}:\n{e}")

    # def start_node(self, node_name):
    #     global IS_RUNNING
    #     if not IS_RUNNING:
    #         return
    #     for _cfgid, launchcfg in self._loaded_files.items():
    #         n = launchcfg.get_node(node_name)
    #         if n is not None:
    #             startcfg = launcher.create_start_config(
    #                 node_name, launchcfg, '', daemonuri='', loglevel='')
    #             launcher.run_node(startcfg)
    #             return
    #     raise Exception("Node '%s' not found!" % node_name)

    # def _autostart_nodes_threaded(self, cfg):
    #     global IS_RUNNING
    #     for item in cfg.roscfg.nodes:
    #         if not IS_RUNNING:
    #             return
    #         node_fullname = ns_join(item.namespace, item.name)
    #         try:
    #             if self._get_start_exclude(cfg, node_fullname):
    #                 # skip autostart
    #                 nmd.ros_node.get_logger().debug(
    #                     "%s is in exclude list, skip autostart" % node_fullname)
    #                 continue
    #             self._autostart_node(node_fullname, cfg)
    #         except Exception as err:
    #             nmd.ros_node.get_logger().warn("Error while start %s: %s" % (node_fullname, err))

    # def _autostart_node(self, node_name, cfg):
    #     global IS_RUNNING
    #     if not IS_RUNNING:
    #         return
    #     start_required = self._get_start_required(cfg, node_name)
    #     start_now = False
    #     if start_required:
    #         import rosgraph
    #         # get published topics from ROS master
    #         master = rosgraph.masterapi.Master(cfg.masteruri)
    #         for topic, _datatype in master.getPublishedTopics(''):
    #             if start_required == topic:
    #                 start_now = True
    #                 break
    #         if not start_now:
    #             # Start the timer for waiting for the topic
    #             start_timer = threading.Timer(
    #                 3.0, self._autostart_node, args=(node_name, cfg))
    #             start_timer.start()
    #     else:
    #         start_now = True
    #     if start_now:
    #         startcfg = launcher.create_start_config(
    #             node_name, cfg, '', daemonuri='', loglevel='')
    #         start_delay = self._get_start_delay(cfg, node_name)
    #         if start_delay > 0:
    #             # start timer for delayed start
    #             start_timer = threading.Timer(
    #                 start_delay, launcher.run_node, args=(startcfg,))
    #             start_timer.setDaemon(True)
    #             start_timer.start()
    #         else:
    #             launcher.run_node(startcfg)

    @wamp.register('ros.launch.load')
    def load_launch(self, request_json: LaunchLoadRequest, return_as_json: bool = True) -> LaunchLoadReply:
        '''
        Loads launch file by crossbar request
        '''
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.load]")
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        launchfile = request.path
        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug(f"{self.__class__.__name__}: Loading launch file: {launchfile} (package: {request.ros_package}, launch: {request.launch}), daemonuri: {daemonuri}, host: {request.host}, args: {request.args}")

        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = ros_pkg.get_share_files_path_from_package(
                    request.ros_package, request.launch)
                if not paths:
                    result.status.code = 'FILE_NOT_FOUND'
                    result.status.error_msg = "Launch files %s in package %s not found!" % (
                        request.launch, request.ros_package)
                    return json.dumps(result, cls=SelfEncoder) if return_as_json else result
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = 'MULTIPLE_LAUNCHES'
                        result.status.msg = "Multiple launch files with name %s in package %s found!" % (
                            request.launch, request.ros_package)
                        for mp in paths:
                            result.paths.append(mp)
                        Log.debug(
                            f"{self.__class__.__name__}: ..load aborted, MULTIPLE_LAUNCHES")
                        return json.dumps(result, cls=SelfEncoder) if return_as_json else result
                else:
                    launchfile = paths[0]
            except LookupError as rnf:
                result.status.code = 'FILE_NOT_FOUND'
                result.status.msg = "Package %s not found: %s" % (
                    request.ros_package, rnf)
                Log.debug(
                    f"{self.__class__.__name__}: ..load aborted, FILE_NOT_FOUND")
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        result.paths.append(launchfile)
        # it is already loaded?
        if (launchfile, daemonuri) in list(self._loaded_files.keys()):
            result.status.code = 'ALREADY_OPEN'
            result.status.msg = "Launch file %s already loaded!" % (
                launchfile)
            Log.debug(
                f"{self.__class__.__name__}: ..load aborted, ALREADY_OPEN")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

        # load launch configuration
        try:
            # validate xml
            self.xml_validator.validate(launchfile)
            # test for required args
            provided_args = [arg.name for arg in request.args]
            # get the list with needed launch args
            req_args = LaunchConfig.get_launch_arguments(
                launchfile, request.args)
            # req_args_dict = launch_config.argv2dict(req_args)
            if request.request_args and req_args:
                for arg in req_args:
                    if arg.name not in provided_args:
                        result.args.extend(req_args)
                        result.status.code = 'PARAMS_REQUIRED'
                        Log.debug(
                            f"{self.__class__.__name__}: ..load aborted, PARAMS_REQUIRED {[arg.name for arg in result.args]}; privided args {provided_args}")
                        return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            # argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args]  # if arg.name in req_args_dict]
            launch_arguments = [(arg.name, arg.value) for arg in request.args]
            # context=self.__launch_context
            launch_config = LaunchConfig(
                launchfile, daemonuri=daemonuri, launch_arguments=launch_arguments)
            Log.debug(f"{self.__class__.__name__}: daemonuri: {daemonuri}")
            # _loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            # result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.resolve_dict.items()])
            self._loaded_files[CfgId(launchfile, daemonuri)] = launch_config
            # notify changes to crossbar GUI
            self.publish_to('ros.launch.changed', {})
            self._add_launch_to_observer(launch_config)
            Log.debug(f"{self.__class__.__name__}: ..load complete!")
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, e)
            Log.warn(
                f"{self.__class__.__name__}: Loading launch file: {err_details}")
            result.status.code = 'ERROR'
            result.status.msg = err_details
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        result.status.code = 'OK'
        return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    @wamp.register('ros.launch.reload')
    def reload_launch(self, request_json: LaunchLoadRequest) -> LaunchLoadReply:
        '''
        Reloads launch file by crossbar request
        '''
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.reload]")
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug(f"{self.__class__.__name__}: Loading launch file: {request.path} (package: {request.ros_package}, launch: {request.launch}), daemonuri: {daemonuri}, host: {request.host}, args: {request.args}")

        result.paths.append(request.path)
        cfgid = CfgId(request.path, daemonuri)
        Log.debug(
            f"{self.__class__.__name__}: reload launch file: {request.path}, daemonuri: {daemonuri}")
        if cfgid in self._loaded_files:
            old_launch = self._loaded_files[cfgid]
            try:
                self._remove_launch_from_observer(old_launch)
                old_launch._unload()
                # use argv from already open file
                launch_config = LaunchConfig(
                    old_launch.filename, daemonuri=daemonuri, launch_arguments=old_launch.provided_launch_arguments)
                self._loaded_files[cfgid] = launch_config
                # stored_roscfg = cfg.roscfg
                # argv = cfg.argv
                # cfg.load(argv)
                result.status.code = 'OK'
                # TODO: added change detection for nodes parameters
                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
                self._add_launch_to_observer(launch_config)
            except Exception as e:
                old_launch._load()
                self._add_launch_to_observer(old_launch)
                print(traceback.format_exc())
                err_text = f"{request.path} loading failed!"
                err_details = f"{err_text}: {e}"
                Log.warn(
                    f"{self.__class__.__name__}: Loading launch file: {err_details}")
                result.status.code = 'ERROR'
                result.status.msg = err_details
                return json.dumps(result, cls=SelfEncoder)
        else:
            result.status.code = 'FILE_NOT_FOUND'
            return json.dumps(result, cls=SelfEncoder)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.unload')
    def unload_launch(self, request_json: LaunchFile) -> LaunchLoadReply:
        Log.debug(f"{self.__class__.__name__}: Request to [ros.launch.unload]")

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        Log.debug(
            f"{self.__class__.__name__}: UnloadLaunch request:\n {request}")
        result = LaunchLoadReply(paths=[], changed_nodes=[], args=[])

        result.paths.append(request.path)
        # cfgid = CfgId(request.path, request.masteruri)
        # TODO: check if we need daemonuri as identification
        cfgid = CfgId(request.path, '')
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(self._loaded_files[cfgid])
                del self._loaded_files[cfgid]
                result.status.code = 'OK'

                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, e)
                Log.warn("Unloading launch file: %s", err_details)
                result.status.code = 'ERROR'
                result.status.msg = err_details
        else:
            result.status.code = 'FILE_NOT_FOUND'
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.get_list')
    def get_list(self) -> List[LaunchContent]:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_list]")
        requested_files = list(self._loaded_files.keys())
        reply = []
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply_lc = LaunchContent(path=cfgid.path, args=[], masteruri=lc.daemonuri, host='',  # lc.host,
                                     nodes=[], parameters=[], associations=[])

            # Add launch arguments
            for name, p in lc.provided_launch_arguments:
                reply_lc.args.append(LaunchArgument(
                    name, p.value if hasattr(p, 'value') else p))

            nodes = lc.nodes()
            for item in nodes:
                node_fullname = item.node_name
                # is_executable = type(
                #     item.node) == launch.actions.execute_process.ExecuteProcess
                # composable_container = ''
                # if item.composable_container is not None:
                #     # get composable container name
                #     composable_container = LaunchConfig.get_name_from_node(
                #         item.composable_container)
                # ni = lmsg.NodeInfo(name=node_fullname, is_executable=is_executable,
                #                   composable_container=composable_container)
                # add composable nodes to container
                # for cn in item.composable_nodes:
                #    ni.composable_nodes.extend(
                #        [LaunchConfig.get_name_from_node(cn)])
                reply_lc.nodes.append(item)
                # reply_lc.nodes.append(LaunchNodeInfo(node_fullname,
                #                                      node_namespace='',
                #                                      package=item.node_package,
                #                                      node_type=item.node_executable,
                #                                      args=shlex.join(item.arguments),
                #                                      respawn=item.respawn,
                #                                      respawn_delay=item.respawn_delay,
                #                                      launch_prefix=item.prefix,
                #                                      file_name=cfgid.path,
                #                                      launch_name=cfgid.path))
                # TODO: add composable nodes to container
                # for cn in item.composable_nodes:
                #    reply_lc.nodes.append(LaunchConfig.get_name_from_node(cn))

            # Add parameter values
            # for name, p in lc.xxx:
            #    reply_lc.parameters.append(RosParameter(name, p.value))
            print('nodes', reply_lc.nodes)
            # TODO: add assosiations
            reply.append(reply_lc)
        return json.dumps(reply, cls=SelfEncoder)

    @wamp.register('ros.launch.start_node')
    def start_node(self, request_json: LaunchNode, return_as_json: bool = True) -> LaunchNodeReply:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.start_node]")

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        result = LaunchNodeReply(name=request.name, paths=[], launch_files=[])

        try:
            launch_configs = []
            if request.opt_launch:
                cfgid = CfgId(request.opt_launch, request.masteruri)
                if cfgid in self._loaded_files:
                    launch_configs.append(self._loaded_files[cfgid])
            if not launch_configs:
                # get launch configurations with given node
                launch_configs = []
                for cfgid, launchcfg in self._loaded_files.items():
                    n = launchcfg.get_node(request.name)
                    if n is not None:
                        Log.debug(f"Found launch file={launchcfg.filename};")
                        launch_configs.append(launchcfg)
            if not launch_configs:
                result.status.code = 'NODE_NOT_FOUND'
                result.status.msg = f"Node '{request.name}' not found"
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            if len(launch_configs) > 1:
                result.status.code = 'MULTIPLE_LAUNCHES'
                result.status.msg = f"Node '{request.name}' found in multiple launch files"
                result.launch_files.extend(
                    [lcfg.filename for lcfg in launch_configs])
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
            try:
                result.launch_files.append(launch_configs[0].filename)
                launch_configs[0].run_node(request.name)
                Log.debug(f'Node={request.name}; start finished')
                result.status.code = 'OK'
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = 'MULTIPLE_BINARIES'
                result.status.msg = f"multiple binaries found for node '{request.name}': {bsr.choices}"
                result.paths.extend(bsr.choices)
                return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        except exceptions.ResourceNotFound as err_nf:
            result.status.code = 'ERROR'
            result.status.msg = f"Error while start node '{request.name}': {err_nf}"
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        except Exception as _errr:
            result.status.code = 'ERROR'
            result.status.msg = f"Error while start node '{request.name}': {traceback.format_exc()}"
            Log.warn(f"{self.__class__.__name__}: {result.status.msg}")
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result
        finally:
            nmd.launcher.server.screen_servicer.system_change()
            return json.dumps(result, cls=SelfEncoder) if return_as_json else result

    @wamp.register('ros.launch.start_nodes')
    def start_nodes(self, request_json: List[LaunchNode], continue_on_error: bool = True) -> List[LaunchNodeReply]:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.start_nodes]")

        result = []
        for request in request_json:
            node_result = self.start_nodes(request, return_as_json=False)
            result.append(node_result)
            if not continue_on_error:
                if result.status.code != 'OK':
                    break

        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.get_included_files')
    def get_included_files(self, request_json: LaunchIncludedFilesRequest) -> List[LaunchIncludedFile]:
        # Convert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        path = request.path
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_included_files]: Path [{path}]")
        result = []
        try:
            search_in_ext = SEARCH_IN_EXT
            if request.search_in_ext:
                search_in_ext = request.search_in_ext
            # search for loaded file and get the arguments
            resolve_args = {arg.name: arg.value for arg in request.args}
            if not resolve_args:
                for cfgid, lcfg in self._loaded_files.items():
                    if cfgid.path == request.path:
                        resolve_args.update(lcfg.resolve_dict)
                        break
            # replay each file
            for inc_file in xml.find_included_files(request.path, recursive=request.recursive, unique=request.unique, search_in_ext=search_in_ext, resolve_args=resolve_args):
                file_size = 0
                if inc_file.exists:
                    file_size = os.path.getsize(inc_file.inc_path)
                lincf = LaunchIncludedFile(path=inc_file.path_or_str,
                                           line_number=inc_file.line_number,
                                           inc_path=inc_file.inc_path,
                                           exists=inc_file.exists,
                                           raw_inc_path=inc_file.raw_inc_path,
                                           rec_depth=inc_file.rec_depth,
                                           args=[LaunchArgument(
                                               name=name, value=value) for name, value in inc_file.args.items()],
                                           default_inc_args=[LaunchArgument(
                                               name=name, value=value) for name, value in inc_file.args.items()],
                                           size=file_size
                                           )
                result.append(lincf)
        except Exception:
            Log.warn(
                f"{self.__class__.__name__}: Can't get include files for {request.path}: {traceback.format_exc()}")
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.interpret_path')
    def interpret_path(self, request_json: LaunchInterpretPathRequest) -> List[LaunchInterpretPathReply]:
        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        text = request.text
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.interpret_path]: {text}")
        args = {arg.name: arg.value for arg in request.args}
        result = []
        if text:
            try:
                for inc_file in xml.find_included_files(text, False, False, search_in_ext=[]):
                    aval = inc_file.raw_inc_path
                    aitems = aval.split("'")
                    for search_for in aitems:
                        if not search_for:
                            continue
                        Log.debug(
                            f"{self.__class__.__name__}: try to interpret: {search_for}")
                        args_in_name = xml.get_arg_names(search_for)
                        request_args = False
                        for arg_name in args_in_name:
                            if not arg_name in args:
                                request_args = True
                                break
                        if request_args:
                            req_args = []
                            for arg_name in args_in_name:
                                if arg_name in args:
                                    req_args.append(LaunchArgument(
                                        arg_name, args[arg_name]))
                                else:
                                    req_args.append(LaunchArgument(arg_name))
                            reply = LaunchInterpretPathReply(
                                text=search_for, status='PARAMS_REQUIRED', args=req_args)
                            reply.status.code = 'PARAMS_REQUIRED'
                            result.append(reply)
                        else:
                            search_for_rpl = xml.replace_arg(search_for, args)
                            reply = LaunchInterpretPathReply(
                                text=search_for, status='OK', path=search_for_rpl, exists=os.path.exists(search_for), args=request.args)
                            result.append(reply)
            except Exception as err:
                reply = LaunchInterpretPathReply(
                    text=text, status='ERROR', args=request.args)
                reply.status.msg = err
                result.append(reply)
        else:
            reply = LaunchInterpretPathReply(
                text=text, status='ERROR', args=request.args)
            reply.status.msg = 'empty request'
            result.append(reply)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.get_msg_struct')
    def get_msg_struct(self, msg_type: str) -> LaunchMessageStruct:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_msg_struct]: msg [{msg_type}]")
        result = LaunchMessageStruct(msg_type)
        try:
            splitted_type = msg_type.replace('/', '.').split('.')
            splitted_type.reverse()
            module = __import__(splitted_type.pop())
            sub_class = getattr(module, splitted_type.pop())
            while splitted_type:
                sub_class = getattr(sub_class, splitted_type.pop())
            if sub_class is None:
                result.error_msg = f"invalid message type: '{msg_type}'. If this is a valid message type, perhaps you need to run 'colcon build'"
                return json.dumps(result, cls=SelfEncoder)
            if not hasattr(sub_class, 'get_fields_and_field_types'):
                result.error_msg = f"unexpected message class: '{sub_class}', no 'get_fields_and_field_types' attribute found!"
                return json.dumps(result, cls=SelfEncoder)
            field_and_types = sub_class.get_fields_and_field_types()
            msg_dict = {'type': msg_type,
                        'name': '',
                        'def': self._expand_fields(field_and_types)}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    # create recursive dictionary for 'ros.launch.get_msg_struct'
    def _expand_fields(self, field_and_types):
        defs = []
        for field_name, field_type in field_and_types.items():
            field_struct = {'name': field_name, 'def': []}
            is_array = False
            base_type = field_type
            seq_length = None
            if field_type.startswith('sequence'):
                # handle sequences defined with sequence<>
                is_array = True
                type_re = re.search('<(.*)>', field_type)
                if type_re is not None:
                    base_type = type_re.group(1)
            elif '[' in field_type:
                # handle arrays defined with []
                is_array = True
                type_re = re.search('(.*)\[(\d*)\]', field_type)
                if type_re is not None:
                    base_type = type_re.group(1)
                    seq_length = type_re.group(2)
            if base_type not in [*rosidl_parser.definition.BASIC_TYPES, 'string', 'str']:
                # type is not a simple type
                # try to import message definition
                splitted_type = base_type.split('/')
                module = __import__(splitted_type[0])
                msg_class = getattr(module, 'msg')
                msg_class = getattr(msg_class, splitted_type[1])
                if msg_class is not None:
                    sub_def = self._expand_fields(
                        msg_class.get_fields_and_field_types())
                field_struct['def'] = sub_def
            reported_type = base_type
            if (is_array):
                reported_type += f'[{seq_length}]' if seq_length else '[]'
            field_struct['type'] = reported_type
            field_struct['is_array'] = is_array
            if seq_length:
                field_struct['length'] = seq_length
            defs.append(field_struct)
        return defs

    def str2typedValue(self, value, value_type):
        result = value
        if 'int' in value_type:
            result = int(value)
        elif 'float' in value_type or 'double' in value_type:
            result = float(value)
        elif value_type.startswith('bool'):
            result = value.lower() in ('yes', 'true', 't', 'y', '1')
        return result

    def _str_from_dict(self, param_dict):
        result = dict()
        fields = param_dict if isinstance(
            param_dict, list) else param_dict['def']
        for field in fields:
            if not field['def']:
                # simple types
                if 'value' in field and field['value']:
                    base_type = field['type'].replace(r'/\[\d*\]/', '')
                    if field['is_array']:
                        # parse to array
                        listvals = field['value'].split(',')
                        result[field['name']] = [self.str2typedValue(
                            n, base_type) for n in listvals]
                    else:
                        result[field['name']] = self.str2typedValue(
                            field['value'], base_type)
            elif field['is_array']:
                # TODO: create array for base types
                result_array = []
                # it is a complex field type
                if 'value' in field:
                    for array_element in field['value']:
                        result_array.append(
                            self._str_from_dict(array_element))
                # append created array
                if result_array:
                    result[field['name']] = result_array
            else:
                sub_result = self._str_from_dict(field['def'])
                if sub_result:
                    result[field['name']] = sub_result
        return result

    @wamp.register('ros.launch.publish_message')
    def publish_message(self, request_json: LaunchPublishMessage) -> None:
        try:
            # Convert input dictionary into a proper python object
            request = json.loads(json.dumps(request_json),
                                 object_hook=lambda d: SimpleNamespace(**d))
            Log.debug(
                f"{self.__class__.__name__}: Request to [ros.launch.publish_message]: msg [{request.msg_type}]")
            opt_str = ''
            opt_name_suf = '__latch_'
            if request.once:
                opt_str = '-1'
            elif request.latched:
                # quality of service for latched topics
                opt_str = '--qos-durability transient_local --qos-reliability reliable'
            elif request.rate != 0.0:
                opt_str = f"-r {request.rate}"
            if request.verbose:
                opt_str += ' -p 1'
            else:
                opt_str += ' -p 10'
            if request.use_rostime:
                opt_str += ' --use-rostime'
            fullname = f"/rostopic_pub/{request.topic_name.strip('/')}".replace(
                '/', '_')
            opt_str += f' -n {fullname}'
            data = json.loads(request.data)
            topic_params = self._str_from_dict(data)
            pub_cmd = f"pub {opt_str} {request.topic_name} {request.msg_type} \"{topic_params}\""
            screen_prefix = ' '.join([screen.get_cmd(fullname)])
            cmd = ' '.join([screen_prefix, 'ros2', 'topic', pub_cmd])
            Log.debug(
                f"{self.__class__.__name__}: run ros2 publisher with: {cmd}")
            SupervisedPopen(shlex.split(cmd),
                            object_id=f"ros_topic_pub_{request.topic_name}", description=f"publish to topic {request.topic_name}")
        except Exception:
            import traceback
            print(traceback.format_exc())

    @wamp.register('ros.launch.get_srv_struct')
    def get_srv_struct(self, srv_type: str) -> LaunchMessageStruct:
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.get_srv_struct]: srv [{srv_type}]")
        result = LaunchMessageStruct(srv_type)
        try:
            splitted_type = srv_type.replace('/', '.').split('.')
            splitted_type.reverse()
            module = __import__(splitted_type.pop())
            sub_class = getattr(module, splitted_type.pop())
            while splitted_type:
                sub_class = getattr(sub_class, splitted_type.pop())
            if sub_class is None:
                result.error_msg = f"invalid service type: '{srv_type}'. If this is a valid service type, perhaps you need to run 'colcon build'"
                return json.dumps(result, cls=SelfEncoder)
            if not hasattr(sub_class.Request, 'get_fields_and_field_types'):
                result.error_msg = f"unexpected service class: '{sub_class}', no 'get_fields_and_field_types' attribute found!"
                return json.dumps(result, cls=SelfEncoder)
            field_and_types = sub_class.Request.get_fields_and_field_types()
            msg_dict = {'type': srv_type,
                        'name': '',
                        'def': self._expand_fields(field_and_types)}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
            result.valid = False
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.call_service')
    def call_service(self, request_json: LaunchCallService) -> None:
        # Convert input dictionary into a proper python object
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.launch.call_service]: {request_json}")
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        result = LaunchMessageStruct(request.srv_type)
        try:
            splitted_type = request.srv_type.replace('/', '.').split('.')
            splitted_type.reverse()
            module = __import__(splitted_type.pop())
            sub_class = getattr(module, splitted_type.pop())
            while splitted_type:
                sub_class = getattr(sub_class, splitted_type.pop())
            if sub_class is None:
                result.error_msg = f"invalid service type: '{request.srv_type}'. If this is a valid service type, perhaps you need to run 'colcon build'"
                return json.dumps(result, cls=SelfEncoder)
            if not hasattr(sub_class.Request, 'get_fields_and_field_types'):
                result.error_msg = f"unexpected service class: '{sub_class}', no 'get_fields_and_field_types' attribute found!"
                return json.dumps(result, cls=SelfEncoder)

            cli = nmd.ros_node.create_client(sub_class, request.service_name)

            # create request message
            service_request = sub_class.Request()
            try:
                data = json.loads(request.data)
                set_message_fields(service_request, self._str_from_dict(data))
            except Exception as e:
                result.error_msg = 'Failed to populate field: {0}'.format(e)

            # call service
            if not cli.service_is_ready():
                Log.debug(
                    f"{self.__class__.__name__}: waiting for service '{request.service_name}' to become available...")
                cli.wait_for_service()
            Log.debug(
                f"{self.__class__.__name__}: requester: making request: {service_request}")
            future = cli.call_async(service_request)
            rclpy.spin_until_future_complete(
                nmd.ros_node, future, nmd.launcher.executor, 30.0)
            if future.result() is not None:
                result.data = message_to_ordereddict(future.result())
                result.valid = True
            else:
                result.error_msg = 'Exception while calling service: %r' % future.exception()
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.subscriber.start')
    def start_subscriber(self, request_json: SubscriberNode) -> bool:
        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        topic = request.topic
        Log.debug(
            f"{self.__class__.__name__}: Request to [ros.subscriber.start]: {topic}")
        package_name = 'fkie_node_manager_daemon'
        executable = 'node_manager_subscriber'
        cmd = f"ros2 run {package_name} {executable}"
        # fullname = f"/_node_manager_subscriber/{topic.replace('.', '/').strip('/')}"
        self.namespace, self.name = ros2_subscriber_nodename_tuple(topic)
        fullname = os.path.join(self.namespace, self.name)
        # args = [f"__name:={fullname}"]
        # args.append(f'--name={fullname}')
        args = []
        args.append(f'--crossbar_port={self.port}')
        args.append(f'--crossbar_realm={self.realm}')
        args.append(f'--topic={topic}')
        args.append(f'--message_type={request.message_type}')
        if request.filter.no_data:
            args.append('--no_data')
        if request.filter.no_arr:
            args.append('--no_arr')
        if request.filter.no_str:
            args.append('--no_str')
        args.append(f'--hz={request.filter.hz}')
        args.append(f'--window={request.filter.window}')
        if request.tcp_no_delay:
            args.append('--tcp_no_delay')

        # run on local host
        # set environment
        new_env = dict(os.environ)

        # set display variable to local display
        if 'DISPLAY' in new_env:
            if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
                del new_env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0'

        # start
        screen_prefix = ' '.join(
            [screen.get_cmd(fullname, new_env, new_env.keys())])
        Log.info(
            f"{self.__class__.__name__}: {screen_prefix} {cmd} {' '.join(args)}")
        # Log.debug(
        #     f"environment while run node '{fullname}': '{new_env}'")
        # Log.debug(
        #     f"args while run node '{fullname}': '{args}', JOINED: {' '.join([screen_prefix, cmd] + args)}")
        SupervisedPopen(shlex.split(' '.join([screen_prefix, cmd] + args)), env=new_env,
                        object_id=f"run_node_{fullname}", description=f"Run [{package_name}]{executable}")
        return True

    def list_nodes(self):
        result = []
        for cfgid in list(self._loaded_files.keys()):
            lc = self._loaded_files[cfgid]
            for item in lc.roscfg.nodes:
                node_fullname = ns_join(item.namespace, item.name)
                result.append(node_fullname)
        return result

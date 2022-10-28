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
import sys
import traceback

import launch
from launch.launch_context import LaunchContext
import asyncio

import json
from types import SimpleNamespace
import asyncio
from autobahn import wamp

from typing import List
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from watchdog.events import FileSystemEvent

from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchArgument
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchFile
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchLoadRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchLoadReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchContent
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNodelets
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchAssociations
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNode
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNodeReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchInterpretPathRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchInterpretPathReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchIncludedFilesRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchIncludedFile
from fkie_multimaster_msgs.defines import SEARCH_IN_EXT
from fkie_multimaster_msgs.launch import ros_pkg
from fkie_multimaster_msgs.launch import xml
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system import exceptions
from fkie_multimaster_msgs.system.host import is_local
from fkie_multimaster_msgs.system.url import equal_uri

from . import launcher
from .launch_config import LaunchConfig
from .launch_validator import LaunchValidator


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
        return not(self == other)

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
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)
        self._watchdog_observer.start()

    def _terminated(self):
        Log.info("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            Log.info("Add callback to peer context @%s" % context.peer())
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        '''
        Stop watchdog and cancel the autostart of the nodes.
        '''
        self._is_running = False
        self._watchdog_observer.stop()

    def on_any_event(self, event: FileSystemEvent):
        if event.src_path in self._included_files:
            change_event = {event.event_type: event.src_path}
            Log.debug("observed change %s on %s" %
                      (event.event_type, event.src_path))
            self.publish('ros.path.changed', json.dumps(
                change_event, cls=SelfEncoder))

    def _add_file_to_observe(self, path):
        directory = os.path.dirname(path)
        Log.debug('observe path: %s' % path)
        if directory not in self._observed_dirs:
            Log.debug('add directory to observer: %s' % directory)
            watch = self._watchdog_observer.schedule(self, directory)
            self._observed_dirs[directory] = watch
        self._included_files.append(path)
        self._included_dirs.append(directory)

    def _remove_file_from_observe(self, path):
        Log.debug('stop observe path: %s' % str(path))
        try:
            directory = os.path.dirname(path)
            self._included_files.remove(path)
            self._included_dirs.remove(directory)
            if directory not in self._included_dirs:
                if directory in self._observed_dirs:
                    Log.debug('remove directory from observer: %s' % directory)
                    self._watchdog_observer.unschedule(
                        self._observed_dirs[directory])
                    del self._observed_dirs[directory]
        except ValueError:
            pass

    def _add_launch_to_observer(self, path):
        try:
            self._add_file_to_observe(path)
            search_in_ext = SEARCH_IN_EXT
            # search for loaded file and get the arguments
            resolve_args = {}
            for cfg_id, cfg in self._loaded_files.items():
                if cfg_id.path == path:
                    resolve_args.update(cfg.resolve_dict)
                    break
            # replay each file
            for inc_file in xml.find_included_files(path, True, True, search_in_ext, resolve_args):
                if inc_file.exists:
                    self._add_file_to_observe(inc_file.inc_path)
        except Exception as e:
            Log.error('_add_launch_to_observer %s:\n%s' % (str(path), e))

    def _remove_launch_from_observer(self, path):
        try:
            self._remove_file_from_observe(path)
            search_in_ext = SEARCH_IN_EXT
            # search for loaded file and get the arguments
            resolve_args = {}
            for cfg_id, cfg in self._loaded_files.items():
                if cfg_id.path == path:
                    resolve_args.update(cfg.resolve_dict)
                    break
            # replay each file
            for inc_file in xml.find_included_files(path, True, True, search_in_ext, resolve_args):
                self._remove_file_from_observe(inc_file.inc_path)
        except Exception as e:
            Log.error('_add_launch_to_observer %s:\n%s' % (str(path), e))

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
    def load_launch(self, request_json: LaunchLoadRequest) -> LaunchLoadReply:
        '''
        Loads launch file by crossbar request
        '''
        Log.debug('Request to [ros.launch.load]')
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        launchfile = request.path
        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug('Loading launch file: %s (package: %s, launch: %s), daemonuri: %s, host: %s, args: %s' % (
            launchfile, request.ros_package, request.launch, daemonuri, request.host, request.args))

        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = ros_pkg.get_share_files_path_from_package(
                    request.ros_package, request.launch)
                if not paths:
                    result.status.code = 'FILE_NOT_FOUND'
                    result.status.error_msg = "Launch files %s in package %s not found!" % (
                        request.launch, request.ros_package)
                    return json.dumps(result, cls=SelfEncoder)
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = 'MULTIPLE_LAUNCHES'
                        result.status.msg = "Multiple launch files with name %s in package %s found!" % (
                            request.launch, request.ros_package)
                        for mp in paths:
                            result.paths.append(mp)
                        Log.debug('..load aborted, MULTIPLE_LAUNCHES')
                        return json.dumps(result, cls=SelfEncoder)
                else:
                    launchfile = paths[0]
            except LookupError as rnf:
                result.status.code = 'FILE_NOT_FOUND'
                result.status.msg = "Package %s not found: %s" % (
                    request.ros_package, rnf)
                Log.debug('..load aborted, FILE_NOT_FOUND')
                return json.dumps(result, cls=SelfEncoder)
        result.paths.append(launchfile)
        # it is already loaded?
        if (launchfile, daemonuri) in self._loaded_files.keys():
            result.status.code = 'ALREADY_OPEN'
            result.status.msg = "Launch file %s already loaded!" % (
                launchfile)
            Log.debug('..load aborted, ALREADY_OPEN')
            return json.dumps(result, cls=SelfEncoder)

        # load launch configuration
        try:
            # validate xml
            self.xml_validator.validate(launchfile)
            # test for required args
            provided_args = [arg.name for arg in request.args]
            print('provided_args', provided_args)
            # get the list with needed launch args
            req_args = LaunchConfig.get_args(launchfile, request.args)
            #req_args_dict = launch_config.argv2dict(req_args)
            if request.request_args and req_args:
                for arg in req_args:
                    if arg.name not in provided_args:
                        result.args.extend(req_args)
                        result.status.code = 'PARAMS_REQUIRED'
                        Log.debug('..load aborted, PARAMS_REQUIRED')
                        return json.dumps(result, cls=SelfEncoder)
            # argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args]  # if arg.name in req_args_dict]
            launch_arguments = [(arg.name, arg.value) for arg in request.args]
            # context=self.__launch_context
            launch_config = LaunchConfig(
                launchfile, daemonuri=request.masteruri, launch_arguments=launch_arguments)
            #_loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            #result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.resolve_dict.items()])
            self._loaded_files[CfgId(
                launchfile, request.masteruri)] = launch_config
            # notify changes to crossbar GUI
            try:
                self.publish('ros.launch.changed',
                             json.dumps({}, cls=SelfEncoder))
                self._add_launch_to_observer(launchfile)
            except Exception as cpe:
                pass
            Log.debug('..load complete!')
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, e)
            Log.warn('Loading launch file: %s', err_details)
            result.status.code = 'ERROR'
            result.status.msg = err_details
            return json.dumps(result, cls=SelfEncoder)
        result.status.code = 'OK'
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.reload')
    def reload_launch(self, request_json: LaunchLoadRequest) -> LaunchLoadReply:
        '''
        Reloads launch file by crossbar request
        '''
        Log.debug('Request to [ros.launch.reload]')
        result = LaunchLoadReply(paths=[], args=[], changed_nodes=[])

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        daemonuri = ''
        if hasattr(request, 'masteruri'):
            daemonuri = request.masteruri
        Log.debug('Loading launch file: %s (package: %s, launch: %s), daemonuri: %s, host: %s, args: %s' % (
            request.path, request.ros_package, request.launch, daemonuri, request.host, request.args))

        result.path.append(request.path)
        cfgid = CfgId(request.path, daemonuri)
        Log.debug("reload launch file: %s, daemonuri: %s",
                  request.path, daemonuri)
        if cfgid in self._loaded_files:
            try:
                # use argv from already open file
                cfg = self._loaded_files[cfgid]
                launch_config = LaunchConfig(cfg.filename, daemonuri=request.daemonuri, launch_arguments=cfg.launch_arguments)
                self._loaded_files[cfgid] = launch_config
                # stored_roscfg = cfg.roscfg
                # argv = cfg.argv
                # cfg.load(argv)
                result.status.code = 'OK'
                #TODO: added change detection for nodes parameters
                # notify changes to crossbar GUI
                try:
                    self.publish('ros.launch.changed',
                                 json.dumps({}, cls=SelfEncoder))
                    self._add_launch_to_observer(request.path)
                except Exception as cpe:
                    pass
            except Exception as e:
                print(traceback.format_exc())
                self._add_launch_to_observer(request.path)
                err_text = f"{request.path} loading failed!"
                err_details = f"{err_text}: {e}"
                Log.warn("Loading launch file: %s", err_details)
                result.status.code = 'ERROR'
                result.status.msg = err_details
                return result
        else:
            result.status.code = 'FILE_NOT_FOUND'
            return result
        return result



    @wamp.register('ros.launch.unload')
    def unload_launch(self, request_json: LaunchFile) -> LaunchLoadReply:
        Log.debug('Request to [ros.launch.unload]')

        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))

        Log.debug('UnloadLaunch request:\n%s' % str(request))
        result = LaunchLoadReply(paths=[], changed_nodes=[], args=[])

        result.paths.append(request.path)
        # cfgid = CfgId(request.path, request.masteruri)
        # TODO: check if we need daemonuri as identification
        cfgid = CfgId(request.path, '')
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(request.path)
                del self._loaded_files[cfgid]
                result.status.code = 'OK'

                # notify changes to crossbar GUI
                try:
                    self.publish('ros.launch.changed',
                                 json.dumps({}, cls=SelfEncoder))
                except Exception as cpe:
                    pass
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
        Log.debug('Request to [ros.launch.get_list]')
        requested_files = list(self._loaded_files.keys())
        reply = []
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply_lc = LaunchContent(path=cfgid.path, args=[], masteruri=lc.daemonuri, host='',  # lc.host,
                                     nodes=[], parameters=[], nodelets=[], associations=[])

            nodes = lc.nodes()
            for item in nodes:
                node_fullname = LaunchConfig.get_name_from_node(item.node)
                is_executable = type(
                    item.node) == launch.actions.execute_process.ExecuteProcess
                composable_container = ''
                if item.composable_container is not None:
                    # get composable container name
                    composable_container = LaunchConfig.get_name_from_node(
                        item.composable_container)
                # ni = lmsg.NodeInfo(name=node_fullname, is_executable=is_executable,
                #                   composable_container=composable_container)
                # add composable nodes to container
                # for cn in item.composable_nodes:
                #    ni.composable_nodes.extend(
                #        [LaunchConfig.get_name_from_node(cn)])
                reply_lc.nodes.append(node_fullname)
                # TODO: add composable nodes to container
                for cn in item.composable_nodes:
                    reply_lc.nodes.append(LaunchConfig.get_name_from_node(cn))

            # Add launch arguments
            for name, p in lc.launch_arguments:
                reply_lc.args.append(LaunchArgument(name, p.value))
            # Add parameter values
            #for name, p in lc.xxx:
            #    reply_lc.parameters.append(RosParameter(name, p.value))
            print('nodes', reply_lc.nodes)
            # TODO: add assosiations
            reply.append(reply_lc)
        return json.dumps(reply, cls=SelfEncoder)

    @wamp.register('ros.launch.start_node')
    def start_node(self, request_json: LaunchNode) -> LaunchNodeReply:
        Log.debug('Request to [ros.launch.start_node]')

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
                        Log.debug('Found launch file=%s;' %
                                  (launchcfg.filename))
                        launch_configs.append(launchcfg)
            if not launch_configs:
                result.status.code = 'NODE_NOT_FOUND'
                result.status.msg = "Node '%s' not found" % request.name
                return json.dumps(result, cls=SelfEncoder)
            if len(launch_configs) > 1:
                result.status.code = 'MULTIPLE_LAUNCHES'
                result.status.msg = "Node '%s' found in multiple launch files" % request.name
                result.launch_files.extend(
                    [lcfg.filename for lcfg in launch_configs])
                return json.dumps(result, cls=SelfEncoder)
            try:
                result.launch_files.append(launch_configs[0].filename)
                n = launch_configs[0].get_node(request.name)
                if n is not None:
                    if n.composable_container:
                        # load plugin in container
                        container_name = launch_configs[0].get_name_from_node(
                            n.composable_container)
                        Log.debug('Load node=%s; as plugin into container=%s;' %
                                  (request.name, container_name))
                        launcher.run_composed_node(
                            n.node, container_name=container_name, context=launch_configs[0].context)
                    else:
                        startcfg = launcher.from_node(
                            n.node, launchcfg=launch_configs[0], executable=request.opt_binary, loglevel=request.loglevel, logformat=request.logformat, cmd_prefix=request.cmd_prefix)
                        launcher.run_node(startcfg)
                        Log.debug('Node=%s; start finished' % (request.name))
                    result.status.code = 'OK'
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = 'MULTIPLE_BINARIES'
                result.status.msg = "multiple binaries found for node '%s': %s" % (
                    request.name, bsr.choices)
                result.paths.extend(bsr.choices)
                return json.dumps(result, cls=SelfEncoder)
        except exceptions.ResourceNotFound as err_nf:
            result.status.code = 'ERROR'
            result.status.msg = "Error while start node '%s': %s" % (
                request.name, err_nf)
            return json.dumps(result, cls=SelfEncoder)
        except Exception as _errr:
            result.status.code = 'ERROR'
            result.status.msg = "Error while start node '%s': %s" % (
                request.name, traceback.format_exc())
            return json.dumps(result, cls=SelfEncoder)
        finally:
            return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.get_included_files')
    def get_included_files(self, request_json: LaunchIncludedFilesRequest) -> List[LaunchIncludedFile]:
        # Convert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        path = request.path
        Log.debug(
            'Request to [ros.launch.get_included_files]: Path [%s]' % str(path))
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
            Log.warn("Can't get include files for %s: %s" %
                     (request.path, traceback.format_exc()))
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.interpret_path')
    def interpret_path(self, request_json: LaunchInterpretPathRequest) -> List[LaunchInterpretPathReply]:
        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        text = request.text
        Log.debug(
            'Request to [ros.launch.interpret_path]: %s' % str(text))
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
                        Log.debug("try to interpret: %s" % search_for)
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

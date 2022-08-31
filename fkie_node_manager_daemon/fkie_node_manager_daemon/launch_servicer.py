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


import grpc
import os
import re
import rclpy
import sys
import threading
import traceback

import launch
from ament_index_python.resources import get_resource
from launch.launch_context import LaunchContext
import asyncio

import fkie_node_manager_daemon.grpc_proto.launch_pb2_grpc as lgrpc
import fkie_node_manager_daemon.grpc_proto.launch_pb2 as lmsg
import fkie_node_manager_daemon as nmd
from . import exceptions
from . import launcher
from . import url
from .common import ns_join
from .common import get_share_files_path_from_package
from .common import INCLUDE_PATTERN, SEARCH_IN_EXT
from .common import find_included_files
from .common import interpret_path
from .common import reset_package_cache
from .host import is_local
from .launch_config import LaunchConfig
from .launch_validator import LaunchValidator
from .startcfg import StartConfig

OK = lmsg.ReturnStatus.StatusType.Value('OK')
ERROR = lmsg.ReturnStatus.StatusType.Value('ERROR')
ALREADY_OPEN = lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN')
MULTIPLE_BINARIES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_BINARIES')
MULTIPLE_LAUNCHES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES')
PARAMS_REQUIRED = lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED')
FILE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND')
NODE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('NODE_NOT_FOUND')
CONNECTION_ERROR = lmsg.ReturnStatus.StatusType.Value('CONNECTION_ERROR')
PACKAGE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('PACKAGE_NOT_FOUND')

IS_RUNNING = True


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
        if url.equal_uri(self.daemonuri, daemonuri):
            return True
        return False


class LaunchServicer(lgrpc.LaunchServiceServicer):
    '''
    Handles GRPC-requests defined in `launch.proto`.
    '''

    def __init__(self, ros_domain_id=-1):
        nmd.ros_node.get_logger().info("Create launch servicer")
        lgrpc.LaunchServiceServicer.__init__(self)
        self.__launch_context = LaunchContext(argv=sys.argv[1:])
        self.__launch_context._set_asyncio_loop(asyncio.get_event_loop())
        self.ros_domain_id = ros_domain_id
        self.xml_validator = LaunchValidator()
        self._is_running = True
        self._peers = {}
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)

    def _terminated(self):
        nmd.ros_node.get_logger().info("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            nmd.ros_node.get_logger().info("Add callback to peer context @%s" % context.peer())
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        '''
        Cancel the autostart of the nodes.
        '''
        global IS_RUNNING
        IS_RUNNING = False

    def load_launch_file(self, path, autostart=False):
        '''
        Load launch file and start all included nodes regarding the autostart parameter:

        - <param name="autostart/exclude" value="false" />
        - <param name="autostart/delay" value="5.0" />
        - <param name="autostart/required/publisher" value="topic_name" />

        :param str path: the absolute path of the launch file
        :param bool autostart: True to start all nodes after the launch file was loaded.
        '''
        launch_config = LaunchConfig(path)
        loaded, res_argv = launch_config.load([])
        if loaded:
            nmd.ros_node.get_logger().debug("loaded %s\n  used args: %s" % (path, res_argv))
            self._loaded_files[CfgId(path, '')] = launch_config
            if autostart:
                start_thread = threading.Thread(
                    target=self._autostart_nodes_threaded, args=(launch_config,))
                start_thread.start()
        else:
            nmd.ros_node.get_logger().warn("load %s failed!" % (path))

    def start_node(self, node_name):
        global IS_RUNNING
        if not IS_RUNNING:
            return
        for _cfgid, launchcfg in self._loaded_files.items():
            n = launchcfg.get_node(node_name)
            if n is not None:
                startcfg = launcher.create_start_config(
                    node_name, launchcfg, '', daemonuri='', loglevel='')
                launcher.run_node(startcfg)
                return
        raise Exception("Node '%s' not found!" % node_name)

    def _autostart_nodes_threaded(self, cfg):
        global IS_RUNNING
        for item in cfg.roscfg.nodes:
            if not IS_RUNNING:
                return
            node_fullname = ns_join(item.namespace, item.name)
            try:
                if self._get_start_exclude(cfg, node_fullname):
                    # skip autostart
                    nmd.ros_node.get_logger().debug(
                        "%s is in exclude list, skip autostart" % node_fullname)
                    continue
                self._autostart_node(node_fullname, cfg)
            except Exception as err:
                nmd.ros_node.get_logger().warn("Error while start %s: %s" % (node_fullname, err))

    def _autostart_node(self, node_name, cfg):
        global IS_RUNNING
        if not IS_RUNNING:
            return
        start_required = self._get_start_required(cfg, node_name)
        start_now = False
        if start_required:
            import rosgraph
            # get published topics from ROS master
            master = rosgraph.masterapi.Master(cfg.masteruri)
            for topic, _datatype in master.getPublishedTopics(''):
                if start_required == topic:
                    start_now = True
                    break
            if not start_now:
                # Start the timer for waiting for the topic
                start_timer = threading.Timer(
                    3.0, self._autostart_node, args=(node_name, cfg))
                start_timer.start()
        else:
            start_now = True
        if start_now:
            startcfg = launcher.create_start_config(
                node_name, cfg, '', daemonuri='', loglevel='')
            start_delay = self._get_start_delay(cfg, node_name)
            if start_delay > 0:
                # start timer for delayed start
                start_timer = threading.Timer(
                    start_delay, launcher.run_node, args=(startcfg,))
                start_timer.setDaemon(True)
                start_timer.start()
            else:
                launcher.run_node(startcfg)

    def _get_start_exclude(self, cfg, node):
        param_name = ns_join(node, 'autostart/exclude')
        try:
            return bool(cfg.roscfg.params[param_name].value)
        except Exception:
            pass
        return False

    def _get_start_delay(self, cfg, node):
        param_name = ns_join(node, 'autostart/delay')
        try:
            return float(cfg.roscfg.params[param_name].value)
        except Exception:
            pass
        return 0.

    def _get_start_required(self, cfg, node):
        param_name = ns_join(node, 'autostart/required/publisher')
        topic = ''
        try:
            topic = cfg.roscfg.params[param_name].value
            if topic:
                import rosgraph
                if rosgraph.names.is_private(topic):
                    nmd.ros_node.get_logger().warn(
                        'Private for autostart required topic `%s` is ignored!' % topic)
                    topic = ''
                elif not rosgraph.names.is_global(topic):
                    topic = ns_join(rosgraph.names.namespace(node), topic)
        except Exception:
            pass
        return topic

    def GetLoadedFiles(self, request, context):
        # self._register_callback(context)
        for _cfgid, lf in self._loaded_files.items():
            reply = lmsg.LoadedFile(
                package=lf.packagename, launch=lf.launchname, path=lf.filename, daemonuri=lf.daemonuri)
            reply.args.extend(lmsg.Argument(name=name, value=value)
                              for name, value in lf.resolve_dict.items())
            yield reply

    def LoadLaunch(self, request, context):
        '''
        Loads launch file
        '''
        result = lmsg.LoadLaunchReply()
        launchfile = request.path
        nmd.ros_node.get_logger().debug("Loading launch file: %s (package: %s, launch: %s), daemonuri: %s, args: %s" %
                                       (launchfile, request.package, request.launch, request.daemonuri, request.args))
        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = get_share_files_path_from_package(
                    request.package, request.launch)
                if not paths:
                    result.status.code = FILE_NOT_FOUND
                    result.status.error_msg = "Launch files %s in package %s not found!" % (
                        request.launch, request.package)
                    return result
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = MULTIPLE_LAUNCHES
                        result.status.error_msg = "Multiple launch files with name %s in package %s found!" % (
                            request.launch, request.package)
                        for mp in paths:
                            result.path.append(mp)
                        nmd.ros_node.get_logger().debug("..load aborted, MULTIPLE_LAUNCHES")
                        return result
                else:
                    launchfile = paths[0]
            except LookupError as rnf:
                result.status.code = FILE_NOT_FOUND
                result.status.error_msg = "Package %s not found: %s" % (
                    request.package, rnf)
                nmd.ros_node.get_logger().debug("..load aborted, FILE_NOT_FOUND")
                return result
        result.path.append(launchfile)
        # it is already loaded?
        if (launchfile, request.daemonuri) in self._loaded_files.keys():
            result.status.code = ALREADY_OPEN
            result.status.error_msg = "Launch file %s already loaded!" % (
                launchfile)
            nmd.ros_node.get_logger().debug("..load aborted, ALREADY_OPEN")
            return result
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
                        result.status.code = PARAMS_REQUIRED
                        nmd.ros_node.get_logger().debug("..load aborted, PARAMS_REQUIRED")
                        return result
            # argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args]  # if arg.name in req_args_dict]
            launch_arguments = [(arg.name, arg.value) for arg in request.args]
            # context=self.__launch_context
            launch_config = LaunchConfig(
                launchfile, daemonuri=request.daemonuri, launch_arguments=launch_arguments)
            #_loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            #result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.resolve_dict.items()])
            self._loaded_files[CfgId(
                launchfile, request.daemonuri)] = launch_config
            nmd.ros_node.get_logger().debug("..load complete!")
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, e)
            nmd.ros_node.get_logger().warn("Loading launch file: %s" % err_details)
            result.status.code = ERROR
            result.status.error_msg = err_details
            return result
        result.status.code = OK
        return result

    def ReloadLaunch(self, request, context):
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.daemonuri)
        nmd.ros_node.get_logger().debug("reload launch file: %s, daemonuri: %s" %
                                       (request.path, request.daemonuri))
        if cfgid in self._loaded_files:
            try:
                cfg = self._loaded_files[cfgid]
                launch_config = LaunchConfig(
                    cfg.filename, daemonuri=request.daemonuri, launch_arguments=cfg.launch_arguments)
                self._loaded_files[cfgid] = launch_config
                # stored_roscfg = cfg.roscfg
                # argv = cfg.argv
                # cfg.load(argv)
                result.status.code = OK
                # TODO: detect changes in ros2 configuration
#                 # detect files changes
#                 if stored_roscfg and cfg.roscfg:
#                     stored_values = [(name, str(p.value)) for name, p in stored_roscfg.params.items()]
#                     new_values = [(name, str(p.value)) for name, p in cfg.roscfg.params.items()]
#                     # detect changes parameter
#                     paramset = set(name for name, _ in (set(new_values) - set(stored_values)))  # _:=value
#                     # detect new parameter
#                     paramset |= (set(cfg.roscfg.params.keys()) - set(stored_roscfg.params.keys()))
#                     # detect removed parameter
#                     paramset |= (set(stored_roscfg.params.keys()) - set(cfg.roscfg.params.keys()))
#                     # detect new nodes
#                     stored_nodes = [ns_join(item.namespace, item.name) for item in stored_roscfg.nodes]
#                     new_nodes = [ns_join(item.namespace, item.name) for item in cfg.roscfg.nodes]
#                     nodes2start = set(new_nodes) - set(stored_nodes)
#                     # determine the nodes of the changed parameter
#                     for p in paramset:
#                         for n in new_nodes:
#                             if p.startswith(n):
#                                 nodes2start.add(n)
#                     # detect changes in the arguments and remap
#                     for n in stored_roscfg.nodes:
#                         for new_n in cfg.roscfg.nodes:
#                             if n.name == new_n.name and n.namespace == new_n.namespace:
#                                 if n.args != new_n.args or n.remap_args != new_n.remap_args:
#                                     nodes2start.add(ns_join(n.namespace, n.name))
#                     # filter out anonymous nodes
#                     for n in nodes2start:
#                         if not re.search(r"\d{3,6}_\d{10,}", n):
#                             result.changed_nodes.append(n)
# #                    result.changed_nodes.extend([n for n in nodes2start if not re.search(r"\d{3,6}_\d{10,}", n)])
            except Exception as e:
                print(traceback.format_exc())
                err_text = "%s loading failed!" % request.path
                err_details = "%s: %s" % (err_text, e)
                nmd.ros_node.get_logger().warn("Loading launch file: %s" % err_details)
                result.status.code = ERROR
                result.status.error_msg = err_details
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def UnloadLaunch(self, request, context):
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.daemonuri)
        if cfgid in self._loaded_files:
            try:
                del self._loaded_files[cfgid]
                result.status.code = OK
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, e)
                nmd.ros_node.get_logger().warn("Unloading launch file: %s" % err_details)
                result.status.code = ERROR
                result.status.error_msg = err_details
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def GetNodes(self, request, context):
        requested_files = []
        lfiles = request.launch_files
        for lfile in lfiles:
            requested_files.append(CfgId(lfile, request.daemonuri))
        if not requested_files:
            requested_files = self._loaded_files.keys()
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply = lmsg.LaunchContent(
                launch_file=cfgid.path, daemonuri=lc.daemonuri)
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
                ni = lmsg.NodeInfo(name=node_fullname, is_executable=is_executable,
                                   composable_container=composable_container)
                # add composable nodes to container
                for cn in item.composable_nodes:
                    ni.composable_nodes.extend(
                        [LaunchConfig.get_name_from_node(cn)])
                reply.nodes.extend([ni])
            # fill the robot description and node capability groups
            if request.request_description and False:
                try:
                    rd_hosts = []
                    # get the robot description
                    robot_desr = lc.get_robot_descr()
                    for host, descr in robot_desr.items():
                        rd = lmsg.RobotDescription()
                        rd.machine = host
                        rd.robot_name = descr['name']
                        rd.robot_type = descr['type']
                        rd.robot_images.extend(descr['images'])
                        rd.robot_descr = descr['description']
                        if host in lc.roscfg.machines:
                            rd.machine = lc.roscfg.machines[host].address
                        rd_hosts.append(rd)
                    # get the sensor description
                    tmp_cap_dict = lc.get_capabilitie_desrc()
                    for machine, ns_dict in tmp_cap_dict.items():
                        rd = None
                        if machine not in rd_hosts:
                            rd = lmsg.RobotDescription(machine=machine)
                            rd_hosts.append(rd)
                        else:
                            rd = rd_hosts[machine]
                        caps = []
                        for ns, group_dict in ns_dict.items():
                            for group, descr_dict in group_dict.items():
                                if descr_dict['nodes']:
                                    cap = lmsg.Capability()
                                    cap.namespace = ns
                                    cap.name = group
                                    cap.type = descr_dict['type']
                                    cap.images.extend(descr_dict['images'])
                                    cap.description = descr_dict['description']
                                    cap.nodes.extend(descr_dict['nodes'])
                                    caps.append(cap)
                        rd.capabilities.extend(caps)
                    reply.description.extend(rd_hosts)
                except Exception:
                    print(traceback.format_exc())
            yield reply

    def StartNode(self, request_iterator, context):
        for request in request_iterator:
            try:
                nmd.ros_node.get_logger().debug('Start node %s' % request.name)
                result = lmsg.StartNodeReply(name=request.name)
                launch_configs = []
                if request.opt_launch:
                    nmd.ros_node.get_logger().debug('Use provided launch file=%s; daemonuri=%s;' %
                                                   (request.opt_launch, request.daemonuri))
                    cfgid = CfgId(request.opt_launch, request.daemonuri)
                    if cfgid in self._loaded_files:
                        launch_configs.append(self._loaded_files[cfgid])
                if not launch_configs:
                    # get launch configurations with given node
                    launch_configs = []
                    for cfgid, launchcfg in self._loaded_files.items():
                        n = launchcfg.get_node(request.name)
                        if n is not None:
                            nmd.ros_node.get_logger().debug('Found launch file=%s;' % (launchcfg.filename))
                            launch_configs.append(launchcfg)
                if not launch_configs:
                    result.status.code = NODE_NOT_FOUND
                    result.status.error_msg = "Node '%s' not found" % request.name
                    yield result
                if len(launch_configs) > 1:
                    result.status.code = MULTIPLE_LAUNCHES
                    result.status.error_msg = "Node '%s' found in multiple launch files" % request.name
                    result.launch.extend(
                        [lcfg.filename for lcfg in launch_configs])
                    yield result
                try:
                    result.launch.append(launch_configs[0].filename)
                    n = launch_configs[0].get_node(request.name)
                    if n is not None:
                        if n.composable_container:
                            # load plugin in container
                            container_name = launch_configs[0].get_name_from_node(
                                n.composable_container)
                            nmd.ros_node.get_logger().debug('Load node=%s; as plugin into container=%s;' %
                                                           (request.name, container_name))
                            launcher.run_composed_node(
                                n.node, container_name=container_name, context=launch_configs[0].context)
                        else:
                            startcfg = launcher.from_node(
                                n.node, launchcfg=launch_configs[0], executable=request.opt_binary, loglevel=request.loglevel, logformat=request.logformat, cmd_prefix=request.cmd_prefix)
                            launcher.run_node(startcfg)
                            nmd.ros_node.get_logger().debug('Node=%s; start finished' % (request.name))
                        result.status.code = OK
                    yield result
                except exceptions.BinarySelectionRequest as bsr:
                    result.status.code = MULTIPLE_BINARIES
                    result.status.error_msg = "multiple binaries found for node '%s': %s" % (
                        request.name, bsr.choices)
                    result.path.extend(bsr.choices)
                    yield result
                except grpc.RpcError as conerr:
                    result.status.code = CONNECTION_ERROR
                    result.status.error_msg = conerr
                    yield result
            except exceptions.ResourceNotFound as err_nf:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (
                    request.name, err_nf)
                yield result
            except Exception as _errr:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (
                    request.name, traceback.format_exc())
                yield result

    def _execute_process(self, context: LaunchContext):
        print('MY _execute_process')

    def StartStandaloneNode(self, request, context):
        result = lmsg.StartNodeReply(name=request.name)
        try:
            startcfg = StartConfig.from_msg(request)
            try:
                launcher.run_node(startcfg)
                result.status.code = OK
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = MULTIPLE_BINARIES
                result.status.error_msg = "multiple binaries found for node '%s': %s" % (
                    request.name, bsr.choices)
                result.launch.extend(bsr.choices)
        except grpc.RpcError as conerr:
            result.status.code = CONNECTION_ERROR
            result.status.error_msg = conerr
        except Exception:
            result = lmsg.StartNodeReply(name=request.name)
            result.status.code = ERROR
            result.status.error_msg = "Error while start node '%s': %s" % (
                request.name, traceback.format_exc())
        return result

    def GetIncludedFiles(self, request, context):
        try:
            pattern = INCLUDE_PATTERN
            if request.pattern:
                pattern = request.pattern
            search_in_ext = SEARCH_IN_EXT
            if request.search_in_ext:
                search_in_ext = request.search_in_ext
            # search for loaded file and get the arguments
            resolve_args = {
                arg.name: arg.value for arg in request.include_args}
            if not resolve_args:
                for cfgid, lcfg in self._loaded_files.items():
                    if cfgid.path == request.path:
                        resolve_args.update(lcfg.resolve_dict)
                        break
            # replay each file
            for inc_file in find_included_files(request.path, recursive=request.recursive, unique=request.unique, include_pattern=pattern, search_in_ext=search_in_ext, resolve_args=resolve_args, rosnode=nmd.ros_node):
                reply = lmsg.IncludedFilesReply()
                reply.root_path = inc_file.path_or_str
                reply.linenr = inc_file.line_number
                reply.path = inc_file.inc_path
                reply.exists = inc_file.exists
                reply.rawname = inc_file.raw_inc_path
                if reply.exists:
                    reply.size = os.path.getsize(reply.path)
                reply.rec_depth = inc_file.rec_depth
                reply.include_args.extend(lmsg.Argument(
                    name=name, value=value) for name, value in inc_file.args.items())
                # return each file one by one
                yield reply
        except Exception:
            nmd.ros_node.get_logger().warn("Can't get include files for %s: %s" %
                                          (request.path, traceback.format_exc()))

    def InterpretPath(self, request, context):
        for text in request.paths:
            if text:
                reply = lmsg.InterpredPath()
                try:
                    reply.path = interpret_path(text)
                    reply.exists = os.path.exists(reply.path)
                    reply.status.code = OK
                except Exception as err:
                    reply.status.code = PACKAGE_NOT_FOUND
                    reply.status.error_msg = err
                yield reply

    def GetMtime(self, request, context):
        try:
            result = lmsg.MtimeReply()
            result.path = request.path
            already_in = []
            mtime = 0
            if os.path.exists(request.path):
                mtime = os.path.getmtime(request.path)
                already_in.append(request.path)
            result.mtime = mtime
            # search for loaded file and get the arguments
            resolve_args = {}
            for cfgid, lcfg in self._loaded_files.items():
                if cfgid.path == request.path:
                    resolve_args.update(lcfg.resolve_dict)
                    break
            # add mtimes for all included files
            inc_files = find_included_files(request.path, recursive=True, unique=True,
                                            include_pattern=INCLUDE_PATTERN, search_in_ext=SEARCH_IN_EXT, resolve_args=resolve_args)
            for inc_file in inc_files:
                incf = inc_file.inc_path
                if incf not in already_in:
                    mtime = 0
                    if os.path.exists(incf):
                        mtime = os.path.getmtime(incf)
                    result.included_files.extend(
                        [lmsg.FileObj(path=incf, mtime=mtime)])
                    already_in.append(incf)
            return result
        except Exception:
            nmd.ros_node.get_logger().warn(traceback.format_exc())

    def GetChangedBinaries(self, request, context):
        result = lmsg.MtimeNodes()
        changed = launcher.changed_binaries([node for node in request.names])
        nodes = []
        for name, mtime in changed:
            node = lmsg.MtimeNode(name=name, mtime=mtime)
            nodes.append(node)
        result.nodes.extend(nodes)
        return result

    def GetStartCfg(self, request, context):
        result = lmsg.StartCfgReply(name=request.name)
        launch_configs = []
        if request.opt_launch:
            cfgid = CfgId(request.opt_launch, request.daemonuri)
            if cfgid in self._loaded_files:
                launch_configs.append(self._loaded_files[cfgid])
        if not launch_configs:
            # get launch configurations with given node
            launch_configs = []
            for cfgid, launchcfg in self._loaded_files.items():
                # if cfgid.equal_masteruri(request.daemonuri):
                n = launchcfg.get_node(request.name)
                if n is not None:
                    launch_configs.append(launchcfg)
        if not launch_configs:
            result.status.code = NODE_NOT_FOUND
            result.status.error_msg = "Node '%s' not found" % request.name
            return result
        if len(launch_configs) > 1:
            result.status.code = MULTIPLE_LAUNCHES
            result.status.error_msg = "Node '%s' found in multiple launch files" % request.name
            result.launch.extend([lcfg.filename for lcfg in launch_configs])
            return result
        result.launch.append(launch_configs[0].filename)
        startcfg = launcher.create_start_config(
            request.name, launch_configs[0], request.opt_binary, daemonuri=request.daemonuri, loglevel=request.loglevel, logformat=request.logformat)
        startcfg.fill_msg(result.startcfg)
        result.status.code = OK
        return result

    def ResetPackageCache(self, request, context):
        result = lmsg.Empty()
        reset_package_cache()
        return result

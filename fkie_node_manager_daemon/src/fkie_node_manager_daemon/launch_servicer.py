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


import grpc
import os
import re
import rospy
import roslib.message
import roslib.msgs
import roslib.names
import roslib.packages
import rospkg
import ruamel.yaml
import shlex
import subprocess
import threading
import traceback

import json
from types import SimpleNamespace
import asyncio
from autobahn import wamp

from typing import List
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from watchdog.events import FileSystemEvent

import fkie_multimaster_msgs.grpc.launch_pb2_grpc as lgrpc
import fkie_multimaster_msgs.grpc.launch_pb2 as lmsg

from . import launcher
from fkie_node_manager_daemon.strings import utf8
from .launch_config import LaunchConfig
from .startcfg import StartConfig
from fkie_multimaster_msgs import ros_pkg
from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.crossbar.runtime_interface import SubscriberNode
from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchArgument
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchCallService
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchFile
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchLoadRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchLoadReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchContent
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchAssociations
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNode
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNodeInfo
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNodeReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchInterpretPathRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchInterpretPathReply
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchIncludedFilesRequest
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchIncludedFile
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchMessageStruct
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchPublishMessage
from fkie_multimaster_msgs.defines import SEARCH_IN_EXT
from fkie_multimaster_msgs.launch import xml
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system import exceptions
from fkie_multimaster_msgs.system import ros1_masteruri
from fkie_multimaster_msgs.system.url import equal_uri
from fkie_multimaster_msgs.system.supervised_popen import SupervisedPopen

from fkie_multimaster_msgs.msg import LinkStatesStamped

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

    def __init__(self, path, masteruri=''):
        '''
        :param str path: absolute path of the launch file.
        :param str masteruri: ROS-Masteruri if empty the masteruri will be determine by :meth:`from fkie_multimaster_msgs.system.ros1_masteruri
.from_master()`
        '''
        self.path = path
        self.masteruri = masteruri
        self._local = False
        if not masteruri:
            self.masteruri = ros1_masteruri.from_master(True)
            self._local = True
        elif equal_uri(self.masteruri, ros1_masteruri.from_master(True)):
            self._local = True

    def __hash__(self, *args, **kwargs):
        return hash("%s%s" % (self.masteruri, self.path))

    def __eq__(self, other):
        '''
        Compares the path of the item.
        '''
        if isinstance(other, tuple):
            return self.path == other[0] and self.equal_masteruri(other[1])
        elif other is not None:
            return self.path == other.path and self.equal_masteruri(other.masteruri)
        return False

    def __ne__(self, other):
        return not (self == other)

    def equal_masteruri(self, masteruri):
        '''
        Compares the ROS-Masteruri of this instance with other ROS-Masteruri.

        :param str masteruri: ROS Masteruri
        '''
        if not masteruri:
            if self._local:
                return True
        if equal_uri(self.masteruri, masteruri):
            return True
        return False


class LaunchServicer(lgrpc.LaunchServiceServicer, CrossbarBaseSession, LoggingEventHandler):
    '''
    Handles GRPC-requests defined in `launch.proto`.
    '''

    def __init__(self, monitor_servicer, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, test_env=False):
        Log.info("Create launch manger servicer")
        lgrpc.LaunchServiceServicer.__init__(self)
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env)
        LoggingEventHandler.__init__(self)
        self._watchdog_observer = Observer()
        self._observed_dirs = {}  # path: watchdog.observers.api.ObservedWatch
        self._included_files = []
        self._included_dirs = []
        self._launch_includes = {}
        self._is_running = True
        self._peers = {}
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)
        self._monitor_servicer = monitor_servicer
        self._watchdog_observer.start()
        self.subscribe_to('ros.nodes.abort', self.crossbar_abort)
        print(self.get_msg_struct("fkie_multimaster_msgs/LinkStatesStamped"))

    def _terminated(self):
        Log.info("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            Log.info("Add callback to peer context @%s" % context.peer())
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        '''
        Cancel the autostart of the nodes.
        '''
        global IS_RUNNING
        IS_RUNNING = False
        self.shutdown()
        self._watchdog_observer.stop()

    def on_any_event(self, event: FileSystemEvent):
        if event.src_path in self._included_files:
            affected_launch_files = []
            for launch_path, path_list in self._launch_includes.items():
                if event.src_path in path_list:
                    affected_launch_files.append(launch_path)
            change_event = {event.event_type: event.src_path,
                            'affected': affected_launch_files}
            Log.debug("observed change %s on %s" %
                      (event.event_type, event.src_path))
            self.publish_to('ros.path.changed', change_event)

    def load_launch_file(self, path, autostart=False):
        '''
        Load launch file and start all included nodes regarding the autostart parameter:

        - <param name="autostart/exclude" value="false" />
        - <param name="autostart/delay" value="5.0" />
        - <param name="autostart/required/publisher" value="topic_name" />

        :param str path: the absolute path of the launch file
        :param bool autostart: True to start all nodes after the launch file was loaded.
        '''
        launch_config = LaunchConfig(
            path, monitor_servicer=self._monitor_servicer)
        loaded, res_argv = launch_config.load([])
        if loaded:
            Log.debug("loaded %s\n  used args: %s" %
                      (path, utf8(res_argv)))
            self._loaded_files[CfgId(path, '')] = launch_config
            if autostart:
                start_thread = threading.Thread(
                    target=self._autostart_nodes_threaded, args=(launch_config,))
                start_thread.start()
        else:
            Log.warn("load %s failed!" % (path))

    def start_node_by_name(self, node_name):
        global IS_RUNNING
        if not IS_RUNNING:
            return
        for _cfgid, launchcfg in self._loaded_files.items():
            n = launchcfg.get_node(node_name)
            if n is not None:
                startcfg = launcher.create_start_config(
                    node_name, launchcfg, '', masteruri='', loglevel='', reload_global_param=False)
                launcher.run_node(startcfg)
                return
        raise Exception("Node '%s' not found!" % node_name)

    def _autostart_nodes_threaded(self, cfg):
        global IS_RUNNING
        for item in cfg.roscfg.nodes:
            if not IS_RUNNING:
                return
            node_fullname = roslib.names.ns_join(item.namespace, item.name)
            try:
                if self._get_start_exclude(cfg, node_fullname):
                    # skip autostart
                    Log.debug(
                        "%s is in exclude list, skip autostart", node_fullname)
                    continue
                self._autostart_node(node_fullname, cfg)
            except Exception as err:
                Log.warn("Error while start %s: %s", node_fullname, err)

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
                node_name, cfg, '', masteruri='', loglevel='', reload_global_param=False)
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
        param_name = rospy.names.ns_join(node, 'autostart/exclude')
        try:
            return bool(cfg.roscfg.params[param_name].value)
        except Exception:
            pass
        return False

    def _get_start_delay(self, cfg, node):
        param_name = rospy.names.ns_join(node, 'autostart/delay')
        try:
            return float(cfg.roscfg.params[param_name].value)
        except Exception:
            pass
        return 0.

    def _get_start_required(self, cfg, node):
        param_name = rospy.names.ns_join(node, 'autostart/required/publisher')
        topic = ''
        try:
            topic = cfg.roscfg.params[param_name].value
            if topic:
                import rosgraph
                if rosgraph.names.is_private(topic):
                    Log.warn(
                        'Private for autostart required topic `%s` is ignored!' % topic)
                    topic = ''
                elif not rosgraph.names.is_global(topic):
                    topic = rospy.names.ns_join(
                        rosgraph.names.namespace(node), topic)
        except Exception:
            pass
        return topic

    def _add_file_to_observe(self, path, launch_file=''):
        directory = os.path.dirname(path)
        Log.debug('observe path: %s' % path)
        if directory not in self._observed_dirs:
            Log.debug('add directory to observer: %s' % directory)
            watch = self._watchdog_observer.schedule(self, directory)
            self._observed_dirs[directory] = watch
        self._included_files.append(path)
        self._included_dirs.append(directory)
        if launch_file:
            if launch_file not in self._launch_includes:
                self._launch_includes[launch_file] = []
            self._launch_includes[launch_file].append(path)

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
            self._add_file_to_observe(path, path)
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
                    self._add_file_to_observe(inc_file.inc_path, path)
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
            del self._launch_includes[path]
        except Exception as e:
            Log.error('_add_launch_to_observer %s:\n%s' % (str(path), e))

    def GetLoadedFiles(self, request, context):
        Log.debug('GetLoadedFiles request:\n%s' % str(request))
        # self._register_callback(context)
        for _cfgid, lf in self._loaded_files.items():
            reply = lmsg.LoadedFile(package=lf.packagename, launch=lf.launchname,
                                    path=lf.filename, masteruri=lf.masteruri, host=lf.host)
            reply.args.extend(lmsg.Argument(name=name, value=value)
                              for name, value in lf.resolve_dict.items())
            yield reply

    def LoadLaunch(self, request, context):
        '''
        Loads launch file
        '''
        result = lmsg.LoadLaunchReply()
        launchfile = request.path
        Log.debug("Loading launch file: %s (package: %s, launch: %s), masteruri: %s, host: %s, args: %s" % (
            launchfile, request.package, request.launch, request.masteruri, request.host, request.args))
        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = roslib.packages.find_resource(
                    request.package, request.launch)
                if not paths:
                    result.status.code = FILE_NOT_FOUND
                    result.status.error_msg = utf8(
                        "Launch files %s in package %s found!" % (request.launch, request.package))
                    return result
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = MULTIPLE_LAUNCHES
                        result.status.error_msg = utf8(
                            "Multiple launch files with name %s in package %s found!" % (request.launch, request.package))
                        for mp in paths:
                            result.path.append(mp)
                        Log.debug("..load aborted, MULTIPLE_LAUNCHES")
                        return result
                else:
                    launchfile = paths[0]
            except rospkg.ResourceNotFound as rnf:
                result.status.code = FILE_NOT_FOUND
                result.status.error_msg = utf8(
                    "Package %s not found: %s" % (request.package, rnf))
                Log.debug("..load aborted, FILE_NOT_FOUND")
                return result
        result.path.append(launchfile)
        # it is already loaded?
        if (launchfile, request.masteruri) in list(self._loaded_files.keys()):
            result.status.code = ALREADY_OPEN
            result.status.error_msg = utf8(
                "Launch file %s already loaded!" % (launchfile))
            Log.debug("..load aborted, ALREADY_OPEN")
            return result
        # load launch configuration
        try:
            # test for required args
            provided_args = ["%s" % arg.name for arg in request.args]
            launch_config = LaunchConfig(
                launchfile, masteruri=request.masteruri, host=request.host, monitor_servicer=self._monitor_servicer)
            # get the list with needed launch args
            req_args = launch_config.get_args()
            req_args_dict = launch_config.argv2dict(req_args)
            if request.request_args and req_args:
                for arg, value in req_args_dict.items():
                    if arg not in provided_args:
                        result.args.extend(
                            [lmsg.Argument(name=arg, value=value) for arg, value in req_args_dict.items()])
                        result.status.code = PARAMS_REQUIRED
                        Log.debug("..load aborted, PARAMS_REQUIRED")
                        return result
            argv = ["%s:=%s" % (arg.name, arg.value)
                    for arg in request.args if arg.name in req_args_dict]
            _loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            result.args.extend([lmsg.Argument(name=name, value=value)
                                for name, value in launch_config.resolve_dict.items()])
            self._loaded_files[CfgId(
                launchfile, request.masteruri)] = launch_config
            Log.debug("..load complete!")

            # notify changes to crossbar GUI
            self.publish_to('ros.launch.changed', {})
            self._add_launch_to_observer(launchfile)
        except Exception as e:
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, utf8(e))
            Log.warn("Loading launch file: %s", err_details)
            result.status.code = ERROR
            result.status.error_msg = utf8(err_details)
            return result
        result.status.code = OK
        return result

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
        Log.debug('Loading launch file: %s (package: %s, launch: %s), masteruri: %s, host: %s, args: %s' % (
            launchfile, request.ros_package, request.launch, request.masteruri, request.host, request.args))

        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = roslib.packages.find_resource(
                    request.ros_package, request.launch)
                if not paths:
                    result.status.code = 'FILE_NOT_FOUND'
                    result.status.msg = utf8('Launch files %s in package %s found!' % (
                        request.launch, request.ros_package))
                    return json.dumps(result, cls=SelfEncoder)
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = 'MULTIPLE_LAUNCHES'
                        result.status.msg = utf8('Multiple launch files with name %s in package %s found!' % (
                            request.launch, request.ros_package))
                        for mp in paths:
                            result.paths.append(mp)
                        Log.debug('..load aborted, MULTIPLE_LAUNCHES')
                        return json.dumps(result, cls=SelfEncoder)
                else:
                    launchfile = paths[0]
            except rospkg.ResourceNotFound as rnf:
                result.status.code = 'FILE_NOT_FOUND'
                result.status.msg = utf8(
                    'Package %s not found: %s' % (request.ros_package, rnf))
                Log.debug('..load aborted, FILE_NOT_FOUND')
                return json.dumps(result, cls=SelfEncoder)
        result.paths.append(launchfile)

        # it is already loaded?
        if (launchfile, request.masteruri) in list(self._loaded_files.keys()):
            result.status.code = 'ALREADY_OPEN'
            result.status.msg = utf8(
                'Launch file %s already loaded!' % (launchfile))
            Log.debug('..load aborted, ALREADY_OPEN')
            return json.dumps(result, cls=SelfEncoder)

        # load launch configuration
        try:
            # test for required args
            provided_args = ['%s' % arg.name for arg in request.args]
            provided_args_dict = {arg.name: arg.value for arg in request.args}
            launch_config = LaunchConfig(
                launchfile, masteruri=request.masteruri, host=request.host, monitor_servicer=self._monitor_servicer)

            # get the list with needed launch args
            req_args = launch_config.get_args()
            req_args_dict = launch_config.argv2dict(req_args)

            if request.request_args and req_args:
                for arg, value in req_args_dict.items():
                    if arg not in provided_args:
                        # result.args.append([LaunchArgument(name=arg, value=value) for arg, value in req_args_dict.items()])
                        la_value = value
                        default_value = None
                        if arg not in provided_args_dict:
                            la_value = None
                            default_value = value
                        else:
                            default_value = provided_args_dict[arg]
                        result.args.append(
                            LaunchArgument(name=arg, value=la_value, default_value=default_value))

                if len(result.args) > 0:
                    result.status.code = 'PARAMS_REQUIRED'
                    Log.debug('..load aborted, PARAMS_REQUIRED')
                    return json.dumps(result, cls=SelfEncoder)

            argv = ['%s:=%s' % (arg.name, arg.value)
                    for arg in request.args if arg.name in req_args_dict]
            _loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            for name, value in launch_config.resolve_dict.items():
                if name in req_args_dict:
                    result.args.append(LaunchArgument(
                        name=name, value=value, default_value=req_args_dict[name]))
                else:
                    result.args.append(LaunchArgument(
                        name=name, value=None, default_value=value))
            self._loaded_files[CfgId(
                launchfile, request.masteruri)] = launch_config
            Log.debug('..load complete!')

            # notify changes to crossbar GUI
            self.publish_to('ros.launch.changed', {})
            self._add_launch_to_observer(launchfile)
        except Exception as e:
            err_text = '%s loading failed!' % launchfile
            err_details = '%s: %s' % (err_text, utf8(e))
            Log.warn('Loading launch file: %s', err_details)
            result.status.code = 'ERROR'
            result.status.msg = utf8(err_details)
            return json.dumps(result, cls=SelfEncoder)
        result.status.code = 'OK'
        return json.dumps(result, cls=SelfEncoder)

    def ReloadLaunch(self, request, context):
        Log.debug('ReloadLaunch request:\n%s' % str(request))
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        Log.debug("reload launch file: %s, masteruri: %s",
                  request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(request.path)
                # use argv from already open file
                cfg = self._loaded_files[cfgid]
                stored_roscfg = cfg.roscfg
                argv = cfg.argv
                cfg.load(argv)
                result.status.code = OK
                # detect files changes
                if stored_roscfg and cfg.roscfg:
                    stored_values = [(name, utf8(p.value))
                                     for name, p in stored_roscfg.params.items()]
                    new_values = [(name, utf8(p.value))
                                  for name, p in cfg.roscfg.params.items()]
                    # detect changes parameter
                    paramset = set(name for name, _ in (
                        set(new_values) - set(stored_values)))  # _:=value
                    # detect new parameter
                    paramset |= (set(cfg.roscfg.params.keys()) -
                                 set(stored_roscfg.params.keys()))
                    # detect removed parameter
                    paramset |= (set(stored_roscfg.params.keys()) -
                                 set(cfg.roscfg.params.keys()))
                    # detect new nodes
                    stored_nodes = [roslib.names.ns_join(
                        item.namespace, item.name) for item in stored_roscfg.nodes]
                    new_nodes = [roslib.names.ns_join(
                        item.namespace, item.name) for item in cfg.roscfg.nodes]
                    nodes2start = set(new_nodes) - set(stored_nodes)
                    # determine the nodes of the changed parameter
                    for p in paramset:
                        for n in new_nodes:
                            if p.startswith(n):
                                nodes2start.add(n)
                    # detect changes in the arguments and remap
                    for n in stored_roscfg.nodes:
                        for new_n in cfg.roscfg.nodes:
                            if n.name == new_n.name and n.namespace == new_n.namespace:
                                if n.args != new_n.args or n.remap_args != new_n.remap_args:
                                    nodes2start.add(
                                        roslib.names.ns_join(n.namespace, n.name))
                    # filter out anonymous nodes
                    for n in nodes2start:
                        if not re.search(r"\d{3,6}_\d{10,}", n):
                            result.changed_nodes.append(n)
#                    result.changed_nodes.extend([n for n in nodes2start if not re.search(r"\d{3,6}_\d{10,}", n)])

                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
                self._add_launch_to_observer(request.path)
            except Exception as e:
                print(traceback.format_exc())
                self._add_launch_to_observer(request.path)
                err_text = "%s loading failed!" % request.path
                err_details = "%s: %s" % (err_text, utf8(e))
                Log.warn("Loading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

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

        Log.debug('Loading launch file: %s (package: %s, launch: %s), masteruri: %s, host: %s, args: %s' % (
            request.path, request.ros_package, request.launch, request.masteruri, request.host, request.args))

        result.paths.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        Log.debug("reload launch file: %s, masteruri: %s",
                  request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(request.path)
                # use argv from already open file
                cfg = self._loaded_files[cfgid]
                stored_roscfg = cfg.roscfg
                argv = cfg.argv
                cfg.load(argv)
                result.status.code = 'OK'
                # detect files changes
                if stored_roscfg and cfg.roscfg:
                    stored_values = [(name, utf8(p.value))
                                     for name, p in stored_roscfg.params.items()]
                    new_values = [(name, utf8(p.value))
                                  for name, p in cfg.roscfg.params.items()]
                    # detect changes parameter
                    paramset = set(name for name, _ in (
                        set(new_values) - set(stored_values)))  # _:=value
                    # detect new parameter
                    paramset |= (set(cfg.roscfg.params.keys()) -
                                 set(stored_roscfg.params.keys()))
                    # detect removed parameter
                    paramset |= (set(stored_roscfg.params.keys()) -
                                 set(cfg.roscfg.params.keys()))
                    # detect new nodes
                    stored_nodes = [roslib.names.ns_join(
                        item.namespace, item.name) for item in stored_roscfg.nodes]
                    new_nodes = [roslib.names.ns_join(
                        item.namespace, item.name) for item in cfg.roscfg.nodes]
                    nodes2start = set(new_nodes) - set(stored_nodes)
                    # determine the nodes of the changed parameter
                    for p in paramset:
                        for n in new_nodes:
                            if p.startswith(n):
                                nodes2start.add(n)
                    # detect changes in the arguments and remap
                    for n in stored_roscfg.nodes:
                        for new_n in cfg.roscfg.nodes:
                            if n.name == new_n.name and n.namespace == new_n.namespace:
                                if n.args != new_n.args or n.remap_args != new_n.remap_args:
                                    nodes2start.add(
                                        roslib.names.ns_join(n.namespace, n.name))
                    # filter out anonymous nodes
                    for n in nodes2start:
                        if not re.search(r"\d{3,6}_\d{10,}", n):
                            result.changed_nodes.append(n)

                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
                self._add_launch_to_observer(request.path)
            except Exception as e:
                print(traceback.format_exc())
                self._add_launch_to_observer(request.path)
                err_text = f"{request.path} loading failed!"
                err_details = f"{err_text}: {e}"
                Log.warn("Loading launch file: %s", err_details)
                result.status.code = 'ERROR'
                result.status.msg = err_details
                return json.dumps(result, cls=SelfEncoder)
        else:
            result.status.code = 'FILE_NOT_FOUND'
            return json.dumps(result, cls=SelfEncoder)
        return json.dumps(result, cls=SelfEncoder)

    def UnloadLaunch(self, request, context):
        Log.debug('UnloadLaunch request:\n%s' % str(request))
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(request.path)
                del self._loaded_files[cfgid]
                result.status.code = OK

                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, utf8(e))
                Log.warn("Unloading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
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
        cfgid = CfgId(request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                self._remove_launch_from_observer(request.path)
                del self._loaded_files[cfgid]
                result.status.code = 'OK'

                # notify changes to crossbar GUI
                self.publish_to('ros.launch.changed', {})
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, utf8(e))
                Log.warn("Unloading launch file: %s", err_details)
                result.status.code = 'ERROR'
                result.status.msg = utf8(err_details)
        else:
            result.status.code = 'FILE_NOT_FOUND'
        return json.dumps(result, cls=SelfEncoder)

    def GetNodes(self, request, context):
        Log.debug('GetNodes request:\n%s' % str(request))
        requested_files = []
        lfiles = request.launch_files
        for lfile in lfiles:
            requested_files.append(CfgId(lfile, request.masteruri))
        if not requested_files:
            requested_files = list(self._loaded_files.keys())
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply = lmsg.LaunchContent(
                launch_file=cfgid.path, masteruri=lc.masteruri, host=lc.host)
            for item in lc.roscfg.nodes:
                node_fullname = roslib.names.ns_join(item.namespace, item.name)
                reply.node.append(node_fullname)
            # fill the robot description and node capability groups
            if request.request_description:
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
            # create nodelets description
            nodelets = {}
            for n in lc.roscfg.nodes:
                if n.package == 'nodelet' and n.type == 'nodelet':
                    args = n.args.split(' ')
                    if len(args) == 3 and args[0] == 'load':
                        nodelet_mngr = roslib.names.ns_join(
                            n.namespace, args[2])
                        if nodelet_mngr not in nodelets:
                            nodelets[nodelet_mngr] = []
                        nodelets[nodelet_mngr].append(
                            roslib.names.ns_join(n.namespace, n.name))
            for mngr, ndl in nodelets.items():
                nlmsg = lmsg.Nodelets(manager=mngr)
                nlmsg.nodes.extend(ndl)
                reply.nodelets.extend([nlmsg])
            # create association description
            associations = {}
            for n in lc.roscfg.nodes:
                node_fullname = roslib.names.ns_join(n.namespace, n.name)
                associations_param = roslib.names.ns_join(
                    node_fullname, 'nm/associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(
                            item.namespace, split))
                    associations[node_fullname] = values
                # DEPRECATED 'associations'
                associations_param = roslib.names.ns_join(
                    node_fullname, 'associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(
                            item.namespace, split))
                    associations[node_fullname] = values
            for node, ass in associations.items():
                assmsg = lmsg.Associations(node=node)
                assmsg.nodes.extend(ass)
                reply.associations.extend([assmsg])
            yield reply

    @wamp.register('ros.launch.get_list')
    def get_list(self) -> List[LaunchContent]:
        Log.debug('Request to [ros.launch.get_list]')
        requested_files = list(self._loaded_files.keys())
        reply = []
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply_lc = LaunchContent(path=cfgid.path, args=[], masteruri=lc.masteruri, host=lc.host,
                                     nodes=[], parameters=[], associations=[])

            # Add launch arguments
            for name, arg in lc.argv2dict(lc.argv).items():
                reply_lc.args.append(LaunchArgument(name, arg))

            node_occurrence = {}
            for item in lc.roscfg.nodes:
                node_fullname = roslib.names.ns_join(item.namespace, item.name)
                if item.launch_name not in node_occurrence:
                    node_occurrence[item.launch_name] = 0
                else:
                    node_occurrence[item.launch_name] += 1

                #  Search the line number of a given node in launch file
                lines_with_node_name = []
                with open(item.filename, 'r') as launch_file:
                    for line_number, line_text in enumerate(launch_file):
                        if f'name="{item.launch_name}"' in line_text:
                            lines_with_node_name.append(
                                [line_number + 1, line_text])

                line_number = -1
                start_column = 0
                end_column = 0
                line_text = ""
                if len(lines_with_node_name) == 0:
                    # no line found. TODO: Report error?
                    line_number = 0
                elif len(lines_with_node_name) == 1:
                    line_number = lines_with_node_name[0][0]
                    line_text = lines_with_node_name[0][1]
                elif len(lines_with_node_name) > node_occurrence[item.launch_name]:
                    # More than one occurrence, but Node are loaded from top to bottom
                    # try to find the correct match
                    line_number = lines_with_node_name[node_occurrence[item.launch_name]][0]
                    line_text = lines_with_node_name[node_occurrence[item.launch_name]][1]

                if len(line_text) > 0:
                    start_column = line_text.index(
                        f'name="{item.launch_name}"') + 7
                    end_column = start_column + len(item.launch_name)

                # range in text where the node appears
                file_range = {"startLineNumber": line_number,
                              "endLineNumber": line_number,
                              "startColumn": start_column,
                              "endColumn": end_column}

                composable_container = None
                if item.package == 'nodelet' and item.type == 'nodelet':
                    args = item.args.split(' ')
                    if len(args) == 3 and args[0] == 'load':
                        composable_container = roslib.names.ns_join(
                            item.namespace, args[2])

                reply_lc.nodes.append(LaunchNodeInfo(node_fullname,
                                                     # remove last "/" character in namespace
                                                     node_name=node_fullname,
                                                     name_configured=node_fullname,
                                                     node_namespace=item.namespace[:-1],
                                                     package_name=item.package,
                                                     executable=item.type,
                                                     respawn=item.respawn,
                                                     respawn_delay=item.respawn_delay,
                                                     args=item.args,
                                                     remap_args=item.remap_args,
                                                     additional_env=item.env_args,
                                                     launch_prefix=item.launch_prefix,
                                                     output=item.output,
                                                     required=item.required,
                                                     file_name=item.filename,
                                                     file_range=file_range,
                                                     launch_context_arg=item.launch_context_arg,
                                                     launch_name=item.launch_name,
                                                     composable_container=composable_container)
                                      )

            # Add parameter values
            for name, p in lc.roscfg.params.items():
                reply_lc.parameters.append(RosParameter(name, p.value))

            # create association description
            associations = {}
            for n in lc.roscfg.nodes:
                node_fullname = roslib.names.ns_join(n.namespace, n.name)
                associations_param = roslib.names.ns_join(
                    node_fullname, 'nm/associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(
                            item.namespace, split))
                    associations[node_fullname] = values
                # DEPRECATED 'associations'
                associations_param = roslib.names.ns_join(
                    node_fullname, 'associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(
                            item.namespace, split))
                    associations[node_fullname] = values
            for node, ass in associations.items():
                assmsg = LaunchAssociations(node=node, nodes=ass)
                reply_lc.associations.append(assmsg)
            reply.append(reply_lc)

        return json.dumps(reply, cls=SelfEncoder)

    def StartNode(self, request_iterator, context):
        for request in request_iterator:
            Log.debug('StartNode request:\n%s' % str(request))
            try:
                result = lmsg.StartNodeReply(name=request.name)
                launch_configs = []
                if request.opt_launch:
                    cfgid = CfgId(request.opt_launch, request.masteruri)
                    if cfgid in self._loaded_files:
                        launch_configs.append(self._loaded_files[cfgid])
                if not launch_configs:
                    # get launch configurations with given node
                    launch_configs = []
                    for cfgid, launchcfg in self._loaded_files.items():
                        if cfgid.equal_masteruri(request.masteruri):
                            n = launchcfg.get_node(request.name)
                            if n is not None:
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
                    startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri, loglevel=request.loglevel,
                                                            logformat=request.logformat, reload_global_param=request.reload_global_param, cmd_prefix=request.cmd_prefix)
                    binary = launcher.run_node(startcfg)
                    if binary:
                        self._add_file_to_observe(binary)
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
                    result.status.error_msg = utf8(conerr)
                    yield result
            except exceptions.ResourceNotFound as err_nf:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (
                    request.name, utf8(err_nf))
                yield result
            except Exception as _errr:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (
                    request.name, utf8(traceback.format_exc()))
                yield result

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
                    if cfgid.equal_masteruri(request.masteruri):
                        n = launchcfg.get_node(request.name)
                        if n is not None:
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
                startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri, loglevel=request.loglevel,
                                                        logformat=request.logformat, reload_global_param=request.reload_global_param, cmd_prefix=request.cmd_prefix)
                binary = launcher.run_node(startcfg)
                if binary:
                    self._add_file_to_observe(binary)
                result.status.code = 'OK'
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = 'MULTIPLE_BINARIES'
                result.status.msg = "multiple binaries found for node '%s': %s" % (
                    request.name, bsr.choices)
                result.paths.extend(bsr.choices)
                return json.dumps(result, cls=SelfEncoder)
            except grpc.RpcError as conerr:
                result.status.code = 'CONNECTION_ERROR'
                result.status.msg = utf8(conerr)
                return json.dumps(result, cls=SelfEncoder)
        except exceptions.ResourceNotFound as err_nf:
            result.status.code = 'ERROR'
            result.status.msg = "Error while start node '%s': %s" % (
                request.name, utf8(err_nf))
            return json.dumps(result, cls=SelfEncoder)
        except Exception as _errr:
            result.status.code = 'ERROR'
            result.status.msg = "Error while start node '%s': %s" % (
                request.name, utf8(traceback.format_exc()))
            return json.dumps(result, cls=SelfEncoder)
        finally:
            return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.start_nodes')
    def start_nodes(self, request_json: List[LaunchNode], continue_on_error: bool = True) -> List[LaunchNodeReply]:
        Log.debug('Request to [ros.launch.start_nodes]')

        result = []
        for request in request_json:
            node_result = self.start_nodes(request, return_as_json=False)
            result.append(node_result)
            if not continue_on_error:
                if result.status.code != 'OK':
                    break

        return json.dumps(result, cls=SelfEncoder)

    def crossbar_abort(self, code: str = '') -> None:
        print(
            f"Abort from GUI received (ros.nodes.abort with code '{code}'). Not Implemented!")

    def StartStandaloneNode(self, request, context):
        Log.debug('StartStandaloneNode request:\n%s' % str(request))
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
            result.status.error_msg = utf8(conerr)
        except Exception:
            result = lmsg.StartNodeReply(name=request.name)
            result.status.code = ERROR
            result.status.error_msg = "Error while start node '%s': %s" % (
                request.name, utf8(traceback.format_exc()))
        return result

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
            for inc_file in xml.find_included_files(request.path, request.recursive, request.unique, search_in_ext, resolve_args):
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

    @wamp.register('ros.launch.get_msg_struct')
    def get_msg_struct(self, msg_type: str) -> LaunchMessageStruct:
        Log.info(
            f"Request to [ros.launch.get_msg_struct]: msg [{msg_type}]")
        result = LaunchMessageStruct(msg_type)

        try:
            mclass = roslib.message.get_message_class(msg_type)
            if mclass is None:
                result.error_msg = f"invalid message type: '{msg_type}'. If this is a valid message type, perhaps you need to run 'catkin build'"
                return json.dumps(result, cls=SelfEncoder)
            slots = mclass.__slots__
            types = mclass._slot_types
            msg_dict = {'type': msg_type,
                        'name': '',
                        'def': self._dict_from_slots(slots, types, {})}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.launch.get_srv_struct')
    def get_srv_struct(self, srv_type: str) -> LaunchMessageStruct:
        Log.info(
            f"Request to [ros.launch.get_srv_struct]: msg [{srv_type}]")
        result = LaunchMessageStruct(srv_type)

        try:
            mclass = roslib.message.get_service_class(srv_type)
            if mclass is None:
                result.error_msg = f"invalid service type: '{srv_type}'. If this is a valid service type, perhaps you need to run 'catkin build'"
                return json.dumps(result, cls=SelfEncoder)
            slots = mclass._request_class.__slots__
            types = mclass._request_class._slot_types
            msg_dict = {'type': srv_type,
                        'name': '',
                        'def': self._dict_from_slots(slots, types, {})}
            result.data = msg_dict
            result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
        return json.dumps(result, cls=SelfEncoder)

    @classmethod
    def _dict_from_slots(cls, slots, types, values={}):
        result = []
        for slot, msg_type in zip(slots, types):
            base_type, is_array, _array_length = roslib.msgs.parse_type(
                msg_type)
            if base_type in roslib.msgs.PRIMITIVE_TYPES or base_type in ['time', 'duration']:
                default_value = 'now' if base_type in [
                    'time', 'duration'] else ''
                if slot in values and values[slot]:
                    default_value = values[slot]
                result.append({'type': msg_type, 'name': slot,
                               'def': [], 'default_value': default_value, 'is_array': is_array})
            else:
                try:
                    list_msg_class = roslib.message.get_message_class(
                        base_type)
                    if is_array and slot in values:
                        subresult = []
                        for slot_value in values[slot]:
                            subvalue = cls._dict_from_slots(
                                list_msg_class.__slots__, list_msg_class._slot_types, slot_value if slot in values and slot_value else {})
                            subresult.append(subvalue)
                        result.append(
                            {'type': msg_type, 'name': slot, 'def': subresult, 'default_value': slot_value, 'is_array': is_array})
                    else:
                        subresult = cls._dict_from_slots(
                            list_msg_class.__slots__, list_msg_class._slot_types, values[slot] if slot in values and values[slot] else {})
                        result.append(
                            {'type': msg_type, 'name': slot, 'def': subresult, 'default_value': [], 'is_array': is_array})
                except ValueError as e:
                    print(traceback.format_exc())
                    Log.warn(
                        f"Error while parse message type '{msg_type}': {e}")
                    raise ValueError(
                        f"Error while parse message type '{msg_type}': {e}")
        return result

    def str2typedValue(self, value, value_type):
        result = value
        if 'int' in value_type:
            result = int(value)
        elif 'float' in value_type or 'double' in value_type:
            result = float(value)
        elif value_type.startswith('bool'):
            result = value.lower() in ('yes', 'true', 't', 'y', '1')
        return result

    def _pubstr_from_dict(self, param_dict):
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
                result_array = []
                # it is a complex field type
                if 'value' in field:
                    for array_element in field['value']:
                        result_array.append(
                            self._pubstr_from_dict(array_element))
                # append created array
                if result_array:
                    result[field['name']] = result_array
            else:
                subresult = self._pubstr_from_dict(field['def'])
                if subresult:
                    result[field['name']] = subresult
        return result

    @wamp.register('ros.launch.publish_message')
    def publish_message(self, request_json: LaunchPublishMessage) -> None:
        try:
            # Convert input dictionary into a proper python object
            request = json.loads(json.dumps(request_json),
                                 object_hook=lambda d: SimpleNamespace(**d))
            Log.debug(
                f"Request to [ros.launch.publish_message]: msg [{request.msg_type}]")
            opt_str = ''
            opt_name_suf = '__latch_'
            if request.once:
                opt_str = '-1'
            elif request.latched:
                opt_str = '-l'
            elif request.rate != 0.0:
                opt_str = f"-r {request.rate}"
            if request.substitute_keywords:
                opt_str += ' -s'
            if request.verbose:
                opt_str += ' -v'
            if request.use_rostime:
                opt_str += ' --use-rostime'
            # remove empty lists
            data = json.loads(request.data)
            topic_params = self._pubstr_from_dict(data)
            pub_cmd = f"pub {request.topic_name} {request.msg_type} \"{topic_params}\" {opt_str}"
            Log.debug(f"rostopic parameter: {pub_cmd}")
            startcfg = StartConfig('rostopic', 'rostopic')
            startcfg.fullname = f"/rostopic_pub/{request.topic_name.strip('/')}"
            startcfg.args = [f"__name:={startcfg.fullname}", pub_cmd]
            launcher.run_node(startcfg)
        except Exception:
            import traceback
            print(traceback.format_exc())

    @wamp.register('ros.launch.call_service')
    def call_service(self, request_json: LaunchCallService) -> None:
        # Convert input dictionary into a proper python object
        Log.debug(
            f"Request to [ros.launch.call_service]: msg [{request_json}]")
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        result = LaunchMessageStruct(request.srv_type)
        try:
            # remove empty lists
            data = json.loads(request.data)
            service_params = self._pubstr_from_dict(data)
            call_cmd = f"rosservice call {request.service_name} \"{service_params}\""
            fullname = f"/rosservice_call{request.service_name.rstrip('/')}"
            Log.debug(f"call service with: {call_cmd}")
            ps = SupervisedPopen(shlex.split(f"{call_cmd}"), stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, object_id=f"call_service_{fullname}", description=f"Call service with {call_cmd}")
            output_err = ps.stderr.read()
            if output_err:
                result.error_msg = repr(output_err).replace('\\n', '\n')
            output = ps.stdout.read()
            if output:
                result.data = ruamel.yaml.load(
                    output, Loader=ruamel.yaml.Loader)
                result.valid = True
        except Exception as err:
            import traceback
            print(traceback.format_exc())
            result.error_msg = repr(err)
        return json.dumps(result, cls=SelfEncoder)

    def GetIncludedFiles(self, request, context):
        Log.debug('GetIncludedFiles request:\n%s' % str(request))
        try:
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
            for inc_file in xml.find_included_files(request.path, request.recursive, request.unique, search_in_ext, resolve_args):
                reply = lmsg.IncludedFilesReply()
                reply.root_path = inc_file.path_or_str
                reply.linenr = inc_file.line_number
                reply.path = inc_file.inc_path
                reply.exists = inc_file.exists
                reply.rawname = inc_file.raw_inc_path
                if reply.exists:
                    reply.size = os.path.getsize(reply.path)
                reply.rec_depth = inc_file.rec_depth
                reply.include_args.extend(lmsg.Argument(name=name, value=value)
                                          for name, value in inc_file.args.items())
                # return each file one by one
                yield reply
        except Exception:
            Log.warn("Can't get include files for %s: %s" %
                     (request.path, traceback.format_exc()))

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
                reply.status.msg = utf8(err)
                result.append(reply)
        else:
            reply = LaunchInterpretPathReply(
                text=text, status='ERROR', args=request.args)
            reply.status.msg = utf8('empty request')
            result.append(reply)
        return json.dumps(result, cls=SelfEncoder)

    def InterpretPath(self, request, context):
        Log.debug('InterpretPath request:\n%s' % str(request))
        for text in request.paths:
            if text:
                reply = lmsg.InterpredPath()
                try:
                    reply.path = xml.interpret_path(text)
                    reply.exists = os.path.exists(reply.path)
                    reply.status.code = OK
                except Exception as err:
                    reply.status.code = PACKAGE_NOT_FOUND
                    reply.status.error_msg = utf8(err)
                yield reply

    def GetMtime(self, request, context):
        Log.debug('GetMtime request:\n%s' % str(request))
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
            inc_files = xml.find_included_files(
                request.path, True, True, SEARCH_IN_EXT, resolve_args)
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
            Log.warn(traceback.format_exc())

    def GetChangedBinaries(self, request, context):
        Log.debug('GetChangedBinaries request:\n%s' % str(request))
        result = lmsg.MtimeNodes()
        changed = launcher.changed_binaries([node for node in request.names])
        nodes = []
        for name, mtime in changed:
            node = lmsg.MtimeNode(name=name, mtime=mtime)
            nodes.append(node)
        result.nodes.extend(nodes)
        return result

    def GetStartCfg(self, request, context):
        Log.debug('GetStartCfg request:\n%s' % str(request))
        result = lmsg.StartCfgReply(name=request.name)
        launch_configs = []
        if request.opt_launch:
            cfgid = CfgId(request.opt_launch, request.masteruri)
            if cfgid in self._loaded_files:
                launch_configs.append(self._loaded_files[cfgid])
        if not launch_configs:
            # get launch configurations with given node
            launch_configs = []
            for cfgid, launchcfg in self._loaded_files.items():
                if cfgid.equal_masteruri(request.masteruri):
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
        startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri,
                                                loglevel=request.loglevel, logformat=request.logformat, reload_global_param=request.reload_global_param)
        startcfg.fill_msg(result.startcfg)
        result.status.code = OK
        return result

    def ResetPackageCache(self, request, context):
        Log.debug('ResetPackageCache request:\n%s' % str(request))
        result = lmsg.Empty()
        ros_pkg.reset_cache()
        return result

    @wamp.register('ros.subscriber.start')
    def start_subscriber(self, request_json: SubscriberNode) -> bool:
        # Covert input dictionary into a proper python object
        request = json.loads(json.dumps(request_json),
                             object_hook=lambda d: SimpleNamespace(**d))
        topic = request.topic
        Log.debug(
            'Request to [ros.subscriber.start]: %s' % str(topic))
        startcfg = StartConfig('fkie_node_manager_daemon',
                               'node_manager_subscriber')
        startcfg.fullname = f"/_node_manager_subscriber/{topic.strip('/')}"
        startcfg.args = [f"__name:={startcfg.fullname}"]
        startcfg.args.append(f'--crossbar_port={self.port}')
        startcfg.args.append(f'--crossbar_realm={self.realm}')
        startcfg.args.append(f'--topic={topic}')
        startcfg.args.append(f'--message_type={request.message_type}')
        if request.no_data:
            startcfg.args.append('--no_data')
        if request.no_arr:
            startcfg.args.append('--no_arr')
        if request.no_str:
            startcfg.args.append('--nostr')
        startcfg.args.append(f'--hz={request.hz}')
        startcfg.args.append(f'--window={request.window}')
        if request.no_str:
            startcfg.args.append('--no_str')
        if request.tcp_no_delay:
            startcfg.args.append('--tcp_no_delay')
        launcher.run_node(startcfg)
        return True

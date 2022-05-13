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
import roslib.names
import roslib.packages
import rospkg
import threading
import traceback

from fkie_master_discovery.common import masteruri_from_master
import fkie_multimaster_msgs.grpc.launch_pb2_grpc as lgrpc
import fkie_multimaster_msgs.grpc.launch_pb2 as lmsg

from . import exceptions
from . import launcher
from . import url
from .common import INCLUDE_PATTERN, SEARCH_IN_EXT, find_included_files, interpret_path, utf8, reset_package_cache
from .launch_config import LaunchConfig
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

    def __init__(self, path, masteruri=''):
        '''
        :param str path: absolute path of the launch file.
        :param str masteruri: ROS-Masteruri if empty the masteruri will be determine by :meth:`fkie_master_discovery.common.masteruri_from_master`
        '''
        self.path = path
        self.masteruri = masteruri
        self._local = False
        if not masteruri:
            self.masteruri = masteruri_from_master(True)
            self._local = True
        elif url.equal_uri(self.masteruri, masteruri_from_master(True)):
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
        return not(self == other)

    def equal_masteruri(self, masteruri):
        '''
        Compares the ROS-Masteruri of this instance with other ROS-Masteruri.

        :param str masteruri: ROS Masteruri
        '''
        if not masteruri:
            if self._local:
                return True
        if url.equal_uri(self.masteruri, masteruri):
            return True
        return False


class LaunchServicer(lgrpc.LaunchServiceServicer):
    '''
    Handles GRPC-requests defined in `launch.proto`.
    '''

    def __init__(self, monitor_servicer):
        rospy.loginfo("Create launch manger servicer")
        lgrpc.LaunchServiceServicer.__init__(self)
        self._is_running = True
        self._peers = {}
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)
        self._monitor_servicer = monitor_servicer

    def _terminated(self):
        rospy.loginfo("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            rospy.loginfo("Add callback to peer context @%s" % context.peer())
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
        launch_config = LaunchConfig(path, monitor_servicer=self._monitor_servicer)
        loaded, res_argv = launch_config.load([])
        if loaded:
            rospy.logdebug("loaded %s\n  used args: %s" % (path, utf8(res_argv)))
            self._loaded_files[CfgId(path, '')] = launch_config
            if autostart:
                start_thread = threading.Thread(target=self._autostart_nodes_threaded, args=(launch_config,))
                start_thread.start()
        else:
            rospy.logwarn("load %s failed!" % (path))

    def start_node(self, node_name):
        global IS_RUNNING
        if not IS_RUNNING:
            return
        for _cfgid, launchcfg in self._loaded_files.items():
            n = launchcfg.get_node(node_name)
            if n is not None:
                startcfg = launcher.create_start_config(node_name, launchcfg, '', masteruri='', loglevel='', reload_global_param=False)
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
                    rospy.logdebug("%s is in exclude list, skip autostart", node_fullname)
                    continue
                self._autostart_node(node_fullname, cfg)
            except Exception as err:
                rospy.logwarn("Error while start %s: %s", node_fullname, err)

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
                start_timer = threading.Timer(3.0, self._autostart_node, args=(node_name, cfg))
                start_timer.start()
        else:
            start_now = True
        if start_now:
            startcfg = launcher.create_start_config(node_name, cfg, '', masteruri='', loglevel='', reload_global_param=False)
            start_delay = self._get_start_delay(cfg, node_name)
            if start_delay > 0:
                # start timer for delayed start
                start_timer = threading.Timer(start_delay, launcher.run_node, args=(startcfg,))
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
                    rospy.logwarn('Private for autostart required topic `%s` is ignored!' % topic)
                    topic = ''
                elif not rosgraph.names.is_global(topic):
                    topic = rospy.names.ns_join(rosgraph.names.namespace(node), topic)
        except Exception:
            pass
        return topic

    def GetLoadedFiles(self, request, context):
        rospy.logdebug('GetLoadedFiles request:\n%s' % str(request))
        # self._register_callback(context)
        for _cfgid, lf in self._loaded_files.items():
            reply = lmsg.LoadedFile(package=lf.packagename, launch=lf.launchname, path=lf.filename, masteruri=lf.masteruri, host=lf.host)
            reply.args.extend(lmsg.Argument(name=name, value=value) for name, value in lf.resolve_dict.items())
            yield reply

    def LoadLaunch(self, request, context):
        '''
        Loads launch file
        '''
        result = lmsg.LoadLaunchReply()
        launchfile = request.path
        rospy.logdebug("Loading launch file: %s (package: %s, launch: %s), masteruri: %s, host: %s, args: %s" % (launchfile, request.package, request.launch, request.masteruri, request.host, request.args))
        if not launchfile:
            # determine path from package name and launch name
            try:
                paths = roslib.packages.find_resource(request.package, request.launch)
                if not paths:
                    result.status.code = FILE_NOT_FOUND
                    result.status.error_msg = utf8("Launch files %s in package %s found!" % (request.launch, request.package))
                    return result
                elif len(paths) > 1:
                    if request.force_first_file:
                        launchfile = paths[0]
                    else:
                        result.status.code = MULTIPLE_LAUNCHES
                        result.status.error_msg = utf8("Multiple launch files with name %s in package %s found!" % (request.launch, request.package))
                        for mp in paths:
                            result.path.append(mp)
                        rospy.logdebug("..load aborted, MULTIPLE_LAUNCHES")
                        return result
                else:
                    launchfile = paths[0]
            except rospkg.ResourceNotFound as rnf:
                    result.status.code = FILE_NOT_FOUND
                    result.status.error_msg = utf8("Package %s not found: %s" % (request.package, rnf))
                    rospy.logdebug("..load aborted, FILE_NOT_FOUND")
                    return result
        result.path.append(launchfile)
        # it is already loaded?
        if (launchfile, request.masteruri) in list(self._loaded_files.keys()):
            result.status.code = ALREADY_OPEN
            result.status.error_msg = utf8("Launch file %s already loaded!" % (launchfile))
            rospy.logdebug("..load aborted, ALREADY_OPEN")
            return result
        # load launch configuration
        try:
            # test for required args
            provided_args = ["%s" % arg.name for arg in request.args]
            launch_config = LaunchConfig(launchfile, masteruri=request.masteruri, host=request.host, monitor_servicer=self._monitor_servicer)
            # get the list with needed launch args
            req_args = launch_config.get_args()
            req_args_dict = launch_config.argv2dict(req_args)
            if request.request_args and req_args:
                for arg, value in req_args_dict.items():
                    if arg not in provided_args:
                        result.args.extend([lmsg.Argument(name=arg, value=value) for arg, value in req_args_dict.items()])
                        result.status.code = PARAMS_REQUIRED
                        rospy.logdebug("..load aborted, PARAMS_REQUIRED")
                        return result
            argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args if arg.name in req_args_dict]
            _loaded, _res_argv = launch_config.load(argv)
            # parse result args for reply
            result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.resolve_dict.items()])
            self._loaded_files[CfgId(launchfile, request.masteruri)] = launch_config
            rospy.logdebug("..load complete!")
        except Exception as e:
            err_text = "%s loading failed!" % launchfile
            err_details = "%s: %s" % (err_text, utf8(e))
            rospy.logwarn("Loading launch file: %s", err_details)
            result.status.code = ERROR
            result.status.error_msg = utf8(err_details)
            return result
        result.status.code = OK
        return result

    def ReloadLaunch(self, request, context):
        rospy.logdebug('ReloadLaunch request:\n%s' % str(request))
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        rospy.logdebug("reload launch file: %s, masteruri: %s", request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                cfg = self._loaded_files[cfgid]
                stored_roscfg = cfg.roscfg
                argv = cfg.argv
                cfg.load(argv)
                result.status.code = OK
                # detect files changes
                if stored_roscfg and cfg.roscfg:
                    stored_values = [(name, utf8(p.value)) for name, p in stored_roscfg.params.items()]
                    new_values = [(name, utf8(p.value)) for name, p in cfg.roscfg.params.items()]
                    # detect changes parameter
                    paramset = set(name for name, _ in (set(new_values) - set(stored_values)))  # _:=value
                    # detect new parameter
                    paramset |= (set(cfg.roscfg.params.keys()) - set(stored_roscfg.params.keys()))
                    # detect removed parameter
                    paramset |= (set(stored_roscfg.params.keys()) - set(cfg.roscfg.params.keys()))
                    # detect new nodes
                    stored_nodes = [roslib.names.ns_join(item.namespace, item.name) for item in stored_roscfg.nodes]
                    new_nodes = [roslib.names.ns_join(item.namespace, item.name) for item in cfg.roscfg.nodes]
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
                                    nodes2start.add(roslib.names.ns_join(n.namespace, n.name))
                    # filter out anonymous nodes
                    for n in nodes2start:
                        if not re.search(r"\d{3,6}_\d{10,}", n):
                            result.changed_nodes.append(n)
#                    result.changed_nodes.extend([n for n in nodes2start if not re.search(r"\d{3,6}_\d{10,}", n)])
            except Exception as e:
                print(traceback.format_exc())
                err_text = "%s loading failed!" % request.path
                err_details = "%s: %s" % (err_text, utf8(e))
                rospy.logwarn("Loading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def UnloadLaunch(self, request, context):
        rospy.logdebug('UnloadLaunch request:\n%s' % str(request))
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                del self._loaded_files[cfgid]
                result.status.code = OK
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s: %s" % (err_text, utf8(e))
                rospy.logwarn("Unloading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def GetNodes(self, request, context):
        rospy.logdebug('GetNodes request:\n%s' % str(request))
        requested_files = []
        lfiles = request.launch_files
        for lfile in lfiles:
            requested_files.append(CfgId(lfile, request.masteruri))
        if not requested_files:
            requested_files = list(self._loaded_files.keys())
        for cfgid in requested_files:
            lc = self._loaded_files[cfgid]
            reply = lmsg.LaunchContent(launch_file=cfgid.path, masteruri=lc.masteruri, host=lc.host)
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
                        nodelet_mngr = roslib.names.ns_join(n.namespace, args[2])
                        if nodelet_mngr not in nodelets:
                            nodelets[nodelet_mngr] = []
                        nodelets[nodelet_mngr].append(roslib.names.ns_join(n.namespace, n.name))
            for mngr, ndl in nodelets.items():
                nlmsg = lmsg.Nodelets(manager=mngr)
                nlmsg.nodes.extend(ndl)
                reply.nodelets.extend([nlmsg])
            # create association description
            associations = {}
            for n in lc.roscfg.nodes:
                node_fullname = roslib.names.ns_join(n.namespace, n.name)
                associations_param = roslib.names.ns_join(node_fullname, 'nm/associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(item.namespace, split))
                    associations[node_fullname] = values
                # DEPRECATED 'associations'
                associations_param = roslib.names.ns_join(node_fullname, 'associations')
                if associations_param in lc.roscfg.params:
                    line = lc.roscfg.params[associations_param].value
                    splits = re.split(r'[;,\s]\s*', line)
                    values = []
                    for split in splits:
                        values.append(roslib.names.ns_join(item.namespace, split))
                    associations[node_fullname] = values
            for node, ass in associations.items():
                assmsg = lmsg.Associations(node=node)
                assmsg.nodes.extend(ass)
                reply.associations.extend([assmsg])
            yield reply

    def StartNode(self, request_iterator, context):
        for request in request_iterator:
            rospy.logdebug('StartNode request:\n%s' % str(request))
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
                    result.launch.extend([lcfg.filename for lcfg in launch_configs])
                    yield result
                try:
                    result.launch.append(launch_configs[0].filename)
                    startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri, loglevel=request.loglevel, logformat=request.logformat, reload_global_param=request.reload_global_param, cmd_prefix=request.cmd_prefix)
                    launcher.run_node(startcfg)
                    result.status.code = OK
                    yield result
                except exceptions.BinarySelectionRequest as bsr:
                    result.status.code = MULTIPLE_BINARIES
                    result.status.error_msg = "multiple binaries found for node '%s': %s" % (request.name, bsr.choices)
                    result.path.extend(bsr.choices)
                    yield result
                except grpc.RpcError as conerr:
                    result.status.code = CONNECTION_ERROR
                    result.status.error_msg = utf8(conerr)
                    yield result
            except exceptions.ResourceNotFound as err_nf:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(err_nf))
                yield result
            except Exception as _errr:
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(traceback.format_exc()))
                yield result

    def StartStandaloneNode(self, request, context):
        rospy.logdebug('StartStandaloneNode request:\n%s' % str(request))
        result = lmsg.StartNodeReply(name=request.name)
        try:
            startcfg = StartConfig.from_msg(request)
            try:
                launcher.run_node(startcfg)
                result.status.code = OK
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = MULTIPLE_BINARIES
                result.status.error_msg = "multiple binaries found for node '%s': %s" % (request.name, bsr.choices)
                result.launch.extend(bsr.choices)
        except grpc.RpcError as conerr:
            result.status.code = CONNECTION_ERROR
            result.status.error_msg = utf8(conerr)
        except Exception:
            result = lmsg.StartNodeReply(name=request.name)
            result.status.code = ERROR
            result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(traceback.format_exc()))
        return result

    def GetIncludedFiles(self, request, context):
        rospy.logdebug('GetIncludedFiles request:\n%s' % str(request))
        try:
            pattern = INCLUDE_PATTERN
            if request.pattern:
                pattern = request.pattern
            search_in_ext = SEARCH_IN_EXT
            if request.search_in_ext:
                search_in_ext = request.search_in_ext
            # search for loaded file and get the arguments
            resolve_args = {arg.name: arg.value for arg in request.include_args}
            if not resolve_args:
                for cfgid, lcfg in self._loaded_files.items():
                    if cfgid.path == request.path:
                        resolve_args.update(lcfg.resolve_dict)
                        break
            # replay each file
            for inc_file in find_included_files(request.path, request.recursive, request.unique, pattern, search_in_ext, resolve_args):
                reply = lmsg.IncludedFilesReply()
                reply.root_path = inc_file.path_or_str
                reply.linenr = inc_file.line_number
                reply.path = inc_file.inc_path
                reply.exists = inc_file.exists
                reply.rawname = inc_file.raw_inc_path
                if reply.exists:
                    reply.size = os.path.getsize(reply.path)
                reply.rec_depth = inc_file.rec_depth
                reply.include_args.extend(lmsg.Argument(name=name, value=value) for name, value in inc_file.args.items())
                # return each file one by one
                yield reply
        except Exception:
            rospy.logwarn("Can't get include files for %s: %s" % (request.path, traceback.format_exc()))

    def InterpretPath(self, request, context):
        rospy.logdebug('InterpretPath request:\n%s' % str(request))
        for text in request.paths:
            if text:
                reply = lmsg.InterpredPath()
                try:
                    reply.path = interpret_path(text)
                    reply.exists = os.path.exists(reply.path)
                    reply.status.code = OK
                except Exception as err:
                    reply.status.code = PACKAGE_NOT_FOUND
                    reply.status.error_msg = utf8(err)
                yield reply

    def GetMtime(self, request, context):
        rospy.logdebug('GetMtime request:\n%s' % str(request))
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
            inc_files = find_included_files(request.path, True, True, INCLUDE_PATTERN, SEARCH_IN_EXT, resolve_args)
            for inc_file in inc_files:
                incf = inc_file.inc_path
                if incf not in already_in:
                    mtime = 0
                    if os.path.exists(incf):
                        mtime = os.path.getmtime(incf)
                    result.included_files.extend([lmsg.FileObj(path=incf, mtime=mtime)])
                    already_in.append(incf)
            return result
        except Exception:
            rospy.logwarn(traceback.format_exc())

    def GetChangedBinaries(self, request, context):
        rospy.logdebug('GetChangedBinaries request:\n%s' % str(request))
        result = lmsg.MtimeNodes()
        changed = launcher.changed_binaries([node for node in request.names])
        nodes = []
        for name, mtime in changed:
            node = lmsg.MtimeNode(name=name, mtime=mtime)
            nodes.append(node)
        result.nodes.extend(nodes)
        return result

    def GetStartCfg(self, request, context):
        rospy.logdebug('GetStartCfg request:\n%s' % str(request))
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
        startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri, loglevel=request.loglevel, logformat=request.logformat, reload_global_param=request.reload_global_param)
        startcfg.fill_msg(result.startcfg)
        result.status.code = OK
        return result

    def ResetPackageCache(self, request, context):
        rospy.logdebug('ResetPackageCache request:\n%s' % str(request))
        result = lmsg.Empty()
        reset_package_cache()
        return result

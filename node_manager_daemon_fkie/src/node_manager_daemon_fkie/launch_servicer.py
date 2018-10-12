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

from master_discovery_fkie.common import masteruri_from_master
import node_manager_daemon_fkie.generated.launch_pb2_grpc as lgrpc
import node_manager_daemon_fkie.generated.launch_pb2 as lmsg

import url
from .common import INCLUDE_PATTERN, included_files, utf8
from .launch_config import LaunchConfig
from .startcfg import StartConfig
import exceptions
import launcher
from docutils.parsers.rst.directives import path

OK = lmsg.ReturnStatus.StatusType.Value('OK')
ERROR = lmsg.ReturnStatus.StatusType.Value('ERROR')
ALREADY_OPEN = lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN')
MULTIPLE_BINARIES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_BINARIES')
MULTIPLE_LAUNCHES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES')
PARAMS_REQUIRED = lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED')
FILE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND')
NODE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('NODE_NOT_FOUND')

IS_RUNNING = True


class CfgId(object):

    def __init__(self, path, masteruri=''):
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
        if not masteruri:
            if self._local:
                return True
        if url.equal_uri(self.masteruri, masteruri):
            return True
        return False


class LaunchServicer(lgrpc.LaunchServiceServicer):

    def __init__(self):
        rospy.loginfo("Create launch manger servicer")
        lgrpc.LaunchServiceServicer.__init__(self)
        self._is_running = True
        self._peers = {}
        self._loaded_files = dict()  # dictionary of (CfgId: LaunchConfig)

    def _terminated(self):
        rospy.loginfo("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            rospy.loginfo("Add callback to peer context @%s" % context.peer())
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def stop(self):
        global IS_RUNNING
        IS_RUNNING = False

    def load_launch_file(self, path, autostart=False):
        launch_config = LaunchConfig(path)
        loaded, res_argv = launch_config.load([])
        if loaded:
            rospy.logdebug("loaded %s\n  used args: %s" % (path, utf8(res_argv)))
            self._loaded_files[CfgId(path, '')] = launch_config
            if autostart:
                start_thread = threading.Thread(target=self._autostart_nodes_threaded, args=(launch_config,))
                start_thread.start()
        else:
            rospy.logwarn("load %s failed!" % (path))

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
                startcfg = launcher.create_start_config(node_fullname, cfg, '', masteruri='', loglevel='', reload_global_param=False)
                start_delay = self._get_start_delay(cfg, node_fullname)
                if start_delay > 0:
                    # start timer for delayed start
                    start_timer = threading.Timer(start_delay, launcher.run_node, args=(startcfg,))
                    start_timer.start()
                else:
                    launcher.run_node(startcfg)
            except Exception as err:
                rospy.logwarn("Error while start %s: %s", node_fullname, err)

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

    def GetLoadedFiles(self, request, context):
        # self._register_callback(context)
        for _cfgid, lf in self._loaded_files.iteritems():
            reply = lmsg.LoadedFile(package=lf.packagename, launch=lf.launchname, path=lf.filename, masteruri=lf.masteruri, host=lf.host)
            reply.args.extend(lmsg.Argument(name=name, value=value) for name, value in lf.argv2dict(lf.argv).items())
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
        if (launchfile, request.masteruri) in self._loaded_files.keys():
            result.status.code = ALREADY_OPEN
            result.status.error_msg = utf8("Launch file %s already loaded!" % (launchfile))
            rospy.logdebug("..load aborted, ALREADY_OPEN")
            return result
        # load launch configuration
        try:
            # test for required args
            provided_args = ["%s" % arg.name for arg in request.args]
            launch_config = LaunchConfig(launchfile, masteruri=request.masteruri, host=request.host)
            if request.request_args:
                # get the list with needed launch args
                req_args = launch_config.get_args()
                if req_args:
                    arg_dict = launch_config.argv2dict(req_args)
                    for arg, value in arg_dict.items():
                        if arg not in provided_args:
                            result.args.extend([lmsg.Argument(name=arg, value=value) for arg, value in arg_dict.items()])
                            result.status.code = PARAMS_REQUIRED
                            rospy.logdebug("..load aborted, PARAMS_REQUIRED")
                            return result
            argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args]
            _loaded, res_argv = launch_config.load(argv)
            # parse result args for reply
            result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.argv2dict(res_argv).items()])
            self._loaded_files[CfgId(launchfile, request.masteruri)] = launch_config
            rospy.logdebug("..load complete!")
        except Exception as e:
            err_text = "%s loading failed!" % os.path.basename(launchfile)
            err_details = "%s\n\n%s: %s" % (err_text, e.__class__.__name__, utf8(e))
            rospy.logwarn("Loading launch file: %s", err_details)
            result.status.code = ERROR
            result.status.error_msg = utf8(err_details)
            return result
        result.status.code = OK
        return result

    def ReloadLaunch(self, request, context):
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
                err_text = "%s loading failed!" % request.path
                err_details = "%s\n\n%s: %s" % (err_text, e.__class__.__name__, utf8(e))
                rospy.logwarn("Loading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def UnloadLaunch(self, request, context):
        result = lmsg.LoadLaunchReply()
        result.path.append(request.path)
        cfgid = CfgId(request.path, request.masteruri)
        if cfgid in self._loaded_files:
            try:
                del self._loaded_files[cfgid]
                result.status.code = OK
            except Exception as e:
                err_text = "%s unloading failed!" % request.path
                err_details = "%s\n\n%s: %s" % (err_text, e.__class__.__name__, utf8(e))
                rospy.logwarn("Unloading launch file: %s", err_details)
                result.status.code = ERROR
                result.status.error_msg = utf8(err_details)
                return result
        else:
            result.status.code = FILE_NOT_FOUND
            return result
        return result

    def GetNodes(self, request, context):
        requested_files = []
        lfiles = request.launch_files
        for lfile in lfiles:
            requested_files.append(CfgId(lfile, request.masteruri))
        if not requested_files:
            requested_files = self._loaded_files.keys()
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
#                                    print("    >", descr_dict['nodes'])
                                    cap.nodes.extend(descr_dict['nodes'])
                                    caps.append(cap)
                        rd.capabilities.extend(caps)
                    reply.description.extend(rd_hosts)
                except Exception:
                    import traceback
                    print traceback.format_exc()
            yield reply

    def StartNode(self, request_iterator, context):
        for request in request_iterator:
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
                    startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binary, masteruri=request.masteruri, loglevel=request.loglevel, reload_global_param=request.reload_global_param)
                    startcfg.host = url.get_nmd_url(request.masteruri)
                    launcher.run_node(startcfg)
                    result.status.code = OK
                    yield result
                except exceptions.BinarySelectionRequest as bsr:
                    result.status.code = MULTIPLE_BINARIES
                    result.status.error_msg = "multiple binaries found for node '%s'" % request.name
                    result.launch.extend(bsr.choices)
                    yield result
            except Exception:
                import traceback
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(traceback.format_exc()))
                yield result

    def StartStandaloneNode(self, request, context):
        try:
            result = lmsg.StartNodeReply(name=request.name)
            startcfg = StartConfig(request.package, request.binary)
            startcfg.binary_path = request.binary_path
            startcfg.name = request.name
            startcfg.namespace = request.namespace
            startcfg.fullname = request.fullname
            startcfg.prefix = request.prefix
            startcfg.cwd = request.cwd
            startcfg.env = {env.name: env.value for env in request.env}
            startcfg.remaps = {remap.from_name: remap.to_name for remap in request.remaps}
            startcfg.params = {param.name: utf8(param.value) for param in request.params}
            startcfg.clear_params = list(request.clear_params)
            startcfg.args = list(request.args)
            startcfg.masteruri = request.masteruri
            startcfg.loglevel = request.loglevel
            startcfg.respawn = request.respawn
            startcfg.respawn_delay = request.respawn_delay
            startcfg.respawn_max = request.respawn_max
            startcfg.respawn_min_runtime = request.respawn_min_runtime
            try:
                launcher.run_node(startcfg)
                result.status.code = OK
            except exceptions.BinarySelectionRequest as bsr:
                result.status.code = MULTIPLE_BINARIES
                result.status.error_msg = "multiple binaries found for node '%s'" % request.name
                result.launch.extend(bsr.choices)
                return result
            return result
        except Exception:
            import traceback
            result = lmsg.StartNodeReply(name=request.name)
            result.status.code = ERROR
            result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(traceback.format_exc()))
            return result

    def RestartNode(self, request_iterator, context):
        # missing associated documentation comment in .proto file
        pass
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetIncludedFiles(self, request, context):
        pattern = INCLUDE_PATTERN
        if request.pattern:
            pattern = request.pattern
        inc_files = included_files(request.path, request.recursive, request.unique, pattern)
        # create a stack to reply the include tree as a list
        queue = []
        if isinstance(inc_files, set):
            queue = [(request.path, 0, path, []) for path in list(inc_files)]
        else:
            queue = [(request.path, linenr, path, file_list) for linenr, path, file_list in reversed(inc_files)]
        while queue:
            node = queue.pop()
            reply = lmsg.IncludedFilesReply()
            reply.root_path = node[0]
            reply.linenr = node[1]
            reply.path = node[2]
            reply.exists = os.path.exists(reply.path)
            yield reply
            for linenr, path, file_list in node[3]:
                queue.append(((node[2], linenr, path, file_list)))

    def GetMtime(self, request, context):
        result = lmsg.MtimeReply()
        result.path = request.path
        mtime = 0
        if os.path.exists(request.path):
            mtime = os.path.getmtime(request.path)
        result.mtime = mtime
        # add mtimes for all included files
        inc_files = included_files(request.path, True, True, INCLUDE_PATTERN)
        for incf in inc_files:
            mtime = 0
            if os.path.exists(incf):
                mtime = os.path.getmtime(incf)
            result.included_files.extend([lmsg.FileObj(path=incf, mtime=mtime)])
        return result

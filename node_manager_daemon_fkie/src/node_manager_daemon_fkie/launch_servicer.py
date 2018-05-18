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
import rospy
import roslib.names
import roslib.packages
import rospkg

import node_manager_daemon_fkie.generated.launch_pb2_grpc as lgrpc
import node_manager_daemon_fkie.generated.launch_pb2 as lmsg

from .common import INCLUDE_PATTERN, included_files, utf8
from .launch_config import LaunchConfig
from .startcfg import StartConfig
import exceptions
import launcher

OK = lmsg.ReturnStatus.StatusType.Value('OK')
ERROR = lmsg.ReturnStatus.StatusType.Value('ERROR')
ALREADY_OPEN = lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN')
MULTIPLE_BINARIES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_BINARIES')
MULTIPLE_LAUNCHES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES')
PARAMS_REQUIRED = lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED')
FILE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND')
NODE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('NODE_NOT_FOUND')


class LaunchServicer(lgrpc.LaunchServiceServicer):

    def __init__(self):
        rospy.loginfo("Create launch manger servicer")
        lgrpc.LaunchServiceServicer.__init__(self)
        self._peers = {}
        self._loaded_files = dict()  # dictionary of (full path: lmsg.LoadedFile)

    def _terminated(self):
        rospy.loginfo("terminated launch context")

    def _register_callback(self, context):
        if (context.peer() not in self._peers):
            rospy.loginfo("Add callback to peer context @%s" % context.peer())
            if context.add_callback(self._terminated):
                self._peers[context.peer()] = context

    def GetLoadedFiles(self, request, context):
        self._register_callback(context)
        for _path, lf in self._loaded_files.iteritems():
            reply = lmsg.LoadedFile(package=lf.packagename, launch=lf.launchname, path=lf.filename, host=lf.host)
            reply.args.extend(lmsg.Argument(name=name, value=value) for name, value in lf.argv2dict(lf.argv).items())
            yield reply

    def LoadLaunch(self, request, context):
        '''
        Loads launch file
        '''
        result = lmsg.LoadLaunchReply()
        launchfile = request.path
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
                        return result
                else:
                    launchfile = paths[0]
            except rospkg.ResourceNotFound as rnf:
                    result.status.code = FILE_NOT_FOUND
                    result.status.error_msg = utf8("Package %s not found: %s" % (request.package, rnf))
                    return result
        result.path.append(launchfile)
        # it is already loaded?
        if launchfile in self._loaded_files.keys():
            result.status.code = ALREADY_OPEN
            result.status.error_msg = utf8("Launch files %s from package %s already loaded!" % (request.launch, request.package))
            return result
        # load launch configuration
        try:
            # test for required args
            launch_config = LaunchConfig(launchfile, host=request.host)
            if request.request_args:
                # get the list with needed launch args
                req_args = launch_config.get_args()
                if req_args:
                    arg_dict = launch_config.argv2dict(req_args)
                    result.args.extend([lmsg.Argument(name=arg, value=value) for arg, value in arg_dict.items()])
                    result.status.code = PARAMS_REQUIRED
                    return result
            argv = ["%s:=%s" % (arg.name, arg.value) for arg in request.args]
            _loaded, res_argv = launch_config.load(argv)
            # parse result args for reply
            result.args.extend([lmsg.Argument(name=name, value=value) for name, value in launch_config.argv2dict(res_argv).items()])
            self._loaded_files[launchfile] = launch_config
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
        if request.path in self._loaded_files:
            try:
                argv = self._loaded_files[request.path].argv
                self._loaded_files[request.path].load(argv)
                result.status.code = OK
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

    def GetNodes(self, request, context):
        requested_files = request.launch_files
        if not requested_files:
            requested_files = self._loaded_files.keys()
        for path in requested_files:
            lc = self._loaded_files[path]
            reply = lmsg.LaunchContent(launch_file=path)
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
                    import traceback
                    print traceback.format_exc()
            yield reply

    def StartNode(self, request_iterator, context):
        for request in request_iterator:
            print("start", request.name)
            try:
                result = lmsg.StartNodeReply(name=request.name)
                launch_configs = []
                if request.opt_launch:
                    if request.opt_launch in self._loaded_files:
                        launch_configs.append(self._loaded_files[request.opt_launch])
                if not launch_configs:
                    # get launch configurations with given node
                    launch_configs = []
                    for launchcfg in self._loaded_files.values():
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
                    startcfg = launcher.create_start_config(request.name, launch_configs[0], request.opt_binariy, loglevel=request.loglevel)
                    launcher.run_node(startcfg)
                    result.status.code = OK
                except exceptions.BinarySelectionRequest as bsr:
                    result.status.code = MULTIPLE_BINARIES
                    result.status.error_msg = "multiple binaries found for node '%s'" % request.name
                    result.launch.extend(bsr.choices)
                    yield result
                yield result
            except Exception:
                import traceback
                result = lmsg.StartNodeReply(name=request.name)
                result.status.code = ERROR
                result.status.error_msg = "Error while start node '%s': %s" % (request.name, utf8(traceback.format_exc()))
                yield result

    def StartStandaloneNode(self, request, context):
        print("start standalone", request.name)
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
            startcfg.params = {param.name: param.value for param in request.params}
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

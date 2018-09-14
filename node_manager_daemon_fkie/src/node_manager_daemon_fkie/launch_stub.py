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

from __future__ import print_function

import exceptions
import settings
import node_manager_daemon_fkie.generated.launch_pb2_grpc as lgrpc
import node_manager_daemon_fkie.generated.launch_pb2 as lmsg
from .launch_description import LaunchDescription, RobotDescription, Capability
# from .startcfg import StartConfig

OK = lmsg.ReturnStatus.StatusType.Value('OK')
ERROR = lmsg.ReturnStatus.StatusType.Value('ERROR')
ALREADY_OPEN = lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN')
MULTIPLE_BINARIES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_BINARIES')
MULTIPLE_LAUNCHES = lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES')
PARAMS_REQUIRED = lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED')
FILE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND')
NODE_NOT_FOUND = lmsg.ReturnStatus.StatusType.Value('NODE_NOT_FOUND')


class LaunchStub(object):

    def __init__(self, channel):
        self.lm_stub = lgrpc.LaunchServiceStub(channel)

    def load_launch(self, package, launch, path='', args=None, request_args=False, masteruri='', host=''):
        '''
        '''
        arguments = args
        if arguments is None:
            arguments = {}
        request = lmsg.LoadLaunchRequest(package=package, launch=launch, path=path, request_args=request_args, masteruri=masteruri, host=host)
        request.args.extend([lmsg.Argument(name=name, value=value) for name, value in arguments.items()])
        response = self.lm_stub.LoadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code == OK:
            pass
        elif response.status.code == MULTIPLE_LAUNCHES:
            raise exceptions.LaunchSelectionRequest([path for path in response.path], response.status.error_msg)
        elif response.status.code == PARAMS_REQUIRED:
            raise exceptions.ParamSelectionRequest({arg.name: arg.value for arg in response.args}, response.status.error_msg)
        elif response.status.code == ALREADY_OPEN:
            raise exceptions.AlreadyOpenException(response.path[0], response.status.error_msg)
        else:
            raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path[0], {arg.name: arg.value for arg in response.args}

    def reload_launch(self, path):
        '''
        '''
        request = lmsg.LaunchFile(path=path)
        response = self.lm_stub.ReloadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code != OK:
            if response.status.code == FILE_NOT_FOUND:
                raise exceptions.ResourceNotFound(path, response.status.error_msg)
            else:
                raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path, response.changed_nodes

    def unload_launch(self, path):
        '''
        '''
        request = lmsg.LaunchFile(path=path)
        response = self.lm_stub.UnloadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code != OK:
            if response.status.code == FILE_NOT_FOUND:
                raise exceptions.ResourceNotFound(path, response.status.error_msg)
            else:
                raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path

    def get_nodes(self, request_description=False):
        result = []
        request = lmsg.ListNodesRequest(request_description=request_description)
        response_stream = self.lm_stub.GetNodes(request, timeout=settings.GRPC_TIMEOUT)
        for response in response_stream:
            descriptions = []
            for descr in response.description:
                rd = RobotDescription(machine=descr.machine, robot_name=descr.robot_name, robot_type=descr.robot_type, robot_images=list(descr.robot_images), robot_descr=descr.robot_descr)
                for cap in descr.capabilities:
                    cp = Capability(name=cap.name, namespace=cap.namespace, cap_type=cap.type, images=[img for img in cap.images], description=cap.description, nodes=[n for n in cap.nodes])
                    rd.capabilities.append(cp)
                descriptions.append(rd)
            ld = LaunchDescription(response.launch_file, response.masteruri, response.host, list(response.node), descriptions)
            result.append(ld)
        return result

    def _get_included_files(self, path, recursive=True, unique=False, include_pattern=[]):
        '''
        :return: Returns the result from grpc server
        :rtype: stream lmsg.IncludedFilesReply
        '''
        request = lmsg.IncludedFilesRequest(path=path, recursive=recursive, unique=unique, pattern=include_pattern)
        return self.lm_stub.GetIncludedFiles(request, timeout=settings.GRPC_TIMEOUT)

    def get_included_files_unique(self, root, recursive=True, include_pattern=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of included files.
        :rtype: [str]
        '''
        reply = self._get_included_files(path=root, recursive=recursive, unique=True, include_pattern=include_pattern)
        result = set()
        for response in reply:
            result.add(response.path)
        return list(result)

    def get_included_files(self, root, recursive=True, include_pattern=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of tuples with line number, path of included file, file exists or not and a recursive list of tuples with included files.
        :rtype: [(int, str, bool, [])]
        '''
        reply = self._get_included_files(path=root, recursive=recursive, unique=False, include_pattern=include_pattern)
        result = []
        queue = [(root, result)]
        current_root = root
        last_root = ''
        for response in reply:
            if current_root == response.root_path:
                last_root = response.path
                queue[-1][1].append((response.linenr, response.path, response.exists, []))
            elif last_root == response.root_path:
                queue.append((last_root, queue[-1][1][-1][3]))
                current_root = last_root
                last_root = response.path
                queue[-1][1].append((response.linenr, response.path, response.exists, []))
            else:
                last_item = queue.pop()
                while last_item[0] != response.root_path and queue:
                    last_item = queue.pop()
                if last_item[0] == response.root_path:
                    current_root = response.root_path
                    queue.append(last_item)
                    last_root = response.path
                    queue[-1][1].append((response.linenr, response.path, response.exists, []))
                else:
                    raise Exception("wrong root item: %s" % response.root_path)
        return result

    def get_included_path(self, text, include_pattern=[]):
        '''
        :param str text: text to search for included files
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of tuples with line number, path of included file, file exists or not and a recursive list of tuples with included files.
        :rtype: [(int, str, bool, [])]
        '''
        reply = self._get_included_files(path=text, recursive=False, unique=False, include_pattern=include_pattern)
        result = []
        for response in reply:
            result.append((response.linenr, response.path, response.exists, []))
        return result

    def _gen_node_list(self, nodes):
        for name, opt_binariy, opt_launch, loglevel in nodes:
            yield lmsg.Node(name=name, opt_binariy=opt_binariy, opt_launch=opt_launch, loglevel=loglevel)

    def start_node(self, name, opt_binariy='', opt_launch='', loglevel=0):
        '''
        Start node.
        :param str name: full name of the ros node exists in the launch file.
        :param str opt_binariy: the full path of the binary. Used in case of multiple binaries in the same package.
        :param str opt_launch: full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
        :param str loglevel: log level
        :raise exceptions.StartException: on start errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        :raise exceptions.LaunchSelectionRequest: on multiple launch files
        '''
        response_stream = self.lm_stub.StartNode(self._gen_node_list([(name, opt_binariy, opt_launch, loglevel)]), timeout=settings.GRPC_TIMEOUT)
        for response in response_stream:
            if response.status.code == 0:
                pass
            elif response.status.code == ERROR:
                raise exceptions.StartException(response.status.error_msg)
            elif response.status.code == NODE_NOT_FOUND:
                raise exceptions.StartException(response.status.error_msg)
            elif response.status.code == MULTIPLE_BINARIES:
                raise exceptions.BinarySelectionRequest(response.path, response.status.error_msg)
            elif response.status.code == MULTIPLE_LAUNCHES:
                raise exceptions.LaunchSelectionRequest(response.launch, response.status.error_msg)

    def start_standalone_node(self, startcfg):
        '''
        Start a node on remote launch manager using a ``StartConfig``
        :param startcfg: start configuration
        :type startcfg: node_manager_daemon_fkie.startcfg.StartConfig
        :raise exceptions.StartException: on errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        '''
        request = lmsg.StartConfig(package=startcfg.package, binary=startcfg.binary)
        if startcfg.binary_path:
            request.binary_path = startcfg.binary_path
        if startcfg.name:
            request.name = startcfg.name
        if startcfg.namespace:
            request.namespace = startcfg.namespace
        if startcfg.fullname:
            request.fullname = startcfg.fullname
        if startcfg.prefix:
            request.prefix = startcfg.prefix
        if startcfg.cwd:
            request.cwd = startcfg.cwd
        if startcfg.env:
            request.env.extend([lmsg.Argument(name=name, value=value) for name, value in startcfg.env.items()])
        if startcfg.remaps:
            request.remaps.extend([lmsg.Remapping(from_name=name, to_name=value) for name, value in startcfg.remaps.items()])
        if startcfg.params:
            request.params.extend([lmsg.Argument(name=name, value=value) for name, value in startcfg.params.items()])
        if startcfg.clear_params:
            request.clear_params.extend(startcfg.clear_params)
        if startcfg.args:
            request.args.extend(startcfg.args)
        if startcfg.masteruri:
            request.masteruri = startcfg.masteruri
        request.loglevel = startcfg.loglevel
        request.respawn = startcfg.respawn
        request.respawn_delay = startcfg.respawn_delay
        request.respawn_max = startcfg.respawn_max
        request.respawn_min_runtime = startcfg.respawn_min_runtime
        response = self.lm_stub.StartStandaloneNode(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code == 0:
            pass
        elif response.status.code == MULTIPLE_BINARIES:
            raise exceptions.BinarySelectionRequest(response.path, 'Multiple executables')
        elif response.status.code == FILE_NOT_FOUND:
            raise exceptions.StartException("Can't find %s in %s" % (startcfg.binary, startcfg.package))
        elif response.status.code == ERROR:
            raise exceptions.StartException(response.status.error_msg)

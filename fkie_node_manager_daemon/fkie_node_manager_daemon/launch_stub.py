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


import fkie_node_manager_daemon.grpc_proto.launch_pb2_grpc as lgrpc
import fkie_node_manager_daemon.grpc_proto.launch_pb2 as lmsg
from . import exceptions
from . import settings
from .common import IncludedFile
from .launch_description import Capability
from .launch_description import LaunchDescription
from .launch_description import NodeDescription
from .launch_description import RobotDescription
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


class LaunchStub(object):

    def __init__(self, channel):
        self.lm_stub = lgrpc.LaunchServiceStub(channel)

    def load_launch(self, package, launch, *, path='', args=None, request_args=False, daemonuri=''):
        '''
        :param str daemonuri: daemon where the node of this launch file should be started. Empty for local.
        :return: tuple with launch file, dictionary of requested arguments
        :rtype: str, {str: str}
        '''
        arguments = args
        if arguments is None:
            arguments = {}
        request = lmsg.LoadLaunchRequest(package=package, launch=launch, path=path, request_args=request_args, daemonuri=daemonuri)
        request.args.extend([lmsg.Argument(name=name, value=value) for name, value in arguments.items()])
        response = self.lm_stub.LoadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code == OK:
            pass
        elif response.status.code == MULTIPLE_LAUNCHES:
            raise exceptions.LaunchSelectionRequest([_path for _path in response.path], response.status.error_msg)
        elif response.status.code == PARAMS_REQUIRED:
            raise exceptions.ParamSelectionRequest({arg.name: arg for arg in response.args}, response.status.error_msg)
        elif response.status.code == ALREADY_OPEN:
            raise exceptions.AlreadyOpenException(response.path[0], response.status.error_msg)
        else:
            raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path[0], {arg.name: arg.value for arg in response.args}

    def reload_launch(self, path, daemonuri=''):
        '''
        :return: tuple with launch file, list of changed node after reload launch
        :rtype: str, [str]
        '''
        request = lmsg.LaunchFile(path=path, daemonuri=daemonuri)
        response = self.lm_stub.ReloadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code != OK:
            if response.status.code == FILE_NOT_FOUND:
                raise exceptions.ResourceNotFound(path, response.status.error_msg)
            else:
                raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path, response.changed_nodes

    def get_loaded_files(self):
        request = lmsg.Empty()
        response = self.lm_stub.GetLoadedFiles(request, timeout=settings.GRPC_TIMEOUT)
        result = []
        for loaded_file in response:
            args = {arg.name: arg.value for arg in loaded_file.args}
            result.append((loaded_file.package, loaded_file.path, args, loaded_file.daemonuri))
        return result

    def get_mtimes(self, path):
        '''
        :return: tuple with launch file, modification time and dictionary of included files and their modification times.
        :rtype: str, double {str: double}
        '''
        request = lmsg.LaunchFile(path=path, daemonuri='')
        response = self.lm_stub.GetMtime(request, timeout=settings.GRPC_TIMEOUT)
        return response.path, response.mtime, response.included_files

    def get_changed_binaries(self, nodes):
        '''
        :param nodes: list with ROS-node names to check for changed binaries.
        :type nodes: list(str)
        :return: Dictionary with ROS-node names of changed binaries since last start and their last modification time.
        :rtype: dict(str: float)
        '''
        request = lmsg.Nodes()
        request.names.extend(nodes)
        response = self.lm_stub.GetChangedBinaries(request, timeout=settings.GRPC_TIMEOUT)
        return {node.name: node.mtime for node in response.nodes}

    def unload_launch(self, path, daemonuri=''):
        '''
        '''
        request = lmsg.LaunchFile(path=path, daemonuri=daemonuri)
        response = self.lm_stub.UnloadLaunch(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code != OK:
            if response.status.code == FILE_NOT_FOUND:
                raise exceptions.ResourceNotFound(path, response.status.error_msg)
            else:
                raise exceptions.RemoteException(response.status.code, response.status.error_msg)
        return response.path[0]

    def get_nodes(self, request_description=False, daemonuri=''):
        result = []
        request = lmsg.ListNodesRequest(request_description=request_description, daemonuri=daemonuri)
        response_stream = self.lm_stub.GetNodes(request, timeout=settings.GRPC_TIMEOUT)
        for response in response_stream:
            descriptions = []
            for descr in response.description:
                rd = RobotDescription(machine=descr.machine, robot_name=descr.robot_name, robot_type=descr.robot_type, robot_images=list(descr.robot_images), robot_descr=descr.robot_descr)
                for cap in descr.capabilities:
                    cp = Capability(name=cap.name, namespace=cap.namespace, cap_type=cap.type, images=[img for img in cap.images], description=cap.description, nodes=[n for n in cap.nodes])
                    rd.capabilities.append(cp)
                descriptions.append(rd)
            nodes = [NodeDescription(n.name, is_executable=n.is_executable, composable_container=n.composable_container, composable_nodes=n.composable_nodes) for n in response.nodes]
            ld = LaunchDescription(response.launch_file, response.daemonuri, nodes=nodes, robot_descriptions=descriptions)
            result.append(ld)
        return result

    def _get_included_files(self, path, *, recursive=True, unique=False, include_args={}, include_pattern=[], search_in_ext=[]):
        '''
        :return: Returns the result from grpc server
        :rtype: stream lmsg.IncludedFilesReply
        '''
        request = lmsg.IncludedFilesRequest(path=path, recursive=recursive, unique=unique, pattern=include_pattern, search_in_ext=search_in_ext)
        request.include_args.extend(lmsg.Argument(name=name, value=value) for name, value in include_args.items())
        return self.lm_stub.GetIncludedFiles(request, timeout=settings.GRPC_TIMEOUT)

    def get_included_files_set(self, root, *, recursive=True, include_args={}, include_pattern=[], search_in_ext=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_args: dictionary with arguments to override while include.
        :type include_args: {str: str}
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :param search_in_ext: file extensions to search in
        :type search_in_ext: [str]
        :return: Returns a list of included files.
        :rtype: list(str)
        '''
        reply = self._get_included_files(path=root, recursive=recursive, unique=True, include_args=include_args, include_pattern=include_pattern, search_in_ext=search_in_ext)
        result = set()
        for response in reply:
            result.add(response.path)
        return list(result)

    def get_included_files(self, root, recursive=True, include_args={}, include_pattern=[], search_in_ext=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_args: dictionary with arguments to override while include.
        :type include_args: {str: str}
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :param search_in_ext: file extensions to search in
        :type search_in_ext: [str]
        :return: Returns an iterator for IncludedFile
        :rtype: fkie_node_manager_daemon.common.IncludedFile
        '''
        reply = self._get_included_files(path=root, recursive=recursive, unique=False, include_args=include_args, include_pattern=include_pattern, search_in_ext=search_in_ext)
        for response in reply:
            args = {arg.name: arg.value for arg in response.include_args}
            yield IncludedFile(response.root_path, response.linenr, response.path, response.exists, response.rawname, response.rec_depth, args, size=response.size)

    def get_interpreted_path(self, paths):
        '''
        :param str text: text to search for included files
        :return: Returns an iterator for tuples with interpreted path and file exists or not
        :rtype: tuple(str, bool)
        '''
        request = lmsg.InterpretPaths(paths=paths)
        reply = self.lm_stub.InterpretPath(request, timeout=settings.GRPC_TIMEOUT)
        for response in reply:
            if response.status.code == OK:
                yield (response.path, response.exists)
            else:
                raise exceptions.ResourceNotFound('', response.status.error_msg)

    def _gen_node_list(self, nodes):
        for name, opt_binary, opt_launch, loglevel, logformat, cmd_prefix, daemonuri in nodes:
            yield lmsg.Node(name=name, opt_binary=opt_binary, opt_launch=opt_launch, loglevel=loglevel, logformat=logformat, cmd_prefix=cmd_prefix, daemonuri=daemonuri)

    def start_node(self, name, *, opt_binary='', opt_launch='', loglevel='', logformat='', cmd_prefix='', daemonuri=''):
        '''
        Start node.

        :param str name: full name of the ros node exists in the launch file.
        :param str opt_binary: the full path of the binary. Used in case of multiple binaries in the same package.
        :param str opt_launch: full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
        :param str loglevel: log level
        :param str cmd_prefix: custom command prefix. It will be prepended before launch prefix.
        :param str daemonuri: daemon where the configuration file was loaded for.
        :raise exceptions.StartException: on start errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        :raise exceptions.LaunchSelectionRequest: on multiple launch files
        '''
        response_stream = self.lm_stub.StartNode(self._gen_node_list([(name, opt_binary, opt_launch, loglevel, logformat, cmd_prefix, daemonuri)]), timeout=settings.GRPC_TIMEOUT)
        for response in response_stream:
            if response.status.code == 0:
                pass
            elif response.status.code == ERROR:
                raise exceptions.StartException(response.status.error_msg)
            elif response.status.code == NODE_NOT_FOUND:
                raise exceptions.StartException(response.status.error_msg)
            elif response.status.code == MULTIPLE_BINARIES:
                raise exceptions.BinarySelectionRequest([path for path in response.path], response.status.error_msg)
            elif response.status.code == MULTIPLE_LAUNCHES:
                raise exceptions.LaunchSelectionRequest([path for path in response.launch], response.status.error_msg)
            elif response.status.code == CONNECTION_ERROR:
                raise exceptions.ConnectionException(response.name, response.status.error_msg)

    def start_standalone_node(self, startcfg):
        '''
        Start a node on remote launch manager using a ``StartConfig``

        :param startcfg: start configuration
        :type startcfg: fkie_node_manager_daemon.startcfg.StartConfig
        :raise exceptions.StartException: on errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        '''
        request = startcfg.to_msg()
        response = self.lm_stub.StartStandaloneNode(request, timeout=settings.GRPC_TIMEOUT)
        if response.status.code == 0:
            pass
        elif response.status.code == MULTIPLE_BINARIES:
            raise exceptions.BinarySelectionRequest([path for path in response.path], 'Multiple executables')
        elif response.status.code == FILE_NOT_FOUND:
            raise exceptions.StartException("Can't find %s in %s" % (startcfg.binary, startcfg.package))
        elif response.status.code == ERROR:
            raise exceptions.StartException(response.status.error_msg)

    def get_start_cfg(self, name, *, opt_launch='', loglevel='', logformat='', daemonuri=''):
        '''
        Returns start configuration for a node.

        :param str name: full name of the ros node exists in the launch file.
        :param str opt_binary: the full path of the binary. Used in case of multiple binaries in the same package.
        :param str opt_launch: full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
        :param str loglevel: log level
        :param str daemonuri: daemon where the configuration file was loaded for.
        :raise exceptions.StartException: on start errors
        :raise exceptions.BinarySelectionRequest: on multiple binaries
        :raise exceptions.LaunchSelectionRequest: on multiple launch files
        '''
        response = self.lm_stub.GetStartCfg(lmsg.Node(name=name, opt_binary='', opt_launch=opt_launch, loglevel=loglevel, logformat=logformat, daemonuri=daemonuri), timeout=settings.GRPC_TIMEOUT)
        startcfg = None
        if response.status.code == 0:
            startcfg = StartConfig.from_msg(response.startcfg)
        elif response.status.code == ERROR:
            raise exceptions.StartException(response.status.error_msg)
        elif response.status.code == NODE_NOT_FOUND:
            raise exceptions.StartException(response.status.error_msg)
        elif response.status.code == MULTIPLE_LAUNCHES:
            raise exceptions.LaunchSelectionRequest([path for path in response.launch], response.status.error_msg)
        elif response.status.code == CONNECTION_ERROR:
            raise exceptions.ConnectionException(response.name, response.status.error_msg)
        return startcfg

    def reset_package_path(self):
        request = lmsg.Empty()
        self.lm_stub.ResetPackageCache(request, timeout=settings.GRPC_TIMEOUT)

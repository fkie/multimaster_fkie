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

import json

from typing import List, Dict

from .common import INCLUDE_PATTERN
from .common import SEARCH_IN_EXT

from fkie_master_discovery.crossbar_interface import RosParameter


class LaunchReturnStatus:
    '''
    The status message included in replies.
    '''
    OK = 'OK'
    ERROR = 'ERROR'
    ALREADY_OPEN = 'ALREADY_OPEN'
    MULTIPLE_BINARIES = 'MULTIPLE_BINARIES'
    MULTIPLE_LAUNCHES = 'MULTIPLE_LAUNCHES'
    PARAMS_REQUIRED = 'PARAMS_REQUIRED'
    FILE_NOT_FOUND = 'FILE_NOT_FOUND'
    NODE_NOT_FOUND = 'NODE_NOT_FOUND'
    PACKAGE_NOT_FOUND = 'PACKAGE_NOT_FOUND'
    CONNECTION_ERROR = 'CONNECTION_ERROR'

    def __init__(self, code: str, msg: str = '') -> None:
        self.code = code
        self.msg = msg

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchArgument:

    def __init__(self, name: str, value: str) -> None:
        self.name = name
        self.value = value

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchLoadReply:

    def __init__(self, *, status: LaunchReturnStatus = LaunchReturnStatus('OK'),
                 paths: List[str] = [],
                 args: List[LaunchArgument] = [],
                 changed_nodes: List[str] = []) -> None:
        self.status = status
        self.paths = paths
        self.args = args
        self.changed_nodes = changed_nodes

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchFile:

    def __init__(self, path: str = '', *,
                 masteruri: str = '', host: str = '') -> None:
        self.path = path
        self.masteruri = masteruri
        self.host = host

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchLoadRequest:
    '''
    The request message to list the ROS packages in given path.
    :param str package: ROS package name
    :param str launch: launch file in the package path. If the package contains
                       more then one launch file with same name,
                       you have to specify the sub-path with launch file.
    :param str path: if set, this will be used instead of package/launch
    :param Argument args: arguments to load the launch file. If args are empty but the launch file needs them,
                          the reply status has code PARAMS_REQUIRED and args list will be filled with requested args.
    :param bool force_first_file: if True, use first file if more than one was found in the package.
    :param bool request_args: If True, the launch file will not be loaded, only launch arguments are requested.
    :param str masteruri: starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
                      the nodes are started on the host specified by hostname of the masteruri.
    :param str host: start nodes of this file on specified host.
    '''

    def __init__(self, *, ros_package: str = '', launch: str = '', path: str = '',
                 args: List[LaunchArgument] = [], force_first_file: bool = False,
                 request_args: bool = False,
                 masteruri: str = '', host: str = '') -> None:
        self.ros_package = ros_package
        self.launch = launch
        self.path = path
        self.args = args
        self.force_first_file = force_first_file
        self.request_args = request_args
        self.masteruri = masteruri
        self.host = host

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchNodelets:
    '''
    Represents the nodelets specified in Launchfile.
    :param str manager: nodelete manager
    :param [str] nodes: list with nodes (full name) controlled by nodelet manager.
    '''

    def __init__(self, manager: str, nodes: List[str]) -> None:
        self.manager = manager
        self.nodes = nodes

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchAssociations:
    '''
    Represents the associations specified in Launch file.
    :param str node: node (full name)
    :param [str] nodes: list with associated nodes (full name).
    '''

    def __init__(self, node: str, nodes: List[str]) -> None:
        self.node = node
        self.nodes = nodes

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchContent:
    '''
    Report the nodes of a launch file.
    :param str path: full path of the launch file with contains the reported nodes.
    :param Argument args: arguments used to load the launch file.
    :param str masteruri: starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
                          the nodes are started on the host specified by hostname of the masteruri.
    :param str host: if not empty, the nodes of this launch file are launched on specified host.
    :param [str] nodes: list of node names.
   '''

    def __init__(self, path: str, *,
                 args: List[LaunchArgument] = [],
                 masteruri: str = '',
                 host: str = '',
                 nodes: List[str] = [],
                 parameters: List[RosParameter] = [],
                 nodelets: List[LaunchNodelets] = [],
                 associations: List[LaunchAssociations] = []) -> None:
        self.path = path
        self.args = args
        self.masteruri = masteruri
        self.host = host
        self.nodes = nodes
        self.parameters = parameters
        self.nodelets = nodelets
        self.associations = associations

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchNode:
    '''
    Starts a ROS node by full name.
    :param str name: full name of the ros node exists in the launch file.
    :param str opt_binary: the full path of the binary. Used in case of multiple binaries in the same package.
    :param str opt_launch: full name of the launch file to use. Used in case the node with same name exists in more then one loaded launch file.
    :param str loglevel: log level
    :param str cmd_prefix: custom command prefix. It will be prepended before launch prefix.*/
    '''

    def __init__(self, name: str, *,
                 opt_binary: str = '',
                 opt_launch: str = '',
                 loglevel: str = '',
                 logformat: str = '',
                 masteruri: str = '',
                 reload_global_param: bool = False,
                 cmd_prefix: str = '') -> None:
        self.name = name
        self.opt_binary = opt_binary
        self.opt_launch = opt_launch
        self.loglevel = loglevel
        self.logformat = logformat
        self.masteruri = masteruri
        self.reload_global_param = reload_global_param
        self.cmd_prefix = cmd_prefix

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchNodeReply:
    '''
    The response message with load status
    :param str name: name of the node.
    :param str status: the status of the start process. One of the codes of LaunchReturnStatus. Default: 'OK'
    :param [str] path: a list of paths with binaries for a node, only if MULTIPLE_BINARIES is returned.
    :param [str] launch: a list with names launch files, only if MULTIPLE_LAUNCHES is returned.
    '''

    def __init__(self, name: str, *,
                 status: str = 'OK',
                 paths: List[str] = [],
                 launch_files: List[str] = []) -> None:
        self.name = name
        self.status = LaunchReturnStatus(status)
        self.paths = paths
        self.launch_files = launch_files

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchInterpretPathRequest:
    '''
    Request to parse the text for included paths.
    :param str text: line in the launch config.
    :param [LaunchArgument] launch: a list of the arguments used load the launch file.
    '''

    def __init__(self, text: str, *,
                 args: List[LaunchArgument] = []) -> None:
        self.text = text
        self.args = args

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchInterpretPathReply:
    '''
    Response for a request to parse the text for included paths.
    :param str text: line in the launch config.
    :param str status: the status of the parsing. One of the codes of LaunchReturnStatus. Default: 'OK'
    :param str path: the path of the configuration file containing the text.
    :param bool exists: True if detected include path exists.
    :param [LaunchArgument] launch: a list of the arguments used load the launch file.
    '''

    def __init__(self, text: str, *,
                 status: str = 'OK',
                 path: str = '',
                 exists: bool = False,
                 args: List[LaunchArgument] = []) -> None:
        self.text = text
        self.status = LaunchReturnStatus(status)
        self.path = path
        self.args = args
        self.exists = exists

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchIncludedFilesRequest:
    def __init__(self, path: str, *,
                 recursive: bool = True,
                 unique: bool = False,
                 pattern: List[str] = INCLUDE_PATTERN,
                 search_in_ext: List[str] = SEARCH_IN_EXT,
                 args: List[LaunchArgument] = []) -> None:
        '''
        :param str path: file to parse.
        :param bool recursive: True to read recursive. Default: True.
        :param bool unique: True to ignore files included multiple times. Default: False.
        :param [str] pattern: pattern to change include detection.
        :param [str] search_in_ext: search only for files with given extensions. Default: ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']
        :param [LaunchArgument] include_args: use include launch arguments.
        '''
        self.path = path
        self.recursive = recursive
        self.unique = unique
        self.pattern = pattern
        self.search_in_ext = search_in_ext
        self.args = args

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LaunchIncludedFile:

    def __init__(self, path: str,
                 line_number: int,
                 inc_path: str,
                 exists: bool,
                 raw_inc_path: str,
                 rec_depth: int,
                 args: List[LaunchArgument],
                 default_inc_args: List[LaunchArgument],
                 size: int = 0):
        '''
        Representation of an included file found in given string or path of a file.

        :param str path: current reading file.
        :param int line_number: line number of the occurrence. If `unique` is True the line number is zero.
        :param str inc_path: resolved path.
        :param bool exists: True if resolved path exists.
        :param str raw_inc_path: representation of included file without resolved arg and find statements.
        :param int rec_depth: depth of recursion. if `unique` is True the depth is zero
        :param [LaunchArgument] args: a list with arguments forwarded within include tag for 'inc_path'.
        :param [LaunchArgument] default_inc_args: a list with default arguments defined in 'inc_path'.
        :param int size: size of the file in bytes.
        '''
        self.path = path
        self.line_number = line_number
        self.inc_path = inc_path
        self.exists = exists
        self.raw_inc_path = raw_inc_path
        self.rec_depth = rec_depth
        self.args = args
        self.default_inc_args = default_inc_args
        self.size = size

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)

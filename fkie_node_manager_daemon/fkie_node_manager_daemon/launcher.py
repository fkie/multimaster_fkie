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

from typing import Text

import os
import pathlib
import rospkg
import rclpy
import shlex
import socket
import sys
import types
import launch

import launch_ros
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import to_parameters_list
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions
from ros2pkg.api import PackageNotFound
from ros2run.api import ExecutableNameCompleter
from ros2run.api import get_executable_path
from ros2run.api import MultipleExecutables
from ros2run.api import run_executable
import composition_interfaces.srv

from . import host
from . import exceptions
from . import remote
from . import screen
from . import settings
from .launch_config import LaunchConfig
from .launch_context import LaunchContext
from .launch_stub import LaunchStub
from .common import get_cwd, get_namespace, package_name, interpret_path, ns_join
from .supervised_popen import SupervisedPopen
from .startcfg import StartConfig
from .url import nmduri as url_nmduri
import fkie_node_manager_daemon as nm

STARTED_BINARIES = dict()
''':var STARTED_BINARIES: dictionary with nodes and tuple of (paths of started binaries and their last modification time). Used to detect changes on binaries.'''


def create_start_config(node, launchcfg, *, executable='', daemonuri=None, loglevel='', logformat='', cmd_prefix='') -> StartConfig:
    '''
    :param str cmd_prefix: custom command prefix. It will be prepended before launch prefix.
    :return: Returns start configuration created from loaded launch file.
    :rtype: fkie_node_manager_daemon.startcfg.StartConfig
    '''
    n = launchcfg.get_node(node)
    print("NODE:", n, type(n))
    if n is None:
        raise exceptions.StartException(
            "Node '%s' not found in launch file %s" % (node, launchcfg.filename))
    result = StartConfig(n.package, n.type)
    result.config_path = launchcfg.filename
    if executable:
        result.binary_path = executable
    result.name = n.name
    result.namespace = n.namespace
    result.fullname = node
    # set launch prefix
    prefix = n.launch_prefix if n.launch_prefix is not None else ''
    if prefix.lower() == 'screen' or prefix.lower().find('screen ') != -1:
        nm.rosnode.get_logger().info("SCREEN prefix removed before start!")
        prefix = ''
    result.prefix = '%s %s' % (cmd_prefix, prefix) if cmd_prefix else prefix
    result.env = {key: value for key, value in n.env_args}
    # set remapings
    result.remaps = {remap[0]: remap[1] for remap in n.remap_args}
    # set respawn parameter
    if n.respawn:
        result.respawn = n.respawn
        if n.respawn_delay > 0:
            result.respawn_delay = n.respawn_delay
        respawn_params = _get_respawn_params(
            ns_join(n.namespace, n.name), launchcfg.roscfg.params, result.respawn_delay)
        result.respawn_max = respawn_params['max']
        result.respawn_min_runtime = respawn_params['min_runtime']
        result.respawn_delay = respawn_params['delay']
    # set log level
    result.loglevel = loglevel
    result.logformat = logformat
    result.daemonuri = launchcfg.daemonuri
    if not result.daemonuri:
        result.daemonuri = daemonuri
    # override host with machine tag
    if n.machine_name and n.machine_name in launchcfg.roscfg.machines:
        result.daemonuri = url_nmduri(
            launchcfg.roscfg.machines[n.machine_name].address)
    # set args
    result.args = n.args.split()
    # set cwd unchanged, it will be resolved on host
    result.cwd = n.cwd
    # add params and clear_params
    nodens = "%s%s%s" % (n.namespace, n.name, '/')
    for pname, param in launchcfg.roscfg.params.items():
        if pname.startswith(nodens):
            result.params[pname] = param.value
    for cparam in launchcfg.roscfg.clear_params:
        if cparam.startswith(nodens):
            result.clear_params.append(cparam)
    nm.rosnode.get_logger().debug("set delete parameter:\n  %s" %
                                  '\n  '.join(result.clear_params))
    nm.rosnode.get_logger().debug("add parameter:\n  %s" % '\n  '.join("%s: %s%s" % (key, str(
        val)[:80], '...' if len(str(val)) > 80 else '') for key, val in result.params.items()))
    return result


def from_node(node: launch.actions.execute_process.ExecuteProcess, launchcfg: LaunchConfig, *, executable: Text = '', loglevel: Text = '', logformat: Text = '', cmd_prefix: Text = '') -> StartConfig:
    lc = launchcfg.context
    pkg_exec = ('', '')
    if type(node) in [launch_ros.actions.node.Node, launch_ros.actions.composable_node_container.ComposableNodeContainer]:
        pkg_exec = get_package_exec(node, lc)
    result = StartConfig(*pkg_exec)
    result.config_path = launchcfg.filename
    if executable:
        result.binary_path = executable
    result.fullname = LaunchConfig.get_name_from_node(node)
    result.namespace = get_namespace(result.fullname, with_sep_suffix=False)
    result.name = os.path.basename(result.fullname)

    # set launch prefix
    prefix = perform_substitutions(lc, node._ExecuteProcess__prefix)
    if prefix.lower() == 'screen' or prefix.lower().find('screen ') != -1:
        nm.rosnode.get_logger().info("SCREEN prefix removed before start!")
        prefix = ''
    result.prefix = '%s %s' % (cmd_prefix, prefix) if cmd_prefix else prefix

    # set remapings
    result.remaps = {}
    if hasattr(node, '_Node__expanded_remappings'):
        if node._Node__expanded_remappings is not None:
            for remapping_from, remapping_to in node._Node__expanded_remappings:
                result.remaps[remapping_from] = remapping_to

    # set respawn parameter, not supported by ROS2
    respawn_params = _get_respawn_params(node)
    result.respawn = respawn_params['enabled']
    result.respawn_max = respawn_params['max']
    result.respawn_min_runtime = respawn_params['min_runtime']
    result.respawn_delay = respawn_params['delay']
    # set log level
    result.loglevel = loglevel
    result.logformat = logformat

    result.daemonuri = launchcfg.daemonuri
    # override host with machine tag, not supported by ROS2
#    if n.machine_name and n.machine_name in launchcfg.roscfg.machines:
#        result.host = launchcfg.roscfg.machines[n.machine_name].address

    # set args
    # TODO: which args?
    # result.args = n.args.split()
    # add params
    if hasattr(node, '_Node__parameters'):
        if node._Node__parameters is not None:
            evaluated_parameters = evaluate_parameters(
                lc, node._Node__parameters)
            for params in evaluated_parameters:
                if isinstance(params, dict):
                    result.params.update(params)
                elif isinstance(params, pathlib.Path):
                    result.param_files.append(str(params))
                else:
                    raise RuntimeError(
                        'invalid normalized parameters {}'.format(repr(params)))
    nm.rosnode.get_logger().debug("add parameter:\n  %s" % '\n  '.join("%s: %s%s" % (key, str(
        val)[:80], '...' if len(str(val)) > 80 else '') for key, val in result.params.items()))

    # Prepare the ros_specific_arguments list and add it to the context so that the
    # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
    # ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
    # if self.__node_name is not None:
    #     ros_specific_arguments['name'] = '__node:={}'.format(self.__expanded_node_name)
    # if self.__expanded_node_namespace != '':
    #     ros_specific_arguments['ns'] = '__ns:={}'.format(self.__expanded_node_namespace)
    # if self.__expanded_parameter_files is not None:
    #     ros_specific_arguments['params'] = self.__expanded_parameter_files

    # context.extend_locals({'ros_specific_arguments': ros_specific_arguments})

    # from ExecuteProcess
    # expand substitutions in arguments to async_execute_process()
    # cmd = [perform_substitutions(lc, x) for x in node._ExecuteProcess__cmd]
    # name = os.path.basename(cmd[0]) if node._ExecuteProcess__name is None \
    #     else perform_substitutions(lc, node._ExecuteProcess__name)
    # with _global_process_counter_lock:
    #     global _global_process_counter
    #     _global_process_counter += 1
    #     node._ExecuteProcess__name = '{}-{}'.format(name, _global_process_counter)
    result.cwd = None
    if node._ExecuteProcess__cwd is not None:
        result.cwd = ''.join([lc.perform_substitution(x)
                              for x in node._ExecuteProcess__cwd])

    env = {}
    if node._ExecuteProcess__env is not None:
        for key, value in node._ExecuteProcess__env:
            env[''.join([lc.perform_substitution(x) for x in key])] = \
                ''.join([lc.perform_substitution(x) for x in value])
    if node._ExecuteProcess__additional_env is not None:
        for key, value in node._ExecuteProcess__additional_env:
            env[''.join([lc.perform_substitution(x) for x in key])] = \
                ''.join([lc.perform_substitution(x) for x in value])
    result.env = env
    if type(node) == launch.actions.execute_process.ExecuteProcess:
        result.cmd = [perform_substitutions(lc, x) for x in node.cmd]
    return result


def get_package_exec(node: launch_ros.actions.node.Node, context: LaunchContext) -> (str, str):
    for cmds in node.cmd:
        for cmd in cmds:
            if isinstance(cmd, launch_ros.substitutions.executable_in_package.ExecutableInPackage):
                executable = perform_substitutions(
                    context, cmd.executable if cmd.executable else cmd.node_executable)
                package = perform_substitutions(context, cmd.package)
                return (package, executable)
            else:
                print('CMD', type(cmd), dir(cmd))
                if hasattr(cmd, 'executable'):
                    print('  EXEC', type(cmd.executable))
                    print('   rpl', perform_substitutions(
                        context, cmd.executable))
    raise NameError('pkg or exec tag not found')


def run_node(startcfg):
    '''
    Start a node local or on specified host using a :class:`.startcfg.StartConfig`

    :param startcfg: start configuration e.g. returned by :meth:`create_start_config`
    :type startcfg: :class:`fkie_node_manager_daemon.startcfg.StartConfig`
    :raise exceptions.StartException: on errors
    :raise exceptions.BinarySelectionRequest: on multiple binaries
    :see: :meth:`fkie_node_manager.host.is_local`
    '''
    hostname = startcfg.hostname
    nodename = startcfg.namespace.rstrip('/') + '/' + startcfg.name
    if not hostname or host.is_local(hostname, wait=True):
        # run on local host
        # interpret arguments with path elements
        args = []
        for arg in startcfg.args:
            new_arg = arg
            # TODO: check if we have to prepand --ros-args
            if arg.startswith('$(find'):
                new_arg = interpret_path(arg)
                nm.rosnode.get_logger().debug("interpret arg '%s' to '%s'" % (arg, new_arg))
            args.append(new_arg)
        print("startcfg", startcfg)
        if not startcfg.cmd:  # it is a node
            # set name and namespace of the node
            if startcfg.name or startcfg.namespace or startcfg.remaps or startcfg.params or startcfg.param_files:
                args.append('--ros-args')
            if startcfg.name:
                args.append('-r __node:=%s' % startcfg.name)
            if startcfg.namespace:
                args.append('-r __ns:=%s' % startcfg.namespace)
            # add remap arguments
            for key, val in startcfg.remaps.items():
                args.append('-r %s:=%s' % (key, val))
            # add param arguments
            for key, val in startcfg.params.items():
                args.append('-p "%s:=%s"' % (key, val))
            # add param_files arguments
            for fname in startcfg.param_files:
                args.append('--params-file "%s"' % fname)
        cmd_type = startcfg.binary_path
        if startcfg.cmd:
            cmd_type = ''
            for cmd_part in startcfg.cmd:
                if ' ' in cmd_type:
                    cmd_type += shlex.quote(cmd_part)
                else:
                    cmd_type += cmd_part
                cmd_type += ' '
            cwd = startcfg.cwd
        # get binary path from package
        elif not cmd_type:
            try:
                cmd_type = get_executable_path(
                    package_name=startcfg.package, executable_name=startcfg.binary)
            except PackageNotFound as e:
                nm.rosnode.get_logger().warn("resource not found: %s" % e)
                raise exceptions.ResourceNotFound(
                    startcfg.package, "resource not found: %s" % e)
            except MultipleExecutables as e:
                err = 'Multiple executables with same name in package [%s]  found:' % startcfg.package
                # for p in e.paths:
                #     err += f'\n- {p}'
                raise exceptions.BinarySelectionRequest(e.paths, err)
            cwd = get_cwd(startcfg.cwd, cmd_type)
            try:
                global STARTED_BINARIES
                STARTED_BINARIES[nodename] = (
                    cmd_type, os.path.getmtime(cmd_type))
            except Exception:
                pass
        # set environment
        new_env = dict(os.environ)
        # set display variable to local display
        if 'DISPLAY' in startcfg.env:
            if not startcfg.env['DISPLAY'] or startcfg.env['DISPLAY'] == 'remote':
                del startcfg.env['DISPLAY']
        else:
            new_env['DISPLAY'] = ':0'
        # add environment from launch
        new_env.update(startcfg.env)
        if startcfg.namespace:
            new_env['ROS_NAMESPACE'] = startcfg.namespace
        # set logging
        if startcfg.logformat:
            new_env['ROSCONSOLE_FORMAT'] = '%s' % startcfg.logformat
        if startcfg.loglevel:
            new_env['ROSCONSOLE_CONFIG_FILE'] = _rosconsole_cfg_file(
                startcfg.package, startcfg.loglevel)
        # handle respawn
        if startcfg.respawn:
            if startcfg.respawn_delay > 0:
                new_env['RESPAWN_DELAY'] = '%d' % startcfg.respawn_delay
            # TODO
            # respawn_params = _get_respawn_params(startcfg.fullname, startcfg.params)
            # if respawn_params['max'] > 0:
            #     new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
            # if respawn_params['min_runtime'] > 0:
            #     new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
            cmd_type = "%s %s %s" % (
                settings.RESPAWN_SCRIPT, startcfg.prefix, cmd_type)
        else:
            cmd_type = "%s %s" % (startcfg.prefix, cmd_type)
        # TODO: check for HOSTNAME
        # masteruri = startcfg.masteruri
        # if masteruri is None:
        #     masteruri = masteruri_from_ros()
        # if masteruri is not None:
        #     if 'ROS_MASTER_URI' not in startcfg.env:
        #         new_env['ROS_MASTER_URI'] = masteruri
        #     # host in startcfg is a nmduri -> get host name
        #     ros_hostname = host.get_ros_hostname(masteruri, hostname)
        #     if ros_hostname:
        #         addr = socket.gethostbyname(ros_hostname)
        #         if addr in set(ip for _n, ip in DiscoverSocket.localifs()):
        #             new_env['ROS_HOSTNAME'] = ros_hostname
            # load params to ROS master
            #_load_parameters(masteruri, startcfg.params, startcfg.clear_params)
        # start
        # cmd_str = '%s %s %s' % (screen.get_cmd(startcfg.fullname, new_env, startcfg.env.keys()), cmd_type, ' '.join(args))
        cmd_str = ' '.join([screen.get_cmd(
            startcfg.fullname, new_env, startcfg.env.keys()), cmd_type, *args])
        nm.rosnode.get_logger().info("%s (launch_file: '%s', daemonuri: %s)" %
                                     (cmd_str, startcfg.config_path, startcfg.daemonuri))
        nm.rosnode.get_logger().debug(
            "environment while run node '%s': '%s'" % (cmd_str, new_env))
        SupervisedPopen(shlex.split(cmd_str), cwd=cwd, env=new_env, object_id="run_node_%s" %
                        startcfg.fullname, description="Run [%s]%s" % (startcfg.package, startcfg.binary), rosnode=nm.rosnode)
    else:
        nmduri = startcfg.nmduri
        nm.rosnode.get_logger().info("remote run node '%s' at '%s'" % (nodename, nmduri))
        startcfg.params.update(_params_to_package_path(startcfg.params))
        startcfg.args = _args_to_package_path(startcfg.args)
        # run on a remote machine
        channel = remote.open_channel(nmduri, rosnode=nm.rosnode)
        if channel is None:
            raise exceptions.StartException(
                "Unknown launch manager url for host %s to start %s" % (nmduri, startcfg.fullname))
        lm = LaunchStub(channel)
        lm.start_standalone_node(startcfg)


def run_composed_node(node: launch_ros.descriptions.ComposableNode, *, container_name: Text, context: LaunchContext):
    # Create a client to load nodes in the target container.
    client_load_node = nm.rosnode.create_client(
        composition_interfaces.srv.LoadNode, '%s/_container/load_node' % container_name)
    composable_node_description = node
    request = composition_interfaces.srv.LoadNode.Request()
    request.package_name = perform_substitutions(
        context, composable_node_description.package
    )
    request.plugin_name = perform_substitutions(
        context, composable_node_description.node_plugin
    )
    if composable_node_description.node_name is not None:
        request.node_name = perform_substitutions(
            context, composable_node_description.node_name
        )
    if composable_node_description.node_namespace is not None:
        request.node_namespace = perform_substitutions(
            context, composable_node_description.node_namespace
        )
    # request.log_level = perform_substitutions(context, node_description.log_level)
    if composable_node_description.remappings is not None:
        for from_, to in composable_node_description.remappings:
            request.remap_rules.append('{}:={}'.format(
                perform_substitutions(context, list(from_)),
                perform_substitutions(context, list(to)),
            ))
    if composable_node_description.parameters is not None:
        request.parameters = [
            param.to_parameter_msg() for param in to_parameters_list(
                context, evaluate_parameters(
                    context, composable_node_description.parameters
                )
            )
        ]
    if composable_node_description.extra_arguments is not None:
        request.extra_arguments = [
            param.to_parameter_msg() for param in to_parameters_list(
                context, evaluate_parameters(
                    context, composable_node_description.extra_arguments
                )
            )
        ]
    print("wait for container")
    if not client_load_node.wait_for_service(timeout_sec=1.0):
        error_msg = 'Timeout while wait for container %s to load %s' % (
            container_name, request.plugin_name)
        nm.rosnode.get_logger().error(error_msg)
        nm.rosnode.destroy_client(client_load_node)
        raise exceptions.StartException(error_msg)
    print("call client")
    response = client_load_node.call(request)
    print("response received")
    node_name = response.full_node_name if response.full_node_name else request.node_name
    nm.rosnode.destroy_client(client_load_node)
    if response.success:
        # if node_name is not None:
        #     add_node_name(context, node_name)
        #     node_name_count = get_node_name_count(context, node_name)
        #     if node_name_count > 1:
        #         container_logger = launch.logging.get_logger(self.__target_container.name)
        #         container_logger.warning(
        #             'there are now at least {} nodes with the name {} created within this '
        #             'launch context'.format(node_name_count, node_name)
        #         )
        nm.rosnode.get_logger().info("Loaded node '{}' in container '{}'".format(
            response.full_node_name, container_name
        ))
    else:
        error_msg = "Failed to load node '{}' of type '{}' in container '{}': {}".format(
            node_name, request.plugin_name, container_name,
            response.error_message
        )
        nm.rosnode.get_logger().error(error_msg)
        raise exceptions.StartException(error_msg)
    print("LOADED")


def changed_binaries(nodes):
    '''
    Checks for each ROS-node however the binary used for the start was changed.

    :param nodes: list of ROS-node names to check
    :type nodes: list(str)
    :return: list with ROS-nodes with changed binary
    :rtype: list(str)
    '''
    result = []
    global STARTED_BINARIES
    for nodename in nodes:
        try:
            binary, mtime = STARTED_BINARIES[nodename]
            new_mtime = os.path.getmtime(binary)
            if mtime != new_mtime:
                result.append((nodename, new_mtime))
        except KeyError:
            pass
        except Exception:
            print(" Error while check changed binary for %s" % nodename)
            import traceback
            print(traceback.format_exc())
    return result


def _rosconsole_cfg_file(package, loglevel='INFO'):
    result = os.path.join(settings.LOG_PATH, '%s.rosconsole.config' % package)
    with open(result, 'w') as cfg_file:
        cfg_file.write('log4j.logger.ros=%s\n' % loglevel)
        cfg_file.write('log4j.logger.ros.roscpp=INFO\n')
        cfg_file.write('log4j.logger.ros.roscpp.superdebug=WARN\n')
    return result


def _get_respawn_params(node: launch_ros.actions.node.Node) -> {}:
    result = {'enabled': False, 'max': 0, 'min_runtime': 0, 'delay': 0}
    return result
    # TODO
    respawn_max = rospy.names.ns_join(node, 'respawn/max')
    respawn_min_runtime = rospy.names.ns_join(node, 'respawn/min_runtime')
    respawn_delay = rospy.names.ns_join(node, 'respawn/delay')
    try:
        result['max'] = int(params[respawn_max].value)
    except Exception:
        pass
    try:
        result['min_runtime'] = int(params[respawn_min_runtime].value)
    except Exception:
        pass
    try:
        result['delay'] = int(params[respawn_delay].value)
    except Exception:
        pass
    return result


# def _load_parameters(masteruri, params, clear_params):
#     """
#     Load parameters onto the parameter server
#     """
#     param_server = xmlrpclib.ServerProxy(masteruri)
#     p = None
#     abs_paths = list()  # tuples of (parameter name, old value, new value)
#     not_found_packages = list()  # packages names
#     param_errors = []
#     try:
#         socket.setdefaulttimeout(6 + len(clear_params))
#         # multi-call style xmlrpc
#         param_server_multi = xmlrpclib.MultiCall(param_server)

#         # clear specified parameter namespaces
#         # #2468 unify clear params to prevent error
#         for p in clear_params:
#             param_server_multi.deleteParam(nm.rosnode.get_name(), p)
#         r = param_server_multi()
#         for code, msg, _ in r:
#             if code != 1 and not msg.find("is not set"):
#                 nm.rosnode.get_logger().warn("Failed to clear parameter: %s", msg)
# #          raise StartException("Failed to clear parameter: %s"%(msg))

#         # multi-call objects are not reusable
#         socket.setdefaulttimeout(6 + len(params))
#         param_server_multi = xmlrpclib.MultiCall(param_server)
#         for pkey, pval in params.items():
#             value = pval
#             # resolve path elements
#             if isinstance(value, types.StringTypes) and (value.startswith('$')):
#                 value = interpret_path(value)
#                 nm.rosnode.get_logger().debug("interpret parameter '%s' to '%s'" % (value, pval))
#             # add parameter to the multicall
#             param_server_multi.setParam(nm.rosnode.get_name(), pkey, value)
#             test_ret = _test_value(pkey, value)
#             if test_ret:
#                 param_errors.extend(test_ret)
#         r = param_server_multi()
#         for code, msg, _ in r:
#             if code != 1:
#                 raise exceptions.StartException("Failed to set parameter: %s" % (msg))
#     except roslaunch.core.RLException as e:
#         raise exceptions.StartException(e)
#     except rospkg.ResourceNotFound as rnf:
#         raise exceptions.StartException("Failed to set parameter. ResourceNotFound: %s" % (rnf))
#     except Exception as e:
#         raise exceptions.StartException("Failed to set parameter. ROS Parameter Server "
#                                         "reports: %s\n\n%s" % (e, '\n'.join(param_errors)))
#     finally:
#         socket.setdefaulttimeout(None)
#     return abs_paths, not_found_packages


def _test_value(key, value):
    result = []
    if value is None:
        msg = "Invalid parameter value of '%s': '%s'" % (key, value)
        result.append(msg)
        nm.rosnode.get_logger().warn(msg)
    elif isinstance(value, list):
        for val in value:
            ret = _test_value(key, val)
            if ret:
                result.extend(ret)
    elif isinstance(value, dict):
        for subkey, val in value.items():
            ret = _test_value("%s/%s" % (key, subkey), val)
            if ret:
                result.extend(ret)
    return result


def _abs_to_package_path(path):
    result = path
    pname, ppath = package_name(path)
    if pname:
        result = path.replace(ppath, '$(find-pkg-share %s)' % pname)
        nm.rosnode.get_logger().debug("replace abs path '%s' by '%s'" % (path, result))
    return result


def _params_to_package_path(params):
    result = {}
    for name, value in params.items():
        if isinstance(value, str):
            if value.startswith('/') and (os.path.isfile(value) or os.path.isdir(value)):
                result[name] = _abs_to_package_path(value)
    return result


def _args_to_package_path(args):
    result = []
    for arg in args:
        new_arg = arg
        if arg.startswith('/') and (os.path.isfile(arg) or os.path.isdir(arg)):
            new_arg = _abs_to_package_path(arg)
        result.append(new_arg)
    return result


def get_global_params(roscfg):
    '''
    Return the parameter of the configuration file, which are not associated with
    any nodes in the configuration.

    :param roscfg: the launch configuration
    :type roscfg: roslaunch.ROSLaunchConfig<http://docs.ros.org/kinetic/api/roslaunch/html/>
    :return: the dictionary with names of the global parameter and their values
    :rtype: dict(str: value type)
    '''
    result = dict()
    nodes = []
    for item in roscfg.resolved_node_names:
        nodes.append(item)
    for name, param in roscfg.params.items():
        nodesparam = False
        for n in nodes:
            if name.startswith(n):
                nodesparam = True
                break
        if not nodesparam:
            result[name] = param.value
    return result

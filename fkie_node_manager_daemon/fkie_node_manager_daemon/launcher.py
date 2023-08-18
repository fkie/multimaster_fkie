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

from typing import Dict
from typing import Text
from typing import Tuple
from typing import Any

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
from launch_ros.utilities.evaluate_parameters import evaluate_parameters, evaluate_parameter_dict
from launch_ros.utilities import to_parameters_list
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions
from ros2pkg.api import PackageNotFound
from ros2run.api import ExecutableNameCompleter
from ros2run.api import get_executable_path
from ros2run.api import MultipleExecutables
from ros2run.api import run_executable
import composition_interfaces.srv

from .launch_config import LaunchConfig
from .launch_config import LaunchNodeWrapper
from launch.launch_context import LaunchContext
# from .launch_stub import LaunchStub  <- TODO: use crossbar instead
from .startcfg import StartConfig
import fkie_node_manager_daemon as nmd
from fkie_multimaster_pylib import names
from fkie_multimaster_pylib import ros_pkg
from fkie_multimaster_pylib.defines import LOG_PATH
from fkie_multimaster_pylib.defines import RESPAWN_SCRIPT
from fkie_multimaster_pylib.launch import xml
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.system import exceptions
from fkie_multimaster_pylib.system import host
from fkie_multimaster_pylib.system import screen
from fkie_multimaster_pylib.system.supervised_popen import SupervisedPopen


STARTED_BINARIES = dict()
''':var STARTED_BINARIES: dictionary with nodes and tuple of (paths of started binaries and their last modification time). Used to detect changes on binaries.'''


def run_node(node: LaunchNodeWrapper):
    '''
    Start a node local or on specified host using a :class:`.startcfg.StartConfig`

    :param startcfg: start configuration e.g. returned by :meth:`create_start_config`
    :type startcfg: :class:`fkie_node_manager_daemon.startcfg.StartConfig`
    :raise exceptions.StartException: on errors
    :raise exceptions.BinarySelectionRequest: on multiple binaries
    :see: :meth:`fkie_node_manager.host.is_local`
    '''
    # run on local host
    # set environment
    new_env = dict(os.environ) if node.env is None else dict(node.env)
    # set display variable to local display
    if 'DISPLAY' in new_env:
        if not new_env['DISPLAY'] or new_env['DISPLAY'] == 'remote':
            del new_env['DISPLAY']
    else:
        new_env['DISPLAY'] = ':0'
    # add environment from launch
    if node.additional_env:
        new_env.update(dict(node.additional_env))
    if node.node_namespace:
        new_env['ROS_NAMESPACE'] = node.node_namespace
    # set logging
    if node.output_format:
        new_env['ROSCONSOLE_FORMAT'] = '%s' % node.output_format
    # if node.loglevel:
    #     new_env['ROSCONSOLE_CONFIG_FILE'] = _rosconsole_cfg_file(
    #         node.package, node.loglevel)
    # handle respawn
    respawn_prefix = ''
    if node.respawn:
        if node.respawn_delay > 0:
            new_env['RESPAWN_DELAY'] = '%d' % node.respawn_delay
        # TODO
        # respawn_params = _get_respawn_params(node.fullname, node.params)
        # if respawn_params['max'] > 0:
        #     new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
        # if respawn_params['min_runtime'] > 0:
        #     new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
        respawn_prefix = f"{RESPAWN_SCRIPT}"

    try:
        global STARTED_BINARIES
        STARTED_BINARIES[node.unique_name] = (node.executable, os.path.getmtime(node.executable))
    except Exception:
        pass


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
    screen_prefix = ' '.join([screen.get_cmd(
        node.unique_name, new_env, new_env.keys())])
    nmd.ros_node.get_logger().info(
        f"{screen_prefix} {respawn_prefix} {node.launch_prefix} {node.cmd} (launch_file: '{node.launch_name}')")
    nmd.ros_node.get_logger().debug(
        f"environment while run node '{node.unique_name}': '{new_env}'")
    SupervisedPopen(shlex.split(' '.join([screen_prefix, respawn_prefix, node.cmd])), cwd=node.cwd, env=new_env,
                    object_id=f"run_node_{node.unique_name}", description=f"Run [{node.package_name}]{node.executable}")

#     else:
#         nmduri = startcfg.nmduri
#         nmd.ros_node.get_logger().info("remote run node '%s' at '%s'" % (nodename, nmduri))
#         startcfg.params.update(_params_to_package_path(startcfg.params))
#         startcfg.args = _args_to_package_path(startcfg.args)
#         # run on a remote machine
#         channel = remote.open_channel(nmduri, rosnode=nmd.ros_node)
#         if channel is None:
#             raise exceptions.StartException(
#                 "Unknown launch manager url for host %s to start %s" % (nmduri, startcfg.fullname))
#         # TODO: remote start using crossar
#         #lm = LaunchStub(channel)
#         # lm.start_standalone_node(startcfg)


def run_composed_node(node: launch_ros.descriptions.ComposableNode, *, container_name: Text, context: LaunchContext):
    # Create a client to load nodes in the target container.
    client_load_node = nmd.ros_node.create_client(
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
        nmd.ros_node.get_logger().error(error_msg)
        nmd.ros_node.destroy_client(client_load_node)
        raise exceptions.StartException(error_msg)
    print("call client")
    response = client_load_node.call(request)
    print("response received")
    node_name = response.full_node_name if response.full_node_name else request.node_name
    nmd.ros_node.destroy_client(client_load_node)
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
        nmd.ros_node.get_logger().info("Loaded node '{}' in container '{}'".format(
            response.full_node_name, container_name
        ))
    else:
        error_msg = "Failed to load node '{}' of type '{}' in container '{}': {}".format(
            node_name, request.plugin_name, container_name,
            response.error_message
        )
        nmd.ros_node.get_logger().error(error_msg)
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
    result = os.path.join(LOG_PATH, '%s.rosconsole.config' % package)
    with open(result, 'w') as cfg_file:
        cfg_file.write('log4j.logger.ros=%s\n' % loglevel)
        cfg_file.write('log4j.logger.ros.roscpp=INFO\n')
        cfg_file.write('log4j.logger.ros.roscpp.superdebug=WARN\n')
    return result

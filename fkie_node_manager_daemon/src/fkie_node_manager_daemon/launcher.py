# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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



import os
import roslib
import rospkg
import rospy
import shlex
import socket
import sys
import types
import roslaunch
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

from rosgraph.network import get_local_addresses
from fkie_master_discovery.common import masteruri_from_ros
from fkie_master_discovery.udp import DiscoverSocket

from . import host
from . import exceptions
from . import remote
from . import screen
from . import settings
from .launch_stub import LaunchStub
from .common import get_cwd, package_name, interpret_path, isstring, utf8

from .supervised_popen import SupervisedPopen
from .startcfg import StartConfig

STARTED_BINARIES = dict()
''':var STARTED_BINARIES: dictionary with nodes and tuple of (paths of started binaries and their last modification time). Used to detect changes on binaries.'''


def create_start_config(node, launchcfg, executable='', masteruri=None, loglevel='', logformat='', reload_global_param=False, cmd_prefix=''):
    '''
    :param str cmd_prefix: custom command prefix. It will be prepended before launch prefix.
    :return: Returns start configuration created from loaded launch file.
    :rtype: fkie_node_manager_daemon.startcfg.StartConfig
    '''
    n = launchcfg.get_node(node)
    if n is None:
        raise exceptions.StartException("Node '%s' not found in launch file %s" % (node, launchcfg.filename))
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
        rospy.loginfo("SCREEN prefix removed before start!")
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
        respawn_params = _get_respawn_params(rospy.names.ns_join(n.namespace, n.name), launchcfg.roscfg.params, result.respawn_delay)
        result.respawn_max = respawn_params['max']
        result.respawn_min_runtime = respawn_params['min_runtime']
        result.respawn_delay = respawn_params['delay']
    # set log level
    result.loglevel = loglevel
    result.logformat = logformat
    # set masteruri and host config
    if 'ROS_MASTER_URI' in result.env:
        result.masteruri = result.env['ROS_MASTER_URI']
        result.host = launchcfg.host
        if not result.host and masteruri:
            result.host = host.get_hostname(masteruri)
    else:
        result.masteruri = masteruri if masteruri or masteruri is None else None
        result.host = launchcfg.host
    # override host with machine tag
    if n.machine_name and n.machine_name in launchcfg.roscfg.machines:
        result.host = launchcfg.roscfg.machines[n.machine_name].address
    # set args
    result.args = n.args.split()
    # set cwd unchanged, it will be resolved on host
    result.cwd = n.cwd
    # add global parameter on start of first node of the launch
    if reload_global_param or launchcfg.changed:
        if result.masteruri in launchcfg.global_param_done:
            launchcfg.global_param_done.remove(result.masteruri)
            launchcfg.changed = False
    if result.masteruri not in launchcfg.global_param_done:
        global_params = get_global_params(launchcfg.roscfg)
        result.params.update(global_params)
        rospy.loginfo("add global parameter for '%s'" % launchcfg.filename)
        rospy.logdebug("add global parameter:\n  %s", '\n  '.join("%s: %s%s" % (key, utf8(val)[:80], '...' if len(utf8(val)) > 80 else'') for key, val in global_params.items()))
        launchcfg.global_param_done.append(result.masteruri)
    # add params and clear_params
    nodens = "%s%s%s" % (n.namespace, n.name, rospy.names.SEP)
    for pname, param in launchcfg.roscfg.params.items():
        if pname.startswith(nodens):
            result.params[pname] = param.value
    for cparam in launchcfg.roscfg.clear_params:
        if cparam.startswith(nodens):
            result.clear_params.append(cparam)
    rospy.logdebug("set delete parameter:\n  %s", '\n  '.join(result.clear_params))
    rospy.logdebug("add parameter:\n  %s", '\n  '.join("%s: %s%s" % (key, utf8(val)[:80], '...' if len(utf8(val)) > 80 else '') for key, val in result.params.items()))
    return result


def remove_src_binary(cmdlist):
    result = []
    count = 0
    if len(cmdlist) > 1:
        for c in cmdlist:
            if c.find('/src/') == -1:
                result.append(c)
                count += 1
    else:
        result = cmdlist
    if count > 1:
        # we have more binaries in src directory
        # aks the user
        result = cmdlist
    return result


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
    nodename = roslib.names.ns_join(startcfg.namespace, startcfg.name)
    if not hostname or host.is_local(hostname, wait=True):
        # run on local host
        # interpret arguments with path elements
        args = []
        for arg in startcfg.args:
            new_arg = arg
            if arg.startswith('$(find'):
                new_arg = interpret_path(arg)
                rospy.logdebug("interpret arg '%s' to '%s'" % (arg, new_arg))
            args.append(new_arg)
        # set name and namespace of the node
        if startcfg.name:
            args.append("__name:=%s" % startcfg.name)
        if startcfg.namespace:
            args.append("__ns:=%s" % startcfg.namespace)
        # add remap arguments
        for key, val in startcfg.remaps.items():
            args.append("%s:=%s" % (key, val))
        cmd_type = startcfg.binary_path
        # get binary path from package
        if not cmd_type:
            try:
                cmd = roslib.packages.find_node(startcfg.package, startcfg.binary)
            except (roslib.packages.ROSPkgException, rospkg.ResourceNotFound) as e:
                # multiple nodes, invalid package
                rospy.logwarn("resource not found: %s" % utf8(e))
                raise exceptions.ResourceNotFound(startcfg.package, "resource not found: %s" % utf8(e))
            if isstring(cmd):
                cmd = [cmd]
            if cmd is None or len(cmd) == 0:
                raise exceptions.StartException('%s in package [%s] not found!' % (startcfg.binary, startcfg.package))
            cmd = remove_src_binary(cmd)
            if len(cmd) > 1:
                # Open selection for executables
                err = 'Multiple executables with same name in package [%s]  found:' % startcfg.package
                raise exceptions.BinarySelectionRequest(cmd, err)
            else:
                cmd_type = cmd[0]
        try:
            global STARTED_BINARIES
            STARTED_BINARIES[nodename] = (cmd_type, os.path.getmtime(cmd_type))
        except Exception:
            pass
        cwd = get_cwd(startcfg.cwd, cmd_type)
        # set environment
        new_env = dict(os.environ)
        # set display variable to local display
        if 'DISPLAY' in startcfg.env:
            if not startcfg.env['DISPLAY'] or startcfg.env['DISPLAY'] == 'remote':
                del startcfg.env['DISPLAY']
        #else:
        #    new_env['DISPLAY'] = ':0'
        # add environment from launch
        new_env.update(startcfg.env)
        if startcfg.namespace:
            new_env['ROS_NAMESPACE'] = startcfg.namespace
        # set logging
        if startcfg.logformat:
            new_env['ROSCONSOLE_FORMAT'] = '%s' % startcfg.logformat
        if startcfg.loglevel:
            new_env['ROSCONSOLE_CONFIG_FILE'] = _rosconsole_cfg_file(startcfg.package, startcfg.loglevel)
        # handle respawn
        if startcfg.respawn:
            if startcfg.respawn_delay > 0:
                new_env['RESPAWN_DELAY'] = '%d' % startcfg.respawn_delay
            respawn_params = _get_respawn_params(startcfg.fullname, startcfg.params)
            if respawn_params['max'] > 0:
                new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
            if respawn_params['min_runtime'] > 0:
                new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
            cmd_type = "%s %s %s" % (settings.RESPAWN_SCRIPT, startcfg.prefix, cmd_type)
        else:
            cmd_type = "%s %s" % (startcfg.prefix, cmd_type)
        # check for masteruri
        masteruri = startcfg.masteruri
        if masteruri is None:
            masteruri = masteruri_from_ros()
        if masteruri is not None:
            if 'ROS_MASTER_URI' not in startcfg.env:
                new_env['ROS_MASTER_URI'] = masteruri
            # host in startcfg is a nmduri -> get host name
            ros_hostname = host.get_ros_hostname(masteruri, hostname)
            if ros_hostname:
                addr = socket.gethostbyname(ros_hostname)
                if addr in set(ip for ip in get_local_addresses()):
                    rospy.loginfo('set ROS_HOSTNAME to %s' % ros_hostname)
                    new_env['ROS_HOSTNAME'] = ros_hostname
            # load params to ROS master
            _load_parameters(masteruri, startcfg.params, startcfg.clear_params)
        # start
        cmd_str = utf8('%s %s %s' % (screen.get_cmd(startcfg.fullname, new_env, list(startcfg.env.keys())), cmd_type, ' '.join(args)))
        rospy.loginfo("%s (launch_file: '%s', masteruri: %s)" % (cmd_str, startcfg.config_path, masteruri))
        rospy.logdebug("environment while run node '%s': '%s'" % (cmd_str, new_env))
        SupervisedPopen(shlex.split(cmd_str), cwd=cwd, env=new_env, object_id="run_node_%s" % startcfg.fullname, description="Run [%s]%s" % (utf8(startcfg.package), utf8(startcfg.binary)))
    else:
        nmduri = startcfg.nmduri
        rospy.loginfo("remote run node '%s' at '%s'" % (nodename, nmduri))
        startcfg.params.update(_params_to_package_path(startcfg.params))
        startcfg.args = _args_to_package_path(startcfg.args)
        # run on a remote machine
        channel = remote.get_insecure_channel(nmduri)
        if channel is None:
            raise exceptions.StartException("Unknown launch manager url for host %s to start %s" % (host, startcfg.fullname))
        lm = LaunchStub(channel)
        lm.start_standalone_node(startcfg)


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


def _get_respawn_params(node, params, respawn_delay_value=0):
    result = {'max': 0, 'min_runtime': 0, 'delay': respawn_delay_value}
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


def _load_parameters(masteruri, params, clear_params):
    """
    Load parameters onto the parameter server
    """
    param_server = xmlrpcclient.ServerProxy(masteruri)
    p = None
    abs_paths = list()  # tuples of (parameter name, old value, new value)
    not_found_packages = list()  # packages names
    param_errors = []
    try:
        socket.setdefaulttimeout(6 + len(clear_params))
        # multi-call style xmlrpc
        param_server_multi = xmlrpcclient.MultiCall(param_server)

        # clear specified parameter namespaces
        # #2468 unify clear params to prevent error
        for p in clear_params:
            param_server_multi.deleteParam(rospy.get_name(), p)
        r = param_server_multi()
        for code, msg, _ in r:
            if code != 1 and not msg.find("is not set"):
                rospy.logwarn("Failed to clear parameter: %s", msg)
#          raise StartException("Failed to clear parameter: %s"%(msg))

        # multi-call objects are not reusable
        socket.setdefaulttimeout(6 + len(params))
        param_server_multi = xmlrpcclient.MultiCall(param_server)
        for pkey, pval in params.items():
            value = pval
            # resolve path elements
            if isstring(value) and (value.startswith('$')):
                value = interpret_path(value)
                rospy.logdebug("interpret parameter '%s' to '%s'" % (value, pval))
            # add parameter to the multicall
            param_server_multi.setParam(rospy.get_name(), pkey, value)
            test_ret = _test_value(pkey, value)
            if test_ret:
                param_errors.extend(test_ret)
        r = param_server_multi()
        for code, msg, _ in r:
            if code != 1:
                raise exceptions.StartException("Failed to set parameter: %s" % (msg))
    except roslaunch.core.RLException as e:
        raise exceptions.StartException(e)
    except rospkg.ResourceNotFound as rnf:
        raise exceptions.StartException("Failed to set parameter. ResourceNotFound: %s" % (rnf))
    except Exception as e:
        raise exceptions.StartException("Failed to set parameter. ROS Parameter Server "
                                        "reports: %s\n\n%s" % (e, '\n'.join(param_errors)))
    finally:
        socket.setdefaulttimeout(None)
    return abs_paths, not_found_packages


def _test_value(key, value):
    result = []
    if value is None:
        msg = "Invalid parameter value of '%s': '%s'" % (key, value)
        result.append(msg)
        rospy.logwarn(msg)
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
    if pname is not None:
        result = path.replace(ppath, '$(find %s)' % pname)
        rospy.logdebug("replace abs path '%s' by '%s'" % (path, result))
    return result


def _params_to_package_path(params):
    result = {}
    for name, value in params.items():
        if isstring(value):
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
                # load global parameter which has names equal to node names
                if name != n:
                    nodesparam = True
                    break
        if not nodesparam:
            result[name] = param.value
    return result

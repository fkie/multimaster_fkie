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
import rospy
import shlex
import socket
import time
import types
import xmlrpclib
import roslaunch

import exceptions
import screen
import host
from master_discovery_fkie.common import get_port, masteruri_from_ros
from .launch_stub import LaunchStub
from .common import get_cwd, utf8
from .supervised_popen import SupervisedPopen
from .startcfg import StartConfig
import remote

CACHED_PKG_PATH = dict()  # {host : {pkg: path}}
RESPAWN_SCRIPT = 'rosrun node_manager_fkie respawn'


def create_start_config(node, launchcfg, executable='', masteruri=None, loglevel=None):
    '''
    :return: Returns start configuration created from loaded launch file.
    :rtype: node_manager_daemon_fkie.startcfg.StartConfig
    '''
    n = launchcfg.get_node(node)
    if n is None:
        raise exceptions.StartException("Node '%s' not found in launch file %s" % (node, launchcfg.filename))
    result = StartConfig(n.package, n.type)
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
    result.prefix = prefix
    # set remapings
    result.remaps = {remap[0]: remap[1] for remap in n.remap_args}
    # set respawn parameter
    if n.respawn:
        result.respawn = n.respawn
        if n.respawn_delay > 0:
            result.respawn_delay = n.respawn_delay
        respawn_params = _get_respawn_params(rospy.names.ns_join(n.namespace, n.name), launchcfg.roscfg.params)
        result.respawn_max = respawn_params['max']
        result.respawn_min_runtime = respawn_params['min_runtime']
    # set log level
    if loglevel is not None:
        result.loglevel = loglevel
    # set masteruri and host config
    result.masteruri = masteruri
    result.host = launchcfg.host
    # set args
    result.args = n.args.split()
    # let cwd unchanged, it will be resolved on host
    result.cwd = n.cwd
    # add global parameter on start of first node of the launch
    if masteruri not in launchcfg.global_param_done:
        global_params = get_global_params(launchcfg.roscfg)
        result.params.update(global_params)
        rospy.loginfo("Register global parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in global_params.values()))
        launchcfg.global_param_done.append(masteruri)
    # add params and clear_params
    nodens = "%s%s%s" % (n.namespace, n.name, rospy.names.SEP)
    for param, value in launchcfg.roscfg.params.items():
        if param.startswith(nodens):
            result.params[param] = value
    for cparam in launchcfg.roscfg.clear_params:
        if cparam.startswith(nodens):
            result.clear_params.append(cparam)
        rospy.loginfo("Delete parameter:\n  %s", '\n  '.join(result.clear_params))
        rospy.loginfo("Register parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in result.params.values()))
    return result


def run_node(startcfg):
    '''
    Start a node local or on specified host using a ``StartConfig``
    :param startcfg: start configuration e.g. returned by ``create_start_config()``
    :type startcfg: node_manager_daemon_fkie.startcfg.StartConfig
    :raise exceptions.StartException: on errors
    :raise exceptions.BinarySelectionRequest: on multiple binaries
    :ref: ``node_manager_fkie.host.is_local()``
    '''
    if not startcfg.host or host.is_local(startcfg.host, wait=True):
        print "RUN LOCAL"
        # run on local host
        args = list(startcfg.args)
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
            except roslib.packages.ROSPkgException as e:
                # multiple nodes, invalid package
                raise exceptions.StartException(utf8(e))
            if isinstance(cmd, types.StringTypes):
                cmd = [cmd]
            if cmd is None or len(cmd) == 0:
                raise exceptions.StartException('%s in package [%s] not found!' % (startcfg.package, startcfg.binary))
            if len(cmd) > 1:
                # Open selection for executables
                err = 'Multiple executables with same name in package [%s]  found:' % startcfg.package
                raise exceptions.BinarySelectionRequest(cmd, err)
            else:
                cmd_type = cmd[0]
        cwd = get_cwd(startcfg.cwd, cmd_type)
        # set environment
        new_env = dict(os.environ)
        if startcfg.namespace:
            new_env['ROS_NAMESPACE'] = startcfg.namespace
        # handle respawn
        if startcfg.respawn:
            if startcfg.respawn_delay > 0:
                new_env['RESPAWN_DELAY'] = '%d' % startcfg.respawn_delay
            respawn_params = _get_respawn_params(startcfg.fullname, startcfg.params)
            if respawn_params['max'] > 0:
                new_env['RESPAWN_MAX'] = '%d' % respawn_params['max']
            if respawn_params['min_runtime'] > 0:
                new_env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']

            cmd_type = "%s %s %s" % (RESPAWN_SCRIPT, startcfg.prefix, cmd_type)
        else:
            cmd_type = "%s %s" % (startcfg.prefix, cmd_type)

        cmd_str = utf8('%s %s %s' % (screen.get_cmd(startcfg.fullname), cmd_type, ' '.join(args)))
        rospy.loginfo("Run: %s", cmd_str)
        # check for masteruri
        masteruri = startcfg.masteruri
        if masteruri is None:
            masteruri = masteruri_from_ros()
        if masteruri is not None:
            _prepare_ros_master(masteruri)
            new_env['ROS_MASTER_URI'] = masteruri
            ros_hostname = host.get_ros_hostname(masteruri)
            if ros_hostname:
                new_env['ROS_HOSTNAME'] = ros_hostname
            # load params to ROS master
            _load_parameters(masteruri, startcfg.params, startcfg.clear_params, False)
        # start
        SupervisedPopen(shlex.split(cmd_str), cwd=cwd, env=new_env, object_id="run_node_%s" % startcfg.fullname, description="Run [%s]%s" % (utf8(startcfg.package), utf8(startcfg.binary)))
    else:
        print "RUN REMOTE"
        # run on a remote machine
        channel = remote.get_insecure_channel(startcfg.host)
        if channel is None:
            raise exceptions.StartException("Unknown launch manager url for host %s to start %s" % (host, startcfg.fullname))
        lm = LaunchStub(channel)
        lm.start_standalone_node(startcfg)


# def run_node(cfg):
#     '''
#     Start the node with given name from the given configuration.
#     :param cfg: the configuration containing the start parameter
#     :type cfg: AdvLaunchCfg
#     :raise StartException: if the screen is not available on host.
#     :raise Exception: on errors while resolving host
#     :ref: node_manager_fkie.is_local()
#     '''
#     # 'print "RUN node", node, time.time()
#     n = cfg.roslaunch_config.get_node(cfg.node)
#     if n is None:
#         raise exceptions.StartException("Node '%s' not found!" % cfg.node)
# 
#     env = dict()
#     # set logging options
#     if cfg.logging is not None:
#         if not cfg.logging.is_default('console_format'):
#             env['ROSCONSOLE_FORMAT'] = '%s' % cfg.logging.console_format
#     if n.respawn:
#         # set the respawn environment variables
#         if n.respawn_delay > 0:
#             env['RESPAWN_DELAY'] = '%d' % n.respawn_delay
#         respawn_params = _get_respawn_params(rospy.names.ns_join(n.namespace, n.name), cfg.roslaunch_config.roscfg.params)
#         if respawn_params['max'] > 0:
#             env['RESPAWN_MAX'] = '%d' % respawn_params['max']
#         if respawn_params['min_runtime'] > 0:
#             env['RESPAWN_MIN_RUNTIME'] = '%d' % respawn_params['min_runtime']
#     prefix = n.launch_prefix if n.launch_prefix is not None else ''
#     if prefix.lower() == 'screen' or prefix.lower().find('screen ') != -1:
#         rospy.loginfo("SCREEN prefix removed before start!")
#         prefix = ''
#     args = ['__ns:=%s' % n.namespace.rstrip(rospy.names.SEP), '__name:=%s' % n.name]
# 
#     # add remaps
#     for remap in n.remap_args:
#         args.append("%s:=%s" % (remap[0], remap[1]))
# 
#     # get host of the node
#     host = cfg.roslaunch_config.hostname
#     env_loader = ''
#     if n.machine_name:
#         machine = cfg.roslaunch_config.roscfg.machines[n.machine_name]
#         if machine.address not in ['localhost', '127.0.0.1']:
#             host = machine.address
# #                 if runcfg.masteruri is None:
# #                     runcfg.masteruri = nm.nameres().masteruri(n.machine_name)
#     # TODO: env-loader support?
#     # if hasattr(machine, "env_loader") and machine.env_loader:
#     #     env_loader = machine.env_loader
#     # set the host to the given host
#     if cfg.force2host is not None:
#         host = cfg.force2host
# 
#     # set the ROS_MASTER_URI
#     if cfg.masteruri is None:
#         cfg.masteruri = masteruri_from_ros()
#         env['ROS_MASTER_URI'] = cfg.masteruri
#         ros_hostname = host.get_ros_hostname(cfg.masteruri)
#         if ros_hostname:
#             env['ROS_HOSTNAME'] = ros_hostname
#     # add the namespace environment parameter to handle some cases,
#     # e.g. rqt_cpp plugins
#     if n.namespace:
#         env['ROS_NAMESPACE'] = n.namespace
# 
#     abs_paths = list()  # tuples of (parameter name, old value, new value)
#     not_found_packages = list()  # package names
#     # set the global parameter
#     if cfg.masteruri is not None and cfg.masteruri not in cfg.roslaunch_config.global_param_done:
#         global_node_names = get_global_params(cfg.roslaunch_config.roscfg)
#         rospy.loginfo("Register global parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in global_node_names.values()))
#         abs_paths[len(abs_paths):], not_found_packages[len(not_found_packages):] = _load_parameters(cfg.masteruri, global_node_names, [], cfg.user, cfg.pw, cfg.auto_pw_request)
#         cfg.roslaunch_config.global_param_done.append(cfg.masteruri)
# 
#     # add params
#     params = dict()
#     clear_params = []
#     if cfg.masteruri is not None:
#         nodens = "%s%s%s" % (n.namespace, n.name, rospy.names.SEP)
#         for param, value in cfg.roslaunch_config.roscfg.params.items():
#             if param.startswith(nodens):
#                 params[param] = value
#         for cparam in cfg.roslaunch_config.roscfg.clear_params:
#             if cparam.startswith(nodens):
#                 clear_params.append(cparam)
#         rospy.loginfo("Delete parameter:\n  %s", '\n  '.join(clear_params))
#         rospy.loginfo("Register parameter:\n  %s", '\n  '.join("%s%s" % (utf8(v)[:80], '...' if len(utf8(v)) > 80 else'') for v in params.values()))
#         abs_paths[len(abs_paths):], not_found_packages[len(not_found_packages):] = _load_parameters(cfg.masteruri, params, clear_params, cfg.user, cfg.pw, cfg.auto_pw_request)
#     # 'print "RUN prepared", node, time.time()
# 
#     if host.is_local(host, wait=True):
#         screen.test_screen()
# 
#         if cfg.executable:
#             cmd_type = cfg.executable
#         else:
#             try:
#                 cmd = roslib.packages.find_node(n.package, n.type)
#             except (Exception, roslib.packages.ROSPkgException) as starterr:
#                 # multiple nodes, invalid package
#                 raise exceptions.StartException("Can't find resource: %s" % starterr)
#             # handle diferent result types str or array of string
#             if isinstance(cmd, types.StringTypes):
#                 cmd = [cmd]
#             cmd_type = ''
#             if cmd is None or len(cmd) == 0:
#                 raise exceptions.StartException("%s in package [%s] not found!\n\nThe package "
#                                                 "was created?\nIs the binary executable?\n" % (n.type, n.package))
#             if len(cmd) > 1:
#                 raise exceptions.BinarySelectionRequest(cmd, 'Multiple executables')
#             else:
#                 cmd_type = cmd[0]
#         # determine the current working path, Default: the package of the node
#         cwd = get_cwd(n.cwd, cmd_type)
#         _prepare_ros_master(cfg.masteruri)
#         node_cmd = [RESPAWN_SCRIPT if n.respawn else '', prefix, cmd_type]
#         cmd_args = [screen.get_cmd(cfg.node)]
#         cmd_args[len(cmd_args):] = node_cmd
#         cmd_args.append(utf8(n.args))
#         cmd_args[len(cmd_args):] = args
#         rospy.loginfo("RUN: %s", ' '.join(cmd_args))
#         # set logging options
# #             if runcfg.logging is not None:
# #                 if not runcfg.logging.is_default('loglevel'):
# #                     env.append(('ROSCONSOLE_CONFIG_FILE', nm.settings().rosconsole_cfg_file(n.package)))
#         new_env = dict(os.environ)
#         new_env.update(env)
#         SupervisedPopen(shlex.split(utf8(' '.join(cmd_args))), cwd=cwd,
#                         env=new_env, object_id="Run node", description="Run node "
#                         "[%s]%s" % (utf8(n.package), utf8(n.type)))
# #            nm.filewatcher().add_binary(cmd_type, runcfg.node, runcfg.masteruri, runcfg.roslaunch_config.Filename)
#     else:
#         channel = remote.get_insecure_channel(host)
#         if channel is None:
#             raise exceptions.StartException("Unknown launch manager url for host %s to start %s" % (host, n.name))
#         lm = LaunchManagerClient(channel)
#         respawn_max = 0
#         respawn_min_runtime = 0
#         if n.respawn:
#             respawn_max = respawn_params['max']
#             respawn_min_runtime = respawn_min_runtime['max']
#         # TODO add global parameter
#         lm.start_standalone_node(n.package, n.type, name=cfg.node, cwd=n.cwd, env=env, params=params, clear_params=clear_params, remaps={remap[0]: remap[1] for remap in n.remap_args},
#                                  args=n.args.split(), prefix=prefix, masteruri=cfg.masteruri, loglevel=4,
#                                  respawn=n.respawn, respawn_delay=n.respawn_delay, respawn_max=respawn_max, respawn_min_runtime=respawn_min_runtime)


def _get_respawn_params(node, params):
    result = {'max': 0, 'min_runtime': 0}
    respawn_max = rospy.names.ns_join(node, 'respawn/max')
    respawn_min_runtime = rospy.names.ns_join(node, 'respawn/min_runtime')
    try:
        result['max'] = int(params[respawn_max].value)
    except Exception:
        pass
    try:
        result['min_runtime'] = int(params[respawn_min_runtime].value)
    except Exception:
        pass
    return result


def _load_parameters(masteruri, params, clear_params, replace_abs=False, user=None, pw=None, auto_pw_request=None):
    """
    Load parameters onto the parameter server
    """
    param_server = xmlrpclib.ServerProxy(masteruri)
    p = None
    abs_paths = list()  # tuples of (parameter name, old value, new value)
    not_found_packages = list()  # packages names
    param_errors = []
    try:
        socket.setdefaulttimeout(6 + len(clear_params))
        # multi-call style xmlrpc
        param_server_multi = xmlrpclib.MultiCall(param_server)

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
        param_server_multi = xmlrpclib.MultiCall(param_server)
        address = host.get_hostname(masteruri)
        for p in params.itervalues():
            value = p.value
            if replace_abs:
                # suppressing this as it causes too much spam
                value, is_abs_path, found, package = _resolve_abs_paths(p.value, address, user, pw, auto_pw_request)
                if is_abs_path:
                    abs_paths.append((p.key, p.value, value))
                    if not found and package:
                        not_found_packages.append(package)
            # add parameter to the multicall
            param_server_multi.setParam(rospy.get_name(), p.key, value)
            test_ret = _test_value(p.key, value)
            if test_ret:
                param_errors.extend(test_ret)
        r = param_server_multi()
        for code, msg, _ in r:
            if code != 1:
                raise exceptions.StartException("Failed to set parameter: %s" % (msg))
    except roslaunch.core.RLException, e:
        raise exceptions.StartException(e)
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


def _resolve_abs_paths(value, host, user, pw, auto_pw_request):
    '''
    Replaces the local absolute path by remote absolute path. Only valid ROS
    package paths are resolved.
    :return: value, is absolute path, remote package found (ignore it on local host or if is not absolute path!), package name (if absolute path and remote package NOT found)
    '''
    if isinstance(value, types.StringTypes) and value.startswith('/') and (os.path.isfile(value) or os.path.isdir(value)):
        if host.is_local(host):
            return value, True, True, ''
        else:
            pass
#            path = os.path.dirname(value) if os.path.isfile(value) else value
#            package, package_path = package_name(path)
#             if package:
#                 _, stdout, _, ok = nm.ssh().ssh_exec(host, ['rospack', 'find', package], user, pw, auto_pw_request, close_stdin=True, close_stderr=True)
#                 output = stdout.read()
#                 stdout.close()
#                 if ok:
#                     if output:
#                         value.replace(package_path, output)
#                         return value.replace(package_path, output.strip()), True, True, package
#                     else:
#                         # package on remote host not found!
#                         # TODO add error message
#                         #      error = stderr.read()
#                         pass
            return value, True, False, ''
    else:
        return value, False, False, ''


# def run_node_wo_config(self, package, binary, name='', cwd='', env={}, params={}, clear_params=[],
#                        remaps={}, args=[], prefix='', masteruri='', loglevel=4,
#                        respawn=False, respawn_delay=30, respawn_max=0, respawn_min_runtime=0):
# #def run_node_wo_config(host, package, binary, name, args=[], masteruri=None, auto_pw_request=False, user=None, pw=None):
#     '''
#     Start a node with using a launch configuration.
#     :param str host: the host or ip to run the node
#     :param str package: the ROS package containing the binary
#     :param str binary: the binary of the node to execute
#     :param str name: the ROS name of the node (with name space)
#     :param args: the list with arguments passed to the binary
#     :type args: [str, ...]
#     :param bool auto_pw_request: opens question dialog directly, use True only if the method is called from the main GUI thread
#     :raise Exception: on errors while resolving host
#     :ref: ``node_manager_fkie.is_local()``
#     '''
#     # create the name with namespace
#     args2 = list(args)
#     fullname = roslib.names.ns_join(roslib.names.SEP, name)
#     namespace = ''
#     for a in args:
#         if a.startswith('__ns:='):
#             namespace = a.replace('__ns:=', '')
#             fullname = roslib.names.ns_join(namespace, name)
#     if name:
#         args2.append('__ns:=%s' % roslib.names.namespace(name))
#         args2.append('__name:=%s' % os.path.basename(name))
#     # run on local host
#     if host.is_local(host, wait=True):
#         try:
#             cmd = roslib.packages.find_node(package, binary)
#         except roslib.packages.ROSPkgException as e:
#             # multiple nodes, invalid package
#             raise exceptions.StartException(utf8(e))
#         if isinstance(cmd, types.StringTypes):
#             cmd = [cmd]
#         cmd_type = ''
#         if cmd is None or len(cmd) == 0:
#             raise exceptions.StartException('%s in package [%s] not found!' % (binary, package))
#         if len(cmd) > 1:
#             # Open selection for executables
#             err = 'Multiple executables with same name in package [%s]  found:' % package
#             raise exceptions.BinarySelectionRequest(cmd, err)
#         else:
#             cmd_type = cmd[0]
#         cmd_str = utf8('%s %s %s' % (screen.get_cmd(fullname), cmd_type, ' '.join(args2)))
#         rospy.loginfo("Run without config: %s", cmd_str)
#         new_env = dict(os.environ)
#         if namespace:
#             new_env['ROS_NAMESPACE'] = namespace
#         if masteruri is not None:
#             _prepare_ros_master(masteruri)
#             new_env['ROS_MASTER_URI'] = masteruri
#             ros_hostname = host.get_ros_hostname(masteruri)
#             if ros_hostname:
#                 new_env['ROS_HOSTNAME'] = ros_hostname
#         SupervisedPopen(shlex.split(cmd_str), env=new_env, object_id="Run without config", description="Run without config [%s]%s" % (utf8(package), utf8(binary)))
#     else:
#         # run on a remote machine
#         channel = remote.get_insecure_channel(host)
#         if channel is None:
#             raise exceptions.StartException("Unknown launch manager url for host %s to start %s" % (host, n.name))
#         lm = LaunchManagerClient(channel)
#         respawn_max = 0
#         respawn_min_runtime = 0
#         if n.respawn:
#             respawn_max = respawn_params['max']
#             respawn_min_runtime = respawn_min_runtime['max']
#         # TODO add global parameter
#         lm.start_standalone_node(n.package, n.type, name=cfg.node, cwd=n.cwd, env=env, params=params, clear_params=clear_params, remaps={remap[0]: remap[1] for remap in n.remap_args},
#                                  args=n.args.split(), prefix=prefix, masteruri=cfg.masteruri, loglevel=4,
#                                  respawn=n.respawn, respawn_delay=n.respawn_delay, respawn_max=respawn_max, respawn_min_runtime=respawn_min_runtime)


def _prepare_ros_master(masteruri):
    if masteruri is None:
        masteruri = masteruri_from_ros()
    # start roscore, if needed
    try:
        if not os.path.isdir(screen.LOG_PATH):
            os.makedirs(screen.LOG_PATH)
        socket.setdefaulttimeout(3)
        master = xmlrpclib.ServerProxy(masteruri)
        master.getUri(rospy.get_name())
        # restart ROSCORE on different masteruri?, not now...
#      master_uri = master.getUri(rospy.get_name())
#      if masteruri != master_uri[2]:
#        # kill the local roscore...
#        raise
    except Exception:
        # run a roscore
        master_host = host.get_hostname(masteruri)
        if host.is_local(master_host, True):
            master_port = get_port(masteruri)
            new_env = dict(os.environ)
            new_env['ROS_MASTER_URI'] = masteruri
            ros_hostname = host.get_ros_hostname(masteruri)
            if ros_hostname:
                new_env['ROS_HOSTNAME'] = ros_hostname
            cmd_args = '%s roscore --port %d' % (screen.get_cmd('/roscore--%d' % master_port), master_port)
            for n in [1, 2, 3, 4]:
                try:
                    if n == 1:
                        print("Launch ROS Master in screen  ...")
                        SupervisedPopen(shlex.split(cmd_args), env=new_env, object_id="ROSCORE", description="Start roscore")
                    elif n == 2:
                        print("ROS Master takes too long for start, wait for next 10 sec ...")
                    elif n == 3:
                        print("A really slow start, wait for last 10 sec ...")
                    # wait for roscore to avoid connection problems while init_node
                    result = -1
                    count = 1
                    while result == -1 and count < 11:
                        try:
                            master = xmlrpclib.ServerProxy(masteruri)
                            result, _, _ = master.getUri(rospy.get_name())  # _:=uri, msg
                            return
                        except Exception:
                            time.sleep(1)
                            count += 1
                    if n == 4 and count >= 11:
                        raise exceptions.StartException('Cannot connect to ROS-Master: %s\n--> please run "roscore" manually!' % utf8(masteruri))
                except Exception as e:
                    raise Exception("Error while call '%s': %s" % (cmd_args, utf8(e)))
        else:
            raise Exception("ROS master '%s' is not reachable" % masteruri)
    finally:
        socket.setdefaulttimeout(None)


def get_global_params(roscfg):
    '''
    Return the parameter of the configuration file, which are not associated with
    any nodes in the configuration.
    :param roscfg: the launch configuration
    :type roscfg: roslaunch.ROSLaunchConfig<http://docs.ros.org/kinetic/api/roslaunch/html/>
    :return: the list with names of the global parameter
    :rtype: dict(param:value, ...)
    '''
    result = dict()
    nodes = []
    for item in roscfg.resolved_node_names:
        nodes.append(item)
    for param, value in roscfg.params.items():
        nodesparam = False
        for n in nodes:
            if param.startswith(n):
                nodesparam = True
                break
        if not nodesparam:
            result[param] = value
    return result

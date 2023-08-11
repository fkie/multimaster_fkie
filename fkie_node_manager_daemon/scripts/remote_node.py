#!/usr/bin/env python3

import os
import psutil
import re
import shlex
import shutil
import socket
import subprocess
import sys
import time
import argparse
from typing import List

from fkie_multimaster_pylib.defines import LOG_PATH
from fkie_multimaster_pylib.defines import RESPAWN_SCRIPT
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.system import host as nmdhost
from fkie_multimaster_pylib.system import screen
import fkie_multimaster_pylib.names as names

if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == "1":
    from fkie_multimaster_pylib.system import ros1_masteruri
    from fkie_node_manager_daemon.strings import isstring
    from rosgraph.network import get_local_addresses
    try:
        from fkie_node_manager import get_ros_home
        from fkie_node_manager import Settings
        from fkie_node_manager import StartHandler
        from fkie_node_manager import StartException
    except Exception:
        from fkie_node_manager.reduced_nm import get_ros_home
        from fkie_node_manager.reduced_nm import Settings
        from fkie_node_manager.reduced_nm import StartHandler
        from fkie_node_manager.reduced_nm import StartException


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Start nodes remotely using the host ROS configuration',
        epilog="Fraunhofer FKIE 2022")

    parser.add_argument('--name', default=None,
                        help='The name of the node (with namespace) or script')
    parser.add_argument('--set_name', type=str2bool, nargs='?',
                        const=True, default=True,
                        help='Set ros name using arguments if True')
    parser.add_argument('--command', default=None,
                        help='Generic command to execute')
    parser.add_argument('--node_type', default=None,
                        help='Type of the node to run')
    parser.add_argument('--package', default=None,
                        help='Package containing the node. If no name specified returns the package path or raise an exception, if the package was not found.')
    parser.add_argument('--respawn', default=None, action='store_true',
                        help='respawn the node, if it terminate unexpectedly')
    parser.add_argument('--show_screen_log', default=None,
                        help='Shows the screen log of the given node')
    parser.add_argument('--tail_screen_log', default=None,
                        help='Tail the screen log of the given node')
    parser.add_argument('--show_ros_log', default=None,
                        help='Shows the ros log of the given node')
    parser.add_argument('--ros_log_path', default=None,
                        help='request for the path of the ros logs')
    parser.add_argument('--ros_logs', default=None,
                        help='request for the list of available log nodes')
    parser.add_argument('--delete_logs', default=None,
                        help='Delete the log files of the given node')
    parser.add_argument('--prefix', default="",
                        help='Prefix used to run a node')
    parser.add_argument('--pidkill', default=None,
                        help='kill the process with given pid')
    parser.add_argument('--masteruri', default=None,
                        help='the ROS MASTER URI for started node')
    parser.add_argument('--force', default=None, action='store_false',
                        help='Force command even if a process with the same name is running')
    parser.add_argument('--pre_check_binary', type=str2bool, nargs='?',
                        const=True, default=False,
                        help='Check if the binary exists and raise an error  if not (before run in screen). Only for commands!')

    args, additional_args = parser.parse_known_args()

    argumentsStr = "\n"
    argumentsStr += f'  name: {args.name}\n' if args.name is not None else ""
    argumentsStr += f'  set_name: {args.set_name}\n'
    argumentsStr += f'  command: {args.command}\n' if args.command is not None else ""
    argumentsStr += f'  node_type: {args.node_type}\n' if args.node_type is not None else ""
    argumentsStr += f'  package: {args.package}\n' if args.package is not None else ""
    argumentsStr += f'  respawn: {args.respawn}\n' if args.respawn is not None else ""
    argumentsStr += f'  show_screen_log: {args.show_screen_log}\n' if args.show_screen_log is not None else ""
    argumentsStr += f'  tail_screen_log: {args.tail_screen_log}\n' if args.tail_screen_log is not None else ""
    argumentsStr += f'  show_ros_log: {args.show_ros_log}\n' if args.show_ros_log is not None else ""
    argumentsStr += f'  ros_log_path: {args.ros_log_path}\n' if args.ros_log_path is not None else ""
    argumentsStr += f'  ros_logs: {args.ros_logs}\n' if args.ros_logs is not None else ""
    argumentsStr += f'  delete_logs: {args.delete_logs}\n' if args.delete_logs is not None else ""
    argumentsStr += f'  prefix: {args.prefix}\n' if len(
        args.prefix) > 0 else ""
    argumentsStr += f'  pidkill: {args.pidkill}\n' if args.pidkill is not None else ""
    argumentsStr += f'  masteruri: {args.masteruri}\n' if args.masteruri is not None else ""
    argumentsStr += f'  force: {args.force}\n' if args.force is not None else ""
    argumentsStr += f'  pre_check_binary: {args.pre_check_binary}\n'

    Log.info("[remote_node.py] Starting Remote script", "\nArguments:", argumentsStr,
             "\nAdditional Arguments:\n", additional_args, "\n")

    return parser, args, additional_args


def find_process_by_name(command: str, package: str, additional_args: List[str]):
    "Return a list of processes matching 'package', 'command' and 'additional_args'. Ignores self node."
    self_name = psutil.Process().name()
    pkg = package if package is not None else ''
    cmd_args = f'{" ".join(additional_args)}'.strip()
    if cmd_args:
        cmd_args = f'\s*{cmd_args}'.replace('[', '\[').replace(']', '\]')
    # try to compare the process with regex
    cmd_reg = re.compile(f'{pkg}.*[\s\/]{command}{cmd_args}\Z')

    result = []
    for p in psutil.process_iter(["pid", "name", "exe", "cmdline"]):
        # ignore self node
        if self_name in p.info['name']:
            continue
        cmdline = ' '.join(p.info['cmdline'])
        if 'remote_node.py' in cmdline:
            continue

        for _ in cmd_reg.finditer(cmdline):
            result.append(p)
            break
    if result:
        Log.info(
            f'Found running processes by pattern "{cmd_reg.pattern}" [{len(result)}]:')
        for ps in result:
            Log.info(f"  {ps.info['pid']}  ", ' '.join(ps.info['cmdline']))
    return result


def getCwdArg(arg, argv):
    for a in argv:
        key, sep, value = a.partition(':=')
        if sep and arg == key:
            return value
    return None


def rosconsole_cfg_file(package, loglevel='INFO'):
    result = os.path.join(LOG_PATH, '%s.rosconsole.config' % package)
    with open(result, 'w') as cfg_file:
        cfg_file.write('log4j.logger.ros=%s\n' % loglevel)
        cfg_file.write('log4j.logger.ros.roscpp=INFO\n')
        cfg_file.write('log4j.logger.ros.roscpp.superdebug=WARN\n')
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


def run_ROS1_node(package: str, executable: str, name: str, args: List[str], prefix='', respawn=False, masteruri=None, loglevel='', set_name=True):
    '''
    Runs a ROS1 node. Starts a roscore if needed.
    '''
    import roslib

    if not masteruri:
        masteruri = ros1_masteruri.from_ros()

    # start roscore, if needed
    StartHandler._prepareROSMaster(masteruri)

    # start node
    try:
        cmd = roslib.packages.find_node(package, executable)
    except roslib.packages.ROSPkgException as e:
        # multiple nodes, invalid package
        raise StartException(str(e))

    # handle different result types str or array of string (electric / fuerte)
    if isstring(cmd):
        cmd = [cmd]
    if cmd is None or len(cmd) == 0:
        raise StartException(' '.join(
            [executable, 'in package [', package, '] not found!\n\nThe package was created?\nIs the binary executable?\n']))

    # create string for node parameter. Set arguments with spaces into "'".
    cmd = remove_src_binary(cmd)
    node_params = ' '.join(''.join(["'", a, "'"]) if a.find(
        ' ') > -1 else a for a in args)

    # get namespace and basename from name
    arg_ns = names.namespace(
        name, with_sep_suffix=False, raise_err_on_none=False)
    arg_name = names.basename(name)

    arg_name_list = []
    if set_name:
        arg_name_list = [f'__name:={arg_name}', f'__ns:={arg_ns}']
    cmd_args = [screen.get_cmd(node=arg_name if arg_name else executable, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', prefix, cmd[0],
                *arg_name_list, node_params]
    Log.info('run on remote host:', ' '.join(cmd_args))

    # determine the current working path
    arg_cwd = getCwdArg('__cwd', args)
    cwd = get_ros_home()
    if not (arg_cwd is None):
        if arg_cwd == 'ROS_HOME':
            cwd = get_ros_home()
        elif arg_cwd == 'node':
            cwd = os.path.dirname(cmd[0])

    # set the masteruri to launch with other one master
    new_env = dict(os.environ)
    new_env['ROS_MASTER_URI'] = masteruri
    ros_hostname = nmdhost.get_ros_hostname(masteruri)
    if ros_hostname:
        addr = socket.gethostbyname(ros_hostname)
        if addr in set(ip for ip in get_local_addresses()):
            new_env['ROS_HOSTNAME'] = ros_hostname
    if loglevel:
        new_env['ROSCONSOLE_CONFIG_FILE'] = rosconsole_cfg_file(package)
    subprocess.Popen(shlex.split(str(' '.join(cmd_args))),
                     cwd=cwd, env=new_env)
    if len(cmd) > 1:
        Log.warn(
            'Multiple executables were found! The first one was started! Executables:\n%s', str(cmd))


def run_ROS2_node(package: str, executable: str, name: str, args: List[str], prefix='', respawn=False, set_name=True):
    '''
    Runs a ROS2 node
    '''

    # get namespace and basename from name
    arg_ns = names.namespace(
        name, with_sep_suffix=False, raise_err_on_none=False)
    arg_name = names.basename(name)

    arg_name_list = f'--ros-args -r __name:={arg_name} -r __ns:={arg_ns}' if set_name else ''
    cmd = f'ros2 run {package} {executable} {arg_name_list}'
    node_params = ' '.join(''.join(["'", a, "'"]) if a.find(
        ' ') > -1 else a for a in args[1:])
    if not set_name and node_params:
        node_params = f'--ros-args {node_params}'
    cmd_args = [screen.get_cmd(node=arg_name if arg_name else executable, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', prefix, cmd, node_params]

    screen_command = ' '.join(cmd_args)

    Log.info('run on remote host:', screen_command)
    subprocess.Popen(shlex.split(screen_command), env=dict(os.environ))


def run_command(name: str, command: str, additional_args: List[str], respawn: bool = False, pre_check_binary: bool = False):
    '''
    Runs a command remotely
    '''
    if pre_check_binary:
        if not shutil.which(command):
            raise Exception(
                f"Cannot find '{command}': No such file or directory")
    # get namespace if given
    arg_ns = getCwdArg('__ns', additional_args)

    cmd_args = [screen.get_cmd(node=name, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', "", command]

    screen_command = ' '.join(cmd_args)
    screen_command += ' ' + ' '.join(additional_args)

    Log.info('run on remote host:', screen_command)
    subprocess.Popen(shlex.split(screen_command), env=dict(os.environ))


def main(argv=sys.argv) -> int:
    parser, args, additional_args = parse_arguments()

    command = args.node_type
    if args.node_type is None:
        command = args.command
    running_processes = find_process_by_name(
        command, args.package, additional_args)
    if len(running_processes) > 0:
        if args.name:
            Log.warn(
                f'A process with the same name [{args.name}] is already running.',
                'Skipping command because [force] is disable' if not args.force else "")
        else:
            Log.warn(
                f'A process of same package/executable [{args.package}/{args.node_type}] is already running.',
                'Skipping command because [force] is disable' if not args.force else "")
        if not args.force:
            return 0

    try:
        print_help = True
        if args.command:
            run_command(args.name, args.command,
                        additional_args, args.respawn, args.pre_check_binary)
            return 0

        if args.show_screen_log:
            logfile = screen.get_logfile(node=args.show_screen_log)
            if not os.path.isfile(logfile):
                raise Exception('screen logfile not found for: %s' %
                                args.show_screen_log)
            cmd = ' '.join([Settings.LOG_VIEWER, str(logfile)])
            Log.info(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False

        if args.tail_screen_log:
            logfile = screen.get_logfile(node=args.tail_screen_log)
            if not os.path.isfile(logfile):
                raise Exception('screen logfile not found for: %s' %
                                args.tail_screen_log)
            cmd = ' '.join(['tail', '-f', '-n', '25', str(logfile)])
            Log.info(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False

        elif args.show_ros_log:
            logfile = screen.get_ros_logfile(node=args.show_ros_log)
            if not os.path.isfile(logfile):
                raise Exception('ros logfile not found for: %s' %
                                args.show_ros_log)
            cmd = ' '.join([Settings.LOG_VIEWER, str(logfile)])
            Log.info(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False

        elif args.ros_log_path:
            if args.ros_log_path == '[]':
                Log.info(get_ros_home())
            else:
                Log.info(screen.get_logfile(node=args.ros_log_path))
            print_help = False

        elif args.delete_logs:
            logfile = screen.get_logfile(node=args.delete_logs)
            pidfile = screen.get_pid_file(node=args.delete_logs)
            roslog = screen.get_ros_logfile(node=args.delete_logs)
            if os.path.isfile(logfile):
                os.remove(logfile)
            if os.path.isfile(pidfile):
                os.remove(pidfile)
            if os.path.isfile(roslog):
                os.remove(roslog)
            print_help = False

        elif args.node_type and args.package:
            if os.environ['ROS_VERSION'] == "1":
                run_ROS1_node(args.package, args.node_type, args.name,
                              additional_args, args.prefix, args.respawn, args.masteruri, set_name=args.set_name)
            elif os.environ['ROS_VERSION'] == "2":
                run_ROS2_node(args.package, args.node_type, args.name,
                              additional_args, args.prefix, args.respawn, set_name=args.set_name)
            else:
                Log.error(f'Invalid ROS Version: {os.environ["ROS_VERSION"]}')

            print_help = False

        elif args.pidkill:
            import signal
            os.kill(int(args.pidkill), signal.SIGKILL)
            print_help = False

        if print_help:
            parser.print_help()
            time.sleep(3)

    except Exception as e:
        Log.error(f'Error while execute command: {e}')
        print(f'Error while execute command: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    result = main()
    exit(result)

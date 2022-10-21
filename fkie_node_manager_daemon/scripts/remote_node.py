#!/usr/bin/env python3

import os
import psutil
import shlex
import socket
import subprocess
import sys
import time
import argparse
from typing import List

from fkie_multimaster_msgs.defines import LOG_PATH
from fkie_multimaster_msgs.defines import RESPAWN_SCRIPT
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system import host as nmdhost
from fkie_multimaster_msgs.system import screen

if os.environ['ROS_VERSION'] == "1":
    from fkie_master_discovery.common import masteruri_from_ros
    from fkie_node_manager_daemon.common import isstring
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


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Start nodes remotely using the host ROS configuration',
        epilog="Fraunhofer FKIE 2022")

    parser.add_argument('--name', default=None,
                        help='The name of the node (with namespace) or script')
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

    args, additional_args = parser.parse_known_args()

    argumentsStr = "\n"
    argumentsStr += f'  name: {args.name}\n' if args.name is not None else ""
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

    Log.info("[remote_node.py] Starting Remote script", "\nArguments:", argumentsStr,
             "\nAdditional Arguments:\n", additional_args, "\n")

    return parser, args, additional_args


def find_process_by_name(name: str):
    "Return a list of processes matching 'name'. Ignores self node."
    ls = []
    self_name = psutil.Process().name()
    for p in psutil.process_iter(["name", "exe", "cmdline"]):
        # ignore self node
        if self_name in p.info['name']:
            continue

        if name in p.info['name']:
            ls.append(p)
            continue

        if p.info['exe'] and name in os.path.basename(p.info['exe']):
            ls.append(p)
            continue

        if p.info['cmdline']:
            for cl in p.info['cmdline']:
                # validate self name also in arguments
                # and process is started in a screen.
                # In other case, own starting process will be added.
                if name in cl and self_name not in cl and 'screen' in p.info['name']:
                    ls.append(p)
                    continue
    return ls


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


def run_ROS1_node(package: str, executable: str, name: str, args: List[str], prefix='', respawn=False, masteruri=None, loglevel=''):
    '''
    Runs a ROS1 node. Starts a roscore if needed.
    '''
    import roslib

    if not masteruri:
        masteruri = masteruri_from_ros()

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

    # get namespace if given
    arg_ns = getCwdArg('__ns', args)

    cmd_args = [screen.get_cmd(node=name, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', prefix, cmd[0], node_params]
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


def run_ROS2_node(package: str, executable: str, name: str, args: List[str], prefix='', respawn=False):
    '''
    Runs a ROS2 node
    '''

    cmd = f'ros2 run {package} {executable}' # --ros-args --remap __name:={name} <-- already added by GUI
    node_params = ' '.join(''.join(["'", a, "'"]) if a.find(
        ' ') > -1 else a for a in args[1:])

    # get namespace if given
    arg_ns = getCwdArg('__ns', args)

    cmd_args = [screen.get_cmd(node=name, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', prefix, cmd, node_params]

    screen_command = ' '.join(cmd_args)

    Log.info('run on remote host:', screen_command)
    subprocess.Popen(shlex.split(screen_command), env=dict(os.environ))


def run_command(name: str, command: str, additional_args: List[str], respawn=False):
    '''
    Runs a command remotely
    '''
    # get namespace if given
    arg_ns = getCwdArg('__ns', additional_args)

    cmd_args = [screen.get_cmd(node=name, namespace=arg_ns),
                RESPAWN_SCRIPT if respawn is not None else '', "", command]

    screen_command = ' '.join(cmd_args)
    screen_command += ' ' + ' '.join(additional_args)

    Log.info('run on remote host:', screen_command)
    subprocess.Popen(shlex.split(screen_command), env=dict(os.environ))


def main(argv=sys.argv):
    parser, args, additional_args = parse_arguments()

    running_processes = find_process_by_name(args.name)
    if len(running_processes) > 0:
        Log.warn(
            f'A process with the same name [{args.name}] is already running.',
            'Skipping command because [force] is disable' if not args.force else "")

        if not args.force:
            return

    try:
        print_help = True
        if args.command:
            run_command(args.name, args.command,
                        additional_args, args.respawn)
            return

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

        elif args.node_type and args.package and args.name:
            if os.environ['ROS_VERSION'] == "1":
                run_ROS1_node(args.package, args.node_type, args.name,
                              additional_args, args.prefix, args.respawn, args.masteruri)
            elif os.environ['ROS_VERSION'] == "2":
                run_ROS2_node(args.package, args.node_type, args.name,
                              additional_args, args.prefix, args.respawn)
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
        import traceback
        Log.error(traceback.format_exc())


if __name__ == '__main__':
    main()

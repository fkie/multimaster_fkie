#!/usr/bin/env python


import os
import shlex
import socket
import subprocess
import sys
import time

import roslib
import rospy
from rosgraph.network import get_local_addresses

from fkie_master_discovery.common import masteruri_from_ros
from fkie_master_discovery.udp import DiscoverSocket
from fkie_node_manager_daemon import host as nmdhost
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon.common import isstring
from fkie_node_manager_daemon.settings import RESPAWN_SCRIPT
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


def _get_optparse():
    '''
    This method is only to print help on options parse errors.
    '''
    import optparse

    parser = optparse.OptionParser(usage='usage: %prog [options] [args]')
    parser.add_option('--show_screen_log', metavar='show_screen_log', default='',
                      help='Shows the screen log of the given node')
    parser.add_option('--tail_screen_log', metavar='tail_screen_log', default='',
                      help='Tail the screen log of the given node')
    parser.add_option('--show_ros_log', metavar='show_ros_log', default='',
                      help='Shows the ros log of the given node')
    parser.add_option('--ros_log_path', metavar='ros_log_path', default=-1,
                      help='request for the path of the ros logs')
    parser.add_option('--ros_logs', metavar='ros_logs', default=-1,
                      help='request for the list of available log nodes')
    parser.add_option('--delete_logs', metavar='delete_logs', default='',
                      help='Delete the log files of the given node')
    parser.add_option('--node', metavar='node', default='',
                      help='Type of the node to run')
    parser.add_option('--node_name', metavar='node_name', default='',
                      help='The name of the node (with namespace)')
    parser.add_option('--package', metavar='package', default='',
                      help='Package containing the node. If no node_name specified returns the package path or raise an exception, if the package was not found.')
    parser.add_option('--prefix', metavar='prefix', default='',
                      help='Prefix used to run a node')
    parser.add_option('--pidkill', metavar='pidkill', default=-1,
                      help='kill the process with given pid')
    parser.add_option('--node_respawn', metavar='node_respawn', default=-1,
                      help='respawn the node, if it terminate unexpectedly')
    parser.add_option('--masteruri', metavar='masteruri', default=-1,
                      help='the ROS MASTER URI for started node')
#  parser.add_option('--has_log', action="store_true", default=False,
#                   help='Tests whether the screen log file is available')
    return parser


def parse_options(args):
    result = {'show_screen_log': '',
              'tail_screen_log': '',
              'show_ros_log': '',
              'ros_log_path': '',
              'ros_logs': '',
              'delete_logs': '',
              'node_type': '',
              'node_name': '',
              'package': '',
              'prefix': '',
              'pidkill': '',
              'node_respawn': '',
              'masteruri': '',
              'loglevel': ''}
    options = [''.join(['--', v]) for v in result.keys()]
    argv = []
    arg_added = False
#  print 'options:', options
#  print 'ENUMERATE:',enumerate(options)
    for i, a in enumerate(args):
        if a in options:
            if i + 1 < len(args) and len(args) > i + 1 and args[i + 1][0] != '-':
                result[a.strip('--')] = args[i + 1]
                arg_added = True
        elif arg_added:
            arg_added = False
        else:
            argv.append(a)
    return result, argv


def getCwdArg(arg, argv):
    for a in argv:
        key, sep, value = a.partition(':=')
        if sep and arg == key:
            return value
    return None


def main(argv=sys.argv):
    try:
        print_help = True
        options, args = parse_options(argv)
        if options['show_screen_log']:
            logfile = screen.get_logfile(node=options['show_screen_log'])
            if not os.path.isfile(logfile):
                raise Exception('screen logfile not found for: %s' % options['show_screen_log'])
            cmd = ' '.join([Settings.LOG_VIEWER, str(logfile)])
            print(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False
        if options['tail_screen_log']:
            logfile = screen.get_logfile(node=options['tail_screen_log'])
            if not os.path.isfile(logfile):
                raise Exception('screen logfile not found for: %s' % options['tail_screen_log'])
            cmd = ' '.join(['tail', '-f', '-n', '25', str(logfile)])
            print(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False
        elif options['show_ros_log']:
            logfile = screen.get_ros_logfile(node=options['show_ros_log'])
            if not os.path.isfile(logfile):
                raise Exception('ros logfile not found for: %s' % options['show_ros_log'])
            cmd = ' '.join([Settings.LOG_VIEWER, str(logfile)])
            print(cmd)
            p = subprocess.Popen(shlex.split(cmd))
            p.wait()
            print_help = False
        elif options['ros_log_path']:
            if options['ros_log_path'] == '[]':
                print(get_ros_home())
            else:
                print(screen.get_logfile(node=options['ros_log_path']))
            print_help = False
        elif options['delete_logs']:
            logfile = screen.get_logfile(node=options['delete_logs'])
            pidfile = screen.get_pidfile(node=options['delete_logs'])
            roslog = screen.get_ros_logfile(node=options['delete_logs'])
            if os.path.isfile(logfile):
                os.remove(logfile)
            if os.path.isfile(pidfile):
                os.remove(pidfile)
            if os.path.isfile(roslog):
                os.remove(roslog)
            print_help = False
        elif options['node_type'] and options['package'] and options['node_name']:
            runNode(options['package'], options['node_type'], options['node_name'],
                    args, options['prefix'], options['node_respawn'], options['masteruri'], loglevel=options['loglevel'])
            print_help = False
        elif options['pidkill']:
            import signal
            os.kill(int(options['pidkill']), signal.SIGKILL)
            print_help = False
        elif options['package']:
            print(roslib.packages.get_pkg_dir(options['package']))
            print_help = False
        if print_help:
            parser = _get_optparse()
            parser.print_help()
            time.sleep(3)
    except Exception as e:
        sys.stderr.write("%s\n" % e)


def rosconsole_cfg_file(package, loglevel='INFO'):
    result = os.path.join(screen.LOG_PATH, '%s.rosconsole.config' % package)
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


def runNode(package, executable, name, args, prefix='', repawn=False, masteruri=None, loglevel=''):
    '''
    Runs a ROS node. Starts a roscore if needed.
    '''
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
        raise StartException(' '.join([executable, 'in package [', package, '] not found!\n\nThe package was created?\nIs the binary executable?\n']))
    # create string for node parameter. Set arguments with spaces into "'".
    cmd = remove_src_binary(cmd)
    node_params = ' '.join(''.join(["'", a, "'"]) if a.find(' ') > -1 else a for a in args[1:])
    cmd_args = [screen.get_cmd(name), RESPAWN_SCRIPT if repawn else '', prefix, cmd[0], node_params]
    print('run on remote host:', ' '.join(cmd_args))
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
    subprocess.Popen(shlex.split(str(' '.join(cmd_args))), cwd=cwd, env=new_env)
    if len(cmd) > 1:
        rospy.logwarn('Multiple executables are found! The first one was started! Exceutables:\n%s', str(cmd))


if __name__ == '__main__':
    main()

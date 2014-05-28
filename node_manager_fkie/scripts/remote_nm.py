#!/usr/bin/env python

import os
import sys
import shlex, subprocess

import time
import roslib; roslib.load_manifest('node_manager_fkie')
import rospy
import node_manager_fkie as nm



def _get_optparse():
  '''
  This method is only to print help on options parse errors.
  '''
  import optparse

  parser = optparse.OptionParser(usage='usage: %prog [options] [args]')
  parser.add_option('--show_screen_log', metavar='show_screen_log', default='',
                     help='Shows the screen log of the given node')
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
  result = {'show_screen_log' : '',
            'show_ros_log' : '',
            'ros_log_path' : '',
            'ros_logs' : '',
            'delete_logs' : '',
            'node_type' : '',
            'node_name' : '',
            'package' : '',
            'prefix' : '',
            'pidkill' : '',
            'node_respawn' : '',
            'masteruri' : ''}
  options = [''.join(['--', v]) for v in result.keys()]
  argv = []
  arg_added = False
#  print 'options:', options
#  print 'ENUMERATE:',enumerate(options)
  for i, a in enumerate(args):
    if a in options:
      if i+1 < len(args) and len(args) > i+1 and args[i+1][0] != '-':
        result[a.strip('--')] = args[i+1]
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
#    (options, args) = parser.parse_args(argv)
    options, args = parse_options(argv)
#    print "ARGS", args
#    print 'OPTIONS:', options
    if args:
      if options['show_screen_log']:
        logfile = nm.ScreenHandler.getScreenLogFile(node=options['show_screen_log'])
        p = subprocess.Popen(shlex.split(' '.join([nm.Settings.LOG_VIEWER, str(logfile)])))
        p.wait()
      elif options['show_ros_log']:
        logfile = nm.ScreenHandler.getROSLogFile(node=options['show_ros_log'])
        p = subprocess.Popen(shlex.split(' '.join([nm.Settings.LOG_VIEWER, str(logfile)])))
        p.wait()
      elif options['ros_log_path']:
        if options['ros_log_path'] == '[]':
          print nm.get_ros_home()
        else:
          print nm.ScreenHandler.getScreenLogFile(node=options['ros_log_path'])
      elif options['delete_logs']:
        logfile = nm.ScreenHandler.getScreenLogFile(node=options['delete_logs'])
        pidfile = nm.ScreenHandler.getScreenPidFile(node=options['delete_logs'])
        roslog = nm.ScreenHandler.getROSLogFile(node=options['delete_logs'])
        if os.path.isfile(logfile):
          os.remove(logfile)
        if os.path.isfile(pidfile):
          os.remove(pidfile)
        if os.path.isfile(roslog):
          os.remove(roslog)
      elif options['node_type'] and options['package'] and options['node_name']:
        runNode(options['package'], options['node_type'], options['node_name'], 
                args, options['prefix'], options['node_respawn'], options['masteruri'])
      elif options['pidkill']:
        import signal
        os.kill(int(options['pidkill']), signal.SIGKILL)
      elif options['package']:
        print roslib.packages.get_pkg_dir(options['package'])
    else:
      parser = _get_optparse()
      parser.print_help()
      time.sleep(3)
    
  except Exception, e:
    print >> sys.stderr, e

def runNode(package, type, name, args, prefix='', repawn=False, masteruri=None):
  '''
  Runs a ROS node. Starts a roscore if needed.
  '''
  if not masteruri: 
    masteruri = nm.masteruri_from_ros()
  #start roscore, if needed
  nm.StartHandler._prepareROSMaster(masteruri)
  # start node
  try:
    cmd = roslib.packages.find_node(package, type)
  except roslib.packages.ROSPkgException as e:
    # multiple nodes, invalid package
    raise nm.StartException(str(e))
  # handle different result types str or array of string (electric / fuerte)
  import types
  if isinstance(cmd, types.StringTypes):
    cmd = [cmd]
  if cmd is None or len(cmd) == 0:
    raise nm.StartException(' '.join([type, 'in package [', package, '] not found!\n\nThe package was created?\nIs the binary executable?\n']))
  # create string for node parameter. Set arguments with spaces into "'".
  node_params = ' '.join(''.join(["'", a, "'"]) if a.find(' ') > -1 else a for a in args[1:])
  cmd_args = [nm.ScreenHandler.getSceenCmd(name), nm.Settings.RESPAWN_SCRIPT if repawn else '', prefix, cmd[0], node_params]
  print 'run on remote host:', ' '.join(cmd_args)
  # determine the current working path
  arg_cwd = getCwdArg('__cwd', args) 
  cwd = nm.get_ros_home()
  if not (arg_cwd is None):
    if arg_cwd == 'ROS_HOME':
      cwd = nm.get_ros_home()
    elif arg_cwd == 'node':
      cwd = os.path.dirname(cmd[0])
  # set the masteruri to launch with other one master
  new_env = dict(os.environ)
  new_env['ROS_MASTER_URI'] = masteruri
  subprocess.Popen(shlex.split(str(' '.join(cmd_args))), cwd=cwd, env=new_env)
  if len(cmd) > 1:
    rospy.logwarn('Multiple executables are found! The first one was started! Exceutables:\n%s', str(cmd))
if __name__ == '__main__':
  main()


#!/usr/bin/env python
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

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2012 Alexander Tiderko, Fraunhofer FKIE/US"
__license__ = "BSD"
__version__ = "0.3.0"
__date__ = "2012-02-01"

import os
import sys
import socket
import threading

PKG_NAME = 'node_manager_fkie'

import roslib; roslib.load_manifest(PKG_NAME)
import rospy

#PYTHONVER = (2, 7, 1)
#if sys.version_info < PYTHONVER:
#  print 'For full scope of operation this application requires python version > %s, current: %s' % (str(PYTHONVER), sys.version_info)

from ssh_handler import SSHhandler, AuthenticationRequest
from screen_handler import ScreenHandler, ScreenSelectionRequest
from start_handler import StartHandler, StartException, BinarySelectionRequest
from progress_queue import InteractionNeededError
from name_resolution import NameResolution
from history import History
from file_watcher import FileWatcher
from common import get_ros_home, masteruri_from_ros

# set the cwd to the package of the node_manager_fkie to support the images
# in HTML descriptions of the robots and capabilities
PACKAGE_DIR = ''.join([roslib.packages.get_pkg_dir(PKG_NAME), os.path.sep])
ROBOTS_DIR = ''.join([PACKAGE_DIR, os.path.sep, 'images', os.path.sep])

CFG_PATH = ''.join(['.node_manager', os.sep])
'''@ivar: configuration path to store the history.'''

LESS = "/usr/bin/less -fKLnQrSU"
STARTER_SCRIPT = 'rosrun node_manager_fkie remote_nm.py'
RESPAWN_SCRIPT = 'rosrun node_manager_fkie respawn'
'''
the script used on remote hosts to start new ROS nodes
'''

HOSTS_CACHE = dict()
''' 
the cache directory to store the results of tests for local hosts.
@see: L{is_local()}
'''

HELP_FILE = ''.join([PACKAGE_DIR, os.path.sep, 'README.rst'])

_lock = threading.RLock()

def terminal_cmd(cmd, title):
  '''
  Creates a command string to run with a terminal prefix
  @param cmd: the list with a command and args
  @type cmd: [str,..]
  @param title: the title of the terminal
  @type title: str
  @return: command with a terminal prefix
  @rtype:  str
  '''
  if os.path.isfile('/usr/bin/xterm'):
    return str(' '.join(['/usr/bin/xterm', '-geometry 112x35', '-title', str(title), '-e', ' '.join(cmd)]))
  elif os.path.isfile('/usr/bin/konsole'):
    return str(' '.join(['/usr/bin/konsole', '--noclose', '-title', str(title), '-e', ' '.join(cmd)]))

main_form = None
_ssh_handler = None
_screen_handler = None
_start_handler = None
_name_resolution = None
_history = None
_file_watcher = None
app = None

def ssh():
  '''
  @return: The SSH handler to handle the SSH connections
  @rtype: L{SSHhandler}
  '''
  global _ssh_handler
  return _ssh_handler

def screen():
  '''
  @return: The screen handler to the screens.
  @rtype: L{ScreenHandler}
  @see: U{http://linuxwiki.de/screen}
  '''
  global _screen_handler
  return _screen_handler

def starter():
  '''
  @return: The start handler to handle the start of new ROS nodes on local or 
  remote machines.
  @rtype: L{StartHandler}
  '''
  global _start_handler
  return _start_handler

def nameres():
  '''
  @return: The name resolution object translate the the name to the host or
  ROS master URI.
  @rtype: L{NameResolution}
  '''
  global _name_resolution
  return _name_resolution

def history():
  '''
  @return: The history of entered parameter.
  @rtype: L{History}
  '''
  global _history
  return _history

def file_watcher():
  '''
  @return: The history of entered parameter.
  @rtype: L{History}
  '''
  global _file_watcher
  return _file_watcher


def is_local(hostname):
  '''
  Test whether the given host name is the name of the local host or not.
  @param hostname: the name or IP of the host
  @type hostname: C{str}
  @return: C{True} if the hostname is local or None
  @rtype: C{bool}
  @raise Exception: on errors while resolving host
  '''
  if (hostname is None):
    return True
  with _lock:
    if hostname in HOSTS_CACHE:
      if isinstance(HOSTS_CACHE[hostname], threading.Thread):
        return False
      return HOSTS_CACHE[hostname]
  
  try:
    machine_addr = socket.inet_aton(hostname)
    local_addresses = ['localhost'] + roslib.network.get_local_addresses()
    # check 127/8 and local addresses
    result = machine_addr.startswith('127.') or machine_addr in local_addresses
    with _lock:
      HOSTS_CACHE[hostname] = result
    return result
  except socket.error:
    # the hostname must be resolved => do it in a thread
    thread = threading.Thread(target=__is_local, args=((hostname,)))
    thread.daemon = True
    thread.start()
    with _lock:
      HOSTS_CACHE[hostname] = thread
  return False

def __is_local(hostname):
  import roslib
  try:
    machine_addr = socket.gethostbyname(hostname)
  except socket.gaierror:
    import traceback
    print traceback.format_exc()
    with _lock:
      HOSTS_CACHE[hostname] = False
    return
  local_addresses = ['localhost'] + roslib.network.get_local_addresses()
  # check 127/8 and local addresses
  result = machine_addr.startswith('127.') or machine_addr in local_addresses
  with _lock:
    HOSTS_CACHE[hostname] = result

def finish(*arg):
  '''
  Callback called on exit of the ros node.
  '''
  # close all ssh sessions
  global _ssh_handler
  if not _ssh_handler is None:
    _ssh_handler.close()
  global _history
  if not _history is None:
    _history.storeAll()
  global main_form
  import main_window
  if isinstance(main_form, main_window.MainWindow):
    main_form.finish()
  global app
  if not app is None:
    app.exit()


def setTerminalName(name):
  '''
  Change the terminal name.
  @param name: New name of the terminal
  @type name:  C{str}
  '''
  sys.stdout.write("".join(["\x1b]2;",name,"\x07"]))


def setProcessName(name):
  '''
  Change the process name.
  @param name: New process name
  @type name:  C{str}
  '''
  try:
    from ctypes import cdll, byref, create_string_buffer
    libc = cdll.LoadLibrary('libc.so.6')
    buff = create_string_buffer(len(name)+1)
    buff.value = name
    libc.prctl(15, byref(buff), 0, 0, 0)
  except:
    pass


def init_cfg_path():
  global CFG_PATH
  masteruri = masteruri_from_ros()
  CFG_PATH = ''.join([get_ros_home(), os.sep, 'node_manager', os.sep])
  '''
  Creates and runs the ROS node.
  '''
  if not os.path.isdir(CFG_PATH):
    os.makedirs(CFG_PATH)
  # start ROS-Master, if not currently running
  StartHandler._prepareROSMaster(masteruri)
  return masteruri

def init_globals(masteruri):
  # initialize the global handler 
  global _ssh_handler
  global _screen_handler
  global _start_handler
  global _name_resolution
  global _history
  global _file_watcher
  _ssh_handler = SSHhandler()
  _screen_handler = ScreenHandler()
  _start_handler = StartHandler()
  _name_resolution = NameResolution()
  _history = History()
  _file_watcher = FileWatcher()

  # test where the roscore is running (local or remote)
  __is_local('localhost') ## fill cache
  __is_local(_name_resolution.getHostname(masteruri)) ## fill cache
  return is_local(_name_resolution.getHostname(masteruri))


#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%                 MAIN                               %%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main(name, anonymous=False):
  masteruri = init_cfg_path()

  args = rospy.myargv(argv=sys.argv)
  # decide to show main or echo dialog
  if len(args) >= 4 and args[1] == '-t':
    name = ''.join([name, '_echo'])
    anonymous = True

  try:
    from python_qt_binding.QtGui import QApplication
  except:
    print >> sys.stderr, "please install 'python-pyside' package!!"
    sys.exit(-1)
  rospy.init_node(name, anonymous=anonymous, log_level=rospy.DEBUG)
  setTerminalName(rospy.get_name())
  setProcessName(rospy.get_name())

  # Initialize Qt
  global app
  app = QApplication(sys.argv)

  # decide to show main or echo dialog
  import main_window, echo_dialog
  global main_form
  if len(args) >= 4 and args[1] == '-t':
    show_hz_only = (len(args) > 4 and args[4] == '--hz')
    main_form = echo_dialog.EchoDialog(args[2], args[3], show_hz_only, masteruri)
  else:
    local_master = init_globals(masteruri)

    #start the gui
    main_form = main_window.MainWindow(args, not local_master)

  if not rospy.is_shutdown():
    os.chdir(PACKAGE_DIR) # change path to be able to the images of descriptions
    main_form.show()
    exit_code = -1
    rospy.on_shutdown(finish)
    exit_code = app.exec_()
#    finally:
#      print "final"
#      sys.exit(exit_code)
#      print "ex"

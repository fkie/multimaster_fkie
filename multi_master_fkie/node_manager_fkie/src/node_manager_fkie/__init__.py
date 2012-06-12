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
#  * Neither the name of I Heart Engineering nor the names of its
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
__version__ = "0.1"
__date__ = "2012-02-01"

import os
import sys
import signal
import socket

import roslib; roslib.load_manifest('node_manager_fkie')
import rospy

#PYTHONVER = (2, 7, 1)
#if sys.version_info < PYTHONVER:
#  print 'For full scope of operation this application requires python version > %s, current: %s' % (str(PYTHONVER), sys.version_info)

from ssh_handler import SSHhandler
from screen_handler import ScreenHandler
from start_handler import StartHandler, StartException 
from name_resolution import NameResolution

NODE_NAME = "node_manager"
ROBOTS_DIR = ''.join([os.path.abspath(os.path.dirname(os.path.dirname(sys.argv[0]))), os.path.sep, 'robots', os.path.sep])
LESS = "/usr/bin/less -fKLnQrSU"
STARTER_SCRIPT = 'rosrun node_manager_fkie remote_nm.py'
'''
the script used on remote hosts to start new ROS nodes
'''
ARG_HISTORY_LENGTH = 5
''' 
the history for each required argument to load a launch file.
''' 
HOSTS_CACHE = {}
''' 
the cache directory to store the results of tests for local hosts.
@see: L{is_local()}
''' 


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
  return ' '.join(['/usr/bin/xterm', '-geometry 112x35', '-title', str(title), '-e', ' '.join(cmd)])

_ssh_handler = None
_screen_handler = None
_start_handler = None
_name_resolution = None
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

def is_local(hostname):
  '''
  Test whether the given host name is the name of the local host or not.
  @param hostname: the name or IP of the host
  @type hostname: C{str}
  @return: C{True} if the hostname is local or None
  @rtype: C{bool}
  @raise Exception: on errors while resolving host
  '''
  if hostname in HOSTS_CACHE:
    return HOSTS_CACHE[hostname]
  
  if (hostname is None):
    return True
  import socket
  import roslib
  try:
    machine_addr = socket.gethostbyname(hostname)
  except socket.gaierror:
    raise Exception("cannot resolve host address for machine [%s]"%str(hostname))
  local_addresses = ['localhost'] + roslib.network.get_local_addresses()
  # check 127/8 and local addresses
  result = machine_addr.startswith('127.') or machine_addr in local_addresses
  #491: override local to be ssh if machine.user != local user
#    if is_local and machine.user:
#      import getpass
#      is_local = machine.user == getpass.getuser()
  HOSTS_CACHE[hostname] = result
  return result

def masteruri_from_ros():
  '''
  Returns the master URI depending on ROS distribution API.
  @return: ROS master URI
  @rtype: C{str}
  '''
  try:
    import rospkg.distro
    distro = rospkg.distro.current_distro_codename()
    if distro in ['electric', 'diamondback', 'cturtle']:
      return roslib.rosenv.get_master_uri()
    else:
      import rosgraph
      return rosgraph.rosenv.get_master_uri()
  except:
    return roslib.rosenv.get_master_uri()

def finish(*arg):
  '''
  Callback called on exit of the ros node.
  '''
  # close all ssh sessions
  global _ssh_handler
  _ssh_handler.close()
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


#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%                 MAIN                               %%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def main():
  '''
  Creates and runs the ROS node.
  '''
  setTerminalName(NODE_NAME)
  setProcessName(NODE_NAME)
  try:
    from PySide.QtGui import QApplication
    from PySide.QtCore import QTimer
  except:
    print >> sys.stderr, "please install 'python-pyside' package!!"
    sys.exit(-1)
  rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
  # Initialize Qt
  global app
  app = QApplication(sys.argv)

  # initialize the global handler 
  global _ssh_handler
  global _screen_handler
  global _start_handler
  global _name_resolution
  _ssh_handler = SSHhandler()
  _screen_handler = ScreenHandler()
  _start_handler = StartHandler()
  _name_resolution = NameResolution()

  #start the gui
  import main_window
  mainForm = main_window.MainWindow()
  if not rospy.is_shutdown():
    mainForm.show()
    exit_code = -1
    try:
      rospy.on_shutdown(finish)
      exit_code = app.exec_()
      mainForm.finish()
    finally:
      sys.exit(exit_code)



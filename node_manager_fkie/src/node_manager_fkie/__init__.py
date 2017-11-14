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

import argparse
import os
import roslib.network
import rospy
import socket
import sys
import threading

from master_discovery_fkie.common import get_hostname
from node_manager_fkie.common import get_ros_home, masteruri_from_ros
from node_manager_fkie.file_watcher import FileWatcher
from node_manager_fkie.history import History
from node_manager_fkie.master_view_proxy import LaunchArgsSelectionRequest
from node_manager_fkie.name_resolution import NameResolution
from node_manager_fkie.progress_queue import InteractionNeededError
from node_manager_fkie.screen_handler import ScreenHandler, ScreenSelectionRequest
from node_manager_fkie.settings import Settings
from node_manager_fkie.ssh_handler import SSHhandler, AuthenticationRequest
from node_manager_fkie.start_handler import StartException, AdvRunCfg
from node_manager_fkie.start_handler import StartHandler, BinarySelectionRequest


PKG_NAME = 'node_manager_fkie'

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2012 Alexander Tiderko, Fraunhofer FKIE/US"
__license__ = "BSD"
__version__ = "0.7.7"  # git describe --tags --dirty --always
__date__ = "2017-10-27"  # git log -1 --date=iso

# PYTHONVER = (2, 7, 1)
# if sys.version_info < PYTHONVER:
#  print 'For full scope of operation this application requires python version > %s, current: %s' % (str(PYTHONVER), sys.version_info)


HOSTS_CACHE = dict()
'''
the cache directory to store the results of tests for local hosts.
@see: L{is_local()}
'''

_LOCK = threading.RLock()

_MAIN_FORM = None
_SETTINGS = None
_SSH_HANDLER = None
_SCREEN_HANDLER = None
_START_HANDLER = None
_NAME_RESOLUTION = None
_HISTORY = None
_FILE_WATCHER = None
_FILE_WATCHER_PARAM = None
_QAPP = None


def settings():
    '''
    @return: The global settings
    @rtype: L{Settings}
    '''
    return _SETTINGS


def ssh():
    '''
    @return: The SSH handler to handle the SSH connections
    @rtype: L{SSHhandler}
    '''
    return _SSH_HANDLER


def screen():
    '''
    @return: The screen handler to the screens.
    @rtype: L{ScreenHandler}
    @see: U{http://linuxwiki.de/screen}
    '''
    return _SCREEN_HANDLER


def starter():
    '''
    @return: The start handler to handle the start of new ROS nodes on local or
    remote machines.
    @rtype: L{StartHandler}
    '''
    return _START_HANDLER


def nameres():
    '''
    @return: The name resolution object translate the the name to the host or
    ROS master URI.
    @rtype: L{NameResolution}
    '''
    return _NAME_RESOLUTION


def history():
    '''
    @return: The history of entered parameter.
    @rtype: L{History}
    '''
    return _HISTORY


def filewatcher():
    '''
    @return: The file watcher object with all loaded configuration files.
    @rtype: L{FileWatcher}
    '''
    return _FILE_WATCHER


def file_watcher_param():
    '''
    @return: The file watcher object with all configuration files referenced by
             parameter value.
    @rtype: L{FileWatcher}
    '''
    return _FILE_WATCHER_PARAM


def get_ros_hostname(url):
    '''
    Returns the host name used in a url, if it is a name. If it is an IP an
    empty string will be returned.

    @return: host or '' if url is an IP or invalid

    @rtype:  C{str}
    '''
    return NameResolution.get_ros_hostname(url)


def is_local(hostname, wait=False):
    '''
    Test whether the given host name is the name of the local host or not.
    @param hostname: the name or IP of the host
    @type hostname: C{str}
    @return: C{True} if the hostname is local or None
    @rtype: C{bool}
    @raise Exception: on errors while resolving host
    '''
    if hostname is None:
        return True
    with _LOCK:
        if hostname in HOSTS_CACHE:
            if isinstance(HOSTS_CACHE[hostname], threading.Thread):
                return False
            return HOSTS_CACHE[hostname]
    try:
        socket.inet_aton(hostname)
        local_addresses = ['localhost'] + roslib.network.get_local_addresses()
        # check 127/8 and local addresses
        result = hostname.startswith('127.') or hostname in local_addresses
        with _LOCK:
            HOSTS_CACHE[hostname] = result
        return result
    except socket.error:
        # the hostname must be resolved => do it in a thread
        if wait:
            result = __is_local(hostname)
            return result
        else:
            thread = threading.Thread(target=__is_local, args=((hostname,)))
            thread.daemon = True
            with _LOCK:
                HOSTS_CACHE[hostname] = thread
            thread.start()
    return False


def __is_local(hostname):
    '''
    Test the hostname whether it is local or not. Uses socket.gethostbyname().
    '''
    try:
        machine_addr = socket.gethostbyname(hostname)
    except socket.gaierror:
        with _LOCK:
            HOSTS_CACHE[hostname] = False
        return False
    local_addresses = ['localhost'] + roslib.network.get_local_addresses()
    # check 127/8 and local addresses
    result = machine_addr.startswith('127.') or machine_addr in local_addresses
    with _LOCK:
        HOSTS_CACHE[hostname] = result
    return result


def finish(*arg):
    '''
    Callback called on exit of the ros node.
    '''
    # close all ssh sessions
    if _SSH_HANDLER is not None:
        _SSH_HANDLER.close()
    # save the launch history
    if _HISTORY is not None:
        try:
            _HISTORY.storeAll()
        except Exception as err:
            print >> sys.stderr, "Error while store history: %s" % err
    from node_manager_fkie.main_window import MainWindow
    # stop all threads in the main window
    if isinstance(_MAIN_FORM, MainWindow):
        _MAIN_FORM.finish()
    if _QAPP is not None:
        _QAPP.exit()


def set_terminal_name(name):
    '''
    Change the terminal name.
    @param name: New name of the terminal
    @type name:  C{str}
    '''
    sys.stdout.write("\x1b]2;%s\x07" % name)


def set_process_name(name):
    '''
    Change the process name.
    @param name: New process name
    @type name:  C{str}
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except:
        pass


def init_settings():
    global _SETTINGS
    _SETTINGS = Settings()


def init_globals(masteruri):
    '''
    :return: True if the masteruri referred to localhost
    :rtype: bool
    '''
    # initialize the global handler
    global _SSH_HANDLER
    global _SCREEN_HANDLER
    global _START_HANDLER
    global _NAME_RESOLUTION
    global _HISTORY
    global _FILE_WATCHER
    global _FILE_WATCHER_PARAM
    _SSH_HANDLER = SSHhandler()
    _SCREEN_HANDLER = ScreenHandler()
    _START_HANDLER = StartHandler()
    _NAME_RESOLUTION = NameResolution()
    _HISTORY = History()
    _FILE_WATCHER = FileWatcher()
    _FILE_WATCHER_PARAM = FileWatcher()

    # test where the roscore is running (local or remote)
    __is_local('localhost')  # fill cache
    return __is_local(get_hostname(masteruri))  # fill cache


def init_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", action="version", version="%s %s" % ("%(prog)s", __version__))
    parser.add_argument("-f", "--file", nargs=1, help="loads the given file as default on start")
    parser.add_argument("-m", "--muri", nargs=1, default='', help="starts ROS master with given URI, usefull on hosts "
                                                                  "with multiple interfaces. ROS_HOSTNAME will be set "
                                                                  "to the host of this URI, but only if it is not an IP.")

    group = parser.add_argument_group('echo')
    group.add_argument("--echo", nargs=2, help="starts an echo dialog instead of node manager", metavar=('name', 'type'))
    group.add_argument("--hz", action="store_true", help="shows only the Hz value instead of topic content in echo dialog")
    group.add_argument("--ssh", action="store_true", help="connects via SSH")

    return parser


def init_echo_dialog(prog_name, masteruri, topic_name, topic_type, hz=False, use_ssh=False):
    '''
    Intialize the environment to start an echo window.
    '''
    # start ROS-Master, if not currently running
#  StartHandler._prepareROSMaster(masteruri)
    name = '%s_echo' % prog_name
    rospy.init_node(name, anonymous=True, log_level=rospy.INFO)
    set_terminal_name(name)
    set_process_name(name)
    from node_manager_fkie.echo_dialog import EchoDialog
    global _SSH_HANDLER
    _SSH_HANDLER = SSHhandler()
    return EchoDialog(topic_name, topic_type, hz, masteruri, use_ssh=use_ssh)


def init_main_window(prog_name, masteruri, launch_files=[]):
    '''
    Intialize the environment to start Node Manager.
    '''
    # start ROS-Master, if not currently running
    StartHandler._prepareROSMaster(masteruri)
    # setup the loglevel
    try:
        log_level = getattr(rospy, rospy.get_param('/%s/log_level' % prog_name, "INFO"))
    except Exception as err:
        print("Error while set the log level: %s\n->INFO level will be used!" % err)
        log_level = rospy.INFO
    rospy.init_node(prog_name, anonymous=False, log_level=log_level)
    set_terminal_name(prog_name)
    set_process_name(prog_name)
    from node_manager_fkie.main_window import MainWindow
    local_master = init_globals(masteruri)
    return MainWindow(launch_files, not local_master, launch_files)

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%                 MAIN                               %%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def main(name):
    '''
    Start the NodeManager or EchoDialog.
    :param name: the name propagated to the rospy.init_node()
    :type name: str
    '''
    try:
        from python_qt_binding.QtGui import QApplication
    except:
        try:
            from python_qt_binding.QtWidgets import QApplication
        except:
            print >> sys.stderr, "please install 'python_qt_binding' package!!"
            sys.exit(-1)

    init_settings()
    parser = init_arg_parser()
    args = rospy.myargv(argv=sys.argv)
    parsed_args = parser.parse_args(args[1:])
    if parsed_args.muri:
        masteruri = parsed_args.muri[0]
        hostname = NameResolution.get_ros_hostname(masteruri)
        os.environ['ROS_MASTER_URI'] = masteruri
        if hostname:
            os.environ['ROS_HOSTNAME'] = hostname
    masteruri = settings().masteruri()
    # Initialize Qt
    global _QAPP
    _QAPP = QApplication(sys.argv)

    # decide to show main or echo dialog
    global _MAIN_FORM
    try:
        if parsed_args.echo:
            _MAIN_FORM = init_echo_dialog(name, masteruri, parsed_args.echo[0],
                                          parsed_args.echo[1], parsed_args.hz,
                                          parsed_args.ssh)
        else:
            _MAIN_FORM = init_main_window(name, masteruri, parsed_args.file)
    except Exception as err:
        sys.exit("%s" % err)

    exit_code = 0
    # resize and show the qt window
    if not rospy.is_shutdown():
        # change path for access to the images of descriptions
        os.chdir(settings().PACKAGE_DIR)
#    _MAIN_FORM.resize(1024, 720)
        screen_size = QApplication.desktop().availableGeometry()
        if (_MAIN_FORM.size().width() >= screen_size.width() or
                _MAIN_FORM.size().height() >= screen_size.height() - 24):
            _MAIN_FORM.showMaximized()
        else:
            _MAIN_FORM.show()
        exit_code = -1
        rospy.on_shutdown(finish)
        exit_code = _QAPP.exec_()
    return exit_code

#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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
import signal
import sys
import traceback

import roslib
import rospy

import remote
from .client import GrpcClient
from .server import GrpcServer
from .common import interpret_path
from .url import get_nmd_url, get_nmd_port


def set_terminal_name(name):
    '''
    Change the terminal name.
    @param name: New name of the terminal
    @type name:  C{str}
    '''
    sys.stdout.write("".join(["\x1b]2;", name, "\x07"]))


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
    except Exception:
        pass


def init_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--load", nargs=1, help="loads given file on start;"
                                                      "comma separated for multiple files")
    parser.add_argument("-a", "--autostart", nargs=1, help="loads given file on start and launch nodes after load launch file;"
                                                           "comma separated for multiple files")
    return parser


def start_server(node_name='node_manager_daemon'):
    '''
    Creates and runs the ROS node
    '''
    # setup the loglevel
    log_level = rospy.DEBUG
    try:
        log_level = getattr(rospy, rospy.get_param('/%s/log_level' % node_name, "INFO"))
    except Exception as e:
        print "Error while set the log level: %s\n->INFO level will be used!" % e
    log_level = rospy.DEBUG
    rospy.init_node(node_name, log_level=log_level)
    set_terminal_name(node_name)
    set_process_name(node_name)
    # load parameter
    parser = init_arg_parser()
    args = rospy.myargv(argv=sys.argv)
    parsed_args = parser.parse_args(args[1:])
    load_files = []
    if parsed_args.load:
        load_files = parsed_args.load[0].split(',')
    start_files = []
    if parsed_args.autostart:
        start_files = parsed_args.autostart[0].split(',')
    try:
        launch_manager = GrpcServer()
        launch_manager.start('[::]:%s' % str(get_nmd_port()))
        # load launch file from parameter
        for lfile in load_files:
            launch_manager.load_launch_file(lfile, autostart=False)
        for sfile in start_files:
            launch_manager.load_launch_file(sfile, autostart=True)
        rospy.spin()
        launch_manager.shutdown()
    except Exception:
        # on load error the process will be killed to notify user in node_manager
        # about error
        rospy.logwarn("%s", traceback.format_exc())
        sys.stdout.write(traceback.format_exc())
        sys.stdout.flush()
        os.kill(os.getpid(), signal.SIGKILL)


def start_client(node_name='node_manager_daemon_client'):
    '''
    Creates and runs the ROS node
    '''
    # setup the loglevel
    log_level = rospy.DEBUG
    try:
        log_level = getattr(rospy, rospy.get_param('/%s/log_level' % node_name, "INFO"))
    except Exception as e:
        print "Error while set the log level: %s\n->INFO level will be used!" % e
    log_level = rospy.DEBUG
    rospy.init_node(node_name, log_level=log_level)
    set_terminal_name(node_name)
    set_process_name(node_name)
    try:
        launch_manager = GrpcClient()
        #launch_manager.start('[::]:12322')
        #remote.add_insecure_channel('localhost:12321')
        launch_manager.test_list_path('', '128.7.92.114:12321')
        launch_manager.test_list_path('/xyz')
        launch_manager.test_list_path('/home/tiderko/ros/src')
        launch_manager.test_get_file_content("%s/.bashrc" % os.getcwd())
        launch_manager.test_get_included_files(interpret_path("$(find node_manager_daemon_fkie)/tests/resources/include_dummy.launch"))
        launch_file = launch_manager.test_load_launch('node_manager_daemon_fkie', 'description_example.launch')
        if launch_file:
            launch_manager.test_reload_launch(launch_file)
        launch_manager.test_get_nodes()
        launch_manager.test_start_node('/example/map')
        rospy.spin()
        #remote.remove_insecure_channel('localhost:12321')
    except Exception:
        # on load error the process will be killed to notify user in node_manager
        # about error
        rospy.logwarn("%s", traceback.format_exc())
        sys.stdout.write(traceback.format_exc())
        sys.stdout.flush()
        os.kill(os.getpid(), signal.SIGKILL)

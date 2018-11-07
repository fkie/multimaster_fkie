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

import shlex

import rospy

from supervised_popen import SupervisedPopen
from node_manager_daemon_fkie.host import get_hostname
from node_manager_daemon_fkie import screen
import node_manager_fkie as nm
from node_manager_fkie.common import utf8


class NoScreenOpenLogRequest(Exception):
    ''' '''

    def __init__(self, node, host):
        Exception.__init__(self)
        self.node = node
        self.host = host

    def msg(self):
        return 'No screen for %s on %s found! See log for details!' % (self.node, self.host)

    def __str__(self):
        return "NoScreenOpenLogRequest for %s on %s" % (self.node, self.host)


class ScreenHandlerException(Exception):
    pass


class ScreenSelectionRequest(Exception):
    ''' '''

    def __init__(self, choices, error):
        Exception.__init__(self)
        self.choices = choices
        self.error = error

    def __str__(self):
        return "ScreenSelectionRequest from " + self.choices + "::" + repr(self.error)


class ScreenHandler(object):
    '''
    The class to handle the running screen sessions and create new sessions on
    start of the ROS nodes.
    '''

    @classmethod
    def open_screen_terminal(cls, host, screen_name, nodename, user=None):
        '''
        Open the screen output in a new terminal.
        :param str host: the host name or ip where the screen is running.
        :param str screen_name: the name of the screen to show
        :param str nodename: the name of the node is used for the title of the terminal
        :raise Exception: on errors while resolving host
        :see: L{node_manager_fkie.is_local()}
        '''
        # create a title of the terminal
        title_opt = 'SCREEN %s on %s' % (nodename, host)
        if nm.is_local(host):
            cmd = nm.settings().terminal_cmd([screen.SCREEN, '-x', screen_name], title_opt)
            rospy.loginfo("Open screen terminal: %s", cmd)
            SupervisedPopen(shlex.split(cmd), object_id=title_opt, description="Open screen terminal: %s" % title_opt)
        else:
            ps = nm.ssh().ssh_x11_exec(host, [screen.SCREEN, '-x', screen_name], title_opt, user)
            rospy.loginfo("Open remote screen terminal: %s", ps)

    @classmethod
    def open_screen(cls, node, grpc_url, auto_item_request=False, user=None, pw=None, items=[]):
        '''
        Searches for the screen associated with the given node and open the screen
        output in a new terminal.
        :param str node: the name of the node those screen output to show
        :param str grpc_url: the url of node manager daemon where the screen is running
        :raise Exception: on errors while resolving host
        :see: L{open_screen_terminal()}
        '''
        if node is None or len(node) == 0:
            return False
        try:
            host = get_hostname(grpc_url)
            if items:
                for item in items:
                    # open the selected screen
                    cls.open_screen_terminal(host, item, node, user)
            else:
                # get the available screens
                screens = nm.nmd().get_screens(grpc_url, node)
                if len(screens) == 1:
                    cls.open_screen_terminal(host, screens.keys()[0], node, user)
                else:
                    # create a list to let the user make a choice, which screen must be open
                    choices = {}
                    for sname, _nname in screens.items():
                        pid, session_name = screen.split_session_name(sname)
                        choices['%s [%d]' % (session_name, pid)] = sname
                    # Open selection
                    if len(choices) > 0:
                        if len(choices) == 1:
                            cls.open_screen_terminal(host, choices[0], node, user)
                        elif auto_item_request:
                            from select_dialog import SelectDialog
                            items, _ = SelectDialog.getValue('Show screen', '', choices.keys(), False)
                            for item in items:
                                # open the selected screen
                                cls.open_screen_terminal(host, choices[item], node, user)
                        else:
                            raise ScreenSelectionRequest(choices, 'Show screen')
                    else:
                        raise nm.InteractionNeededError(NoScreenOpenLogRequest(node, host), nm.starter().openLog, (node, host, user))
                return len(screens) > 0
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, cls.open_screen, (node, host, auto_item_request))
        except ScreenSelectionRequest as e:
            raise nm.InteractionNeededError(e, cls.open_screen, (node, host, auto_item_request, user, pw))

    @classmethod
    def kill_screens(cls, node, grpc_url, auto_ok_request=True, user=None, pw=None):
        '''
        Searches for the screen associated with the given node and kill this screens.
        :param str node: the name of the node those screen output to kill
        :param str grpc_url: the url of node manager daemon where the screen is running
        '''
        if node is None or len(node) == 0:
            return False
        try:
            # get the available screens
            screens = nm.nmd().get_screens(grpc_url, node)
            if screens:
                do_kill = True
                if auto_ok_request:
                    from node_manager_fkie.detailed_msg_box import MessageBox
                    result = MessageBox.question(None, "Kill SCREENs?", '\n'.join(screens.keys()), buttons=MessageBox.Ok | MessageBox.Cancel)
                    if result == MessageBox.Ok:
                        do_kill = True
                if do_kill:
                    host = get_hostname(grpc_url)
                    for sname, _nname in screens.items():
                        pid, _, _ = sname.partition('.')
                        if pid:
                            try:
                                nm.starter()._kill_wo(host, int(pid), auto_ok_request, user, pw)
                            except:
                                import traceback
                                rospy.logwarn("Error while kill screen (PID: %s) on host '%s': %s", utf8(pid), utf8(host), traceback.format_exc(1))
                    if nm.is_local(host):
                        SupervisedPopen([screen.SCREEN, '-wipe'], object_id='screen -wipe', description="screen: clean up the socket with -wipe")
                    else:
                        nm.ssh().ssh_exec(host, [screen.SCREEN, '-wipe'], close_stdin=True, close_stdout=True, close_stderr=True)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, cls.kill_screens, (node, grpc_url, auto_ok_request))

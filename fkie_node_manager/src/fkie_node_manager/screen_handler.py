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
import subprocess
import sys

import grpc
import rospy

from fkie_node_manager_daemon.supervised_popen import SupervisedPopen
from fkie_node_manager_daemon.host import get_hostname
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon import url
import fkie_node_manager as nm
from fkie_node_manager_daemon.common import utf8


class NoScreenOpenLogRequest(Exception):
    ''' '''

    def __init__(self, node, host):
        Exception.__init__(self)
        self.node = node
        self.host = host

    def msg(self):
        return 'No screen for %s on %s found' % (self.node, self.host)

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
    def open_screen_terminal(cls, masteruri, screen_name, nodename, use_log_widget=False, user=None):
        '''
        Open the screen output in a new terminal.
        :param str masteruri: the masteruri where the screen is running.
        :param str screen_name: the name of the screen to show
        :param str nodename: the name of the node is used for the title of the terminal
        :raise Exception: on errors while resolving host
        :see: L{fkie_node_manager.is_local()}
        '''
        # create a title of the terminal
        if use_log_widget:
            nm._MAIN_FORM.open_screen_dock(masteruri, screen_name, nodename, user)
            return
        host = get_hostname(masteruri)
        title_opt = 'SCREEN %s on %s' % (nodename, host)
        if nm.is_local(host):
            cmd = nm.settings().terminal_cmd([screen.SCREEN, '-x', screen_name], title_opt)
            rospy.loginfo("Open screen terminal: %s", cmd)
            SupervisedPopen(shlex.split(cmd), object_id=title_opt, description="Open screen terminal: %s" % title_opt)
        else:
            rospy.loginfo("Open remote screen terminal for %s to %s" % (nodename, host))
            _ps = nm.ssh().ssh_x11_exec(host, [screen.SCREEN, '-x', screen_name], title_opt, user)

    @classmethod
    def open_screen(cls, node, grpc_url, auto_item_request=False, use_log_widget=False, user=None, pw=None, items=[], use_nmd=True):
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
            try:
                muri = url.masteruri(grpc_url)
            except ValueError:
                muri = host
            if items:
                for item in items:
                    # open the selected screen
                    cls.open_screen_terminal(muri, item, node, use_log_widget, user)
            else:
                # get the available screens
                screens = {}
                try:
                    if use_nmd:
                        screens = nm.nmd().screen.get_screens(grpc_url, node)
                    else:
                        screens = cls._bc_get_active_screens(host, node, False, user=user, pwd=pw)
                except grpc.RpcError as e:
                    rospy.logwarn("can not connect to node manager daemon, detect screens using ssh...")
                    screens = cls._bc_get_active_screens(host, node, False, user=user, pwd=pw)
                if len(screens) == 1:
                    cls.open_screen_terminal(muri, list(screens.keys())[0], node, use_log_widget,user)
                else:
                    # create a list to let the user make a choice, which screen must be open
                    choices = {}
                    for sname, _nname in screens.items():
                        pid, session_name = screen.split_session_name(sname)
                        choices['%s [%d]' % (session_name, pid)] = sname
                    # Open selection
                    if len(choices) > 0:
                        if len(choices) == 1:
                            cls.open_screen_terminal(muri, choices[0], node, use_log_widget=False, user=user)
                        elif auto_item_request:
                            from select_dialog import SelectDialog
                            items, _ = SelectDialog.getValue('Show screen', '', list(choices.keys()), False, store_geometry='show_screens')
                            for item in items:
                                # open the selected screen
                                cls.open_screen_terminal(muri, choices[item], node, use_log_widget=False, user=user)
                        else:
                            raise ScreenSelectionRequest(choices, 'Show screen')
                    else:
                        if use_log_widget:
                            nm._MAIN_FORM.open_screen_dock(muri, '', node, user)
                        raise nm.InteractionNeededError(NoScreenOpenLogRequest(node, host), nm.starter().openLog, {'nodename' : node, 'host': host, 'user': user })
                return len(screens) > 0
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, cls.open_screen,
                                            {'node': node,
                                             'grpc_url': grpc_url,
                                             'auto_item_request': auto_item_request,
                                             'use_log_widget': use_log_widget,
                                             'user': user,
                                             'pw': pw,
                                             'items': items,
                                             'use_nmd': use_nmd
                                            })
        except ScreenSelectionRequest as e:
            # set use_log_widget to False on multiple screens for same node
            raise nm.InteractionNeededError(e, cls.open_screen,
                                            {'node': node,
                                             'grpc_url': grpc_url,
                                             'auto_item_request': auto_item_request,
                                             'use_log_widget': False,
                                             'user': user,
                                             'pw': pw,
                                             'items': items,
                                             'use_nmd': use_nmd
                                            })

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
            screens = nm.nmd().screen.get_screens(grpc_url, node)
            if screens:
                do_kill = True
                if auto_ok_request:
                    from fkie_node_manager.detailed_msg_box import MessageBox
                    result = MessageBox.question(None, "Kill SCREENs?", '\n'.join(list(screens.keys())), buttons=MessageBox.Ok | MessageBox.Cancel)
                    if result == MessageBox.Ok:
                        do_kill = True
                if do_kill:
                    host = get_hostname(grpc_url)
                    for sname, _nname in screens.items():
                        pid, _, _ = sname.partition('.')
                        if pid:
                            try:
                                nm.nmd().monitor.kill_process(int(pid), grpc_url)
                                # nm.starter()._kill_wo(host, int(pid), auto_ok_request, user, pw)
                            except Exception:
                                import traceback
                                rospy.logwarn("Error while kill screen (PID: %s) on host '%s': %s", utf8(pid), utf8(host), traceback.format_exc(1))
                    nm.nmd().screen.wipe_screens(grpc_url)
                    # if nm.is_local(host):
                    #     SupervisedPopen([screen.SCREEN, '-wipe'], object_id='screen -wipe', description="screen: clean up the socket with -wipe")
                    # else:
                    #     nm.ssh().ssh_exec(host, [screen.SCREEN, '-wipe'], close_stdin=True, close_stdout=True, close_stderr=True)
        except nm.AuthenticationRequest as e:
            raise nm.InteractionNeededError(e, cls.kill_screens,  {'node': node, 'grpc_url': grpc_url, 'auto_ok_request': auto_ok_request, 'user': user, 'pw': pw})

# #############################################################################
#         for backward compatibility
# #############################################################################

    @classmethod
    def _bc_get_active_screens(cls, host, nodename='', auto_pw_request=True, user=None, pwd=None):
        '''
        Returns the list with all compatible screen names. If the session is set to
        an empty string all screens will be returned.

        :param str host: the host name or IP to search for the screen session.
        :param str nodename: the name of the node
        :return: dictionary of screen name and corresponding ROS node name
        :rtype: {str: str}
        :raise Exception: on errors while resolving host
        :see: fkie_node_manager.is_local()
        '''
        output = None
        result = {}
        if nm.is_local(host):
            ps = SupervisedPopen(shlex.split('%s -ls' % screen.SCREEN), stdout=subprocess.PIPE)
            output = ps.stdout.read()
            ps.stdout.close()
        else:
            _, stdout, _, _ = nm.ssh().ssh_exec(host, [screen.SCREEN, ' -ls'], user, pwd, auto_pw_request, close_stdin=True, close_stderr=True)
            output = stdout.read()
            stdout.close()
        if output:
            splits = output.split()
            session = screen.create_session_name(nodename)
            for i in splits:
                sname = i if sys.version_info[0] <= 2 else str(i, 'utf-8')
                if sname.count('.') > 0 and sname.endswith(session) and sname.find('._') >= 0:
                    result[i] = nodename
        return result

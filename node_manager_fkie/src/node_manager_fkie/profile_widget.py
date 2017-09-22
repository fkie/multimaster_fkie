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
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
try:
    from python_qt_binding.QtGui import QDockWidget, QFileDialog
except:
    from python_qt_binding.QtWidgets import QDockWidget, QFileDialog
import os
import roslib
import rospy
import uuid
from master_discovery_fkie.common import get_hostname, resolve_url

import node_manager_fkie as nm
from .common import get_rosparam, delete_rosparam, package_name, to_url, utf8
from .detailed_msg_box import MessageBox


class ProfileWidget(QDockWidget):
    '''
    Profile widget to show the current load state of the profile
    '''

    def __init__(self, main_window, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QDockWidget.__init__(self, parent)
        # load the UI file
        profile_dock_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ProfileWidget.ui')
        loadUi(profile_dock_file, self)
        self._main_window = main_window
        self.setVisible(False)
        self._current_profile = dict()

    def get_profile_file(self):
        # save the profile
        (path, _) = QFileDialog.getSaveFileName(self,
                                                "New profile file",
                                                nm.settings().current_dialog_path,
                                                "node manager profile files (*.nmprofile);;All files (*)")  # _:=filter
        if path:
            if not path.endswith('.nmprofile'):
                path = "%s.nmprofile" % path
            nm.settings().current_dialog_path = os.path.dirname(path)
            try:
                (pkg, _) = package_name(os.path.dirname(path))  # _:=pkg_path
                if pkg is None:
                    ret = MessageBox.warning(self, "New File Error",
                                             'The new file is not in a ROS package', buttons=MessageBox.Ok | MessageBox.Cancel)
                    if ret == MessageBox.Cancel:
                        return None
                return path
            except EnvironmentError as e:
                MessageBox.warning(self, "New File Error",
                                   'Error while create a new file',
                                   utf8(e))
        return None

    def on_save_profile(self, masteruri='', path=None):
        '''
        Saves the current environment to a node manager profile.
        :param path: the pach the file to save
        :type path: str
        :param masteruri: If not empty, save the profile only for given master
        :type masteruri: str
        '''
        try:
            if path is None:
                path = self.get_profile_file()
                if path is None:
                    return
            rospy.loginfo("Save profile %s" % path)
            import yaml
            content = {}
            for muri, master in self._main_window.masters.items():
                if not masteruri or masteruri == muri:
                    running_nodes = master.getRunningNodesIfLocal()
                    configs = {}
                    md_param = {}
                    ms_param = {}
                    zc_param = {}
                    smuri = muri
                    addr = nm.nameres().address(smuri)
                    hostname = get_hostname(smuri)
                    mastername = ''
                    if nm.is_local(addr):
                        smuri = smuri.replace(hostname, '$LOCAL$')
                        addr = '$LOCAL$'
                    else:
                        mastername = nm.nameres().mastername(smuri, nm.nameres().address(smuri))
                    for node_name in running_nodes.keys():
                        node_items = master.getNode(node_name)
                        for node in node_items:
                            if node.is_running() and node.launched_cfg is not None:  # node.has_launch_cfgs(node.cfgs):
                                cfg = node.launched_cfg
                                if isinstance(node.launched_cfg, (str, unicode)):
                                    # it is default config
                                    cfg = node.launched_cfg.replace("/%s" % hostname, '/$LOCAL$').rstrip('/run')
                                else:
                                    # it is a loaded launch file, get the filename
                                    cfg = to_url(node.launched_cfg.Filename)
                                if cfg not in configs:
                                    configs[cfg] = {'nodes': []}
                                configs[cfg]['nodes'].append(node_name)
                            elif node_name.endswith('master_discovery'):
                                md_param = get_rosparam('master_discovery', muri)
                            elif node_name.endswith('master_sync'):
                                ms_param = get_rosparam('master_sync', muri)
                            elif node_name.endswith('zeroconf'):
                                zc_param = get_rosparam('zeroconf', muri)
                            elif node_name.endswith('default_cfg'):
                                # store parameter for default configuration
                                nn = node_name.replace("/%s" % hostname, '/$LOCAL$')
                                if nn not in configs:
                                    configs[nn] = {'nodes': []}
                                configs[nn]['params'] = get_rosparam(node_name, muri)
                                configs[nn]['default'] = True
                    # store arguments for launchfiles
                    for a, b in master.launchfiles.items():
                        resolved_a = to_url(a)
                        if resolved_a not in configs:
                            if resolved_a.endswith('default_cfg/run'):
                                pass
                            else:
                                configs[resolved_a] = {}
                                configs[resolved_a]['default'] = False
                        configs[resolved_a]['argv'] = b.argv
                    # fill the configuration content for yaml as dictionary
                    content[smuri] = {'mastername': mastername,
                                      'address': addr,
                                      'configs': configs}
                    if md_param:
                        content[smuri]['master_discovery'] = md_param
                    if ms_param:
                        content[smuri]['master_sync'] = ms_param
                    if zc_param:
                        content[smuri]['zeroconf'] = zc_param
            text = yaml.dump(content, default_flow_style=False)
            with open(path, 'w+') as f:
                f.write(text)
        except Exception as e:
            import traceback
            print utf8(traceback.format_exc(3))
            MessageBox.warning(self, "Save profile Error",
                               'Error while save profile',
                               utf8(e))

    def on_load_profile_file(self, path):
        '''
        Load the profile file.
        :param path: the path of the profile file.
        :type path: str
        '''
        rospy.loginfo("Load profile %s" % path)
        self.progressBar.setValue(0)
        self.setVisible(True)
        self.setWindowTitle("%s profile started" % os.path.basename(path).rstrip('.nmprofile'))
        hasstart = False
        if path:
            try:
                import yaml
                with open(path, 'r') as f:
                    # print yaml.load(f.read())
                    content = yaml.load(f.read())
                    if not isinstance(content, dict):
                        raise Exception("Mailformed profile: %s" % os.path.basename(path))
                    for muri, master_dict in content.items():
                        local_hostname = get_hostname(self._main_window.getMasteruri())
                        rmuri = muri.replace('$LOCAL$', local_hostname)
                        master = self._main_window.getMaster(rmuri)
                        running_nodes = master.getRunningNodesIfLocal()
                        usr = None
                        self._current_profile[rmuri] = set()
                        if 'user' in master_dict:
                            usr = master_dict['user']
                        if master_dict['mastername'] and master_dict['mastername']:
                            nm.nameres().add_master_entry(master.masteruri, master_dict['mastername'], master_dict['address'])
                        hostname = master_dict['address'].replace('$LOCAL$', local_hostname)
                        if 'master_discovery' in master_dict:
                            self._start_node_from_profile(master, hostname, 'master_discovery_fkie', 'master_discovery', usr, cfg=master_dict['master_discovery'])
                            self._current_profile[rmuri].add('/master_discovery')
                        if 'master_sync' in master_dict:
                            self._start_node_from_profile(master, hostname, 'master_sync_fkie', 'master_sync', usr, cfg=master_dict['master_sync'])
                            self._current_profile[rmuri].add('/master_sync')
                        if 'zeroconf' in master_dict:
                            self._start_node_from_profile(master, hostname, 'master_discovery_fkie', 'zeroconf', usr, cfg=master_dict['zeroconf'])
                            self._current_profile[rmuri].add('/zeroconf')
                        try:
                            do_start = []
                            do_not_stop = {'/rosout', rospy.get_name(), '/node_manager', '/master_discovery', '/master_sync', '*default_cfg', '/zeroconf'}
                            configs = master_dict['configs']
                            conf_set = set()
                            for cfg_name, cmdict in configs.items():
                                cfg_name = cfg_name.replace('$LOCAL$', local_hostname)
                                if cfg_name.startswith("pkg://"):
                                    cfg_name = os.path.abspath(resolve_url(cfg_name))
                                conf_set.add(cfg_name)
                                reload_launch = True
                                argv = []
                                is_default = False
                                if 'default' in cmdict:
                                    if cmdict['default']:
                                        # it is a default configuration, test for load or not
                                        is_default = True
                                        if 'argv_used' in cmdict['params']:
                                            argv = cmdict['params']['argv_used']
                                        if cfg_name in master.default_cfgs:
                                            reload_launch = False
                                            if argv:
                                                params = get_rosparam(cfg_name, rmuri)
                                                if 'argv_used' in params:
                                                    reload_launch = set(params['argv_used']) != set(argv)
                                        if reload_launch:
                                            self._main_window.on_load_launch_as_default_bypkg(cmdict['params']['package'], cmdict['params']['launch_file'], master, argv, local_hostname)
                                        else:
                                            do_not_stop.add(cfg_name)
                                elif 'argv' in cmdict:
                                    if 'argv' in cmdict:
                                        argv = cmdict['argv']
                                    # do we need to load to load/reload launch file
                                    if cfg_name in master.launchfiles:
                                        reload_launch = set(master.launchfiles[cfg_name].argv) != set(argv)
                                if reload_launch:
                                    self._main_window.launch_dock.load_file(cfg_name, argv, master.masteruri)
                                if 'nodes' in cmdict:
                                    self._current_profile[rmuri].update(cmdict['nodes'])
                                    force_start = True
                                    cfg = cfg_name if not is_default else roslib.names.ns_join(cfg_name, 'run')
                                    if not reload_launch:
                                        force_start = False
                                        do_not_stop.update(set(cmdict['nodes']))
                                        do_start.append((reload_launch, cfg, cmdict['nodes'], force_start))
                                    else:
                                        do_start.append((reload_launch, cfg, cmdict['nodes'], force_start))
                            # close unused configurations
                            for lfile in set(master.launchfiles.keys()) - conf_set:
                                master._close_cfg(lfile)
                            master.stop_nodes_by_name(running_nodes.keys(), True, do_not_stop)
                            for reload_launch, cfg, nodes, force_start in do_start:
                                if nodes:
                                    hasstart = True
                                if reload_launch:
                                    master.start_nodes_after_load_cfg(cfg.rstrip('/run'), list(nodes), force_start)
                                else:
                                    master.start_nodes_by_name(list(nodes), cfg, force_start)
                        except Exception as ml:
                            import traceback
                            print utf8(traceback.format_exc(1))
                            rospy.logwarn("Can not load launch file for %s: %s" % (muri, utf8(ml)))
            except Exception as e:
                import traceback
                print traceback.format_exc(1)
                MessageBox.warning(self, "Load profile error",
                                   'Error while load profile',
                                   utf8(e))
            if not hasstart:
                self.update_progress()
            else:
                QTimer.singleShot(1000, self.update_progress)

    def update_progress(self):
        if self.isVisible():
            count = 0
            count_run = 0
            for muri, nodes in self._current_profile.items():
                count += len(nodes)
                master = self._main_window.getMaster(muri, False)
                if master is not None:
                    running_nodes = master.getRunningNodesIfLocal()
                    profile_nodes = nodes & set(running_nodes.keys())
                    count_run += len(profile_nodes)
            if count > 0:
                progress = float(count_run) / float(count) * 100
                if progress >= 100:
                    self.setVisible(False)
                else:
                    self.progressBar.setValue(progress)

    def closeEvent(self, event):
        rospy.loginfo("Cancel profile loading...")
        QDockWidget.closeEvent(self, event)
        ret = MessageBox.warning(self, "Cancel Start?",
                                 'This stops all starting queues!', buttons=MessageBox.Ok | MessageBox.Cancel)
        if ret == MessageBox.Cancel:
            return None
        self._main_window._progress_queue.stop()
        self._main_window.launch_dock.progress_queue.stop()
        for muri, _ in self._current_profile.items():
            master = self._main_window.getMaster(muri, False)
            if master is not None:
                master.start_nodes_after_load_cfg_clear()
                master._progress_queue.stop()
        rospy.loginfo("Profile loading canceled!")

    def _start_node_from_profile(self, master, hostname, pkg, binary, usr, cfg={}):
        try:
            args = []
            restart = False
            # test for start or not
            node = master.getNode("/%s" % binary)
            if node:
                param = get_rosparam(binary, master.masteruri)
                if set(param.keys()) == set(cfg.keys()):
                    for k, v in param.items():
                        if v != cfg[k]:
                            restart = True
                            master.stop_node(node[0], True)
                            break
            else:
                restart = True
            if restart:
                delete_rosparam(binary, master.masteruri)
                for pname, pval in cfg.items():
                    args.append('_%s:=%s' % (pname, pval))
                self._main_window._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               'start %s on %s' % (binary, hostname),
                                               nm.starter().runNodeWithoutConfig,
                                               (utf8(hostname), pkg, utf8(binary), utf8(binary), args, master.masteruri, False, usr))
                self._main_window._progress_queue.start()
        except Exception as me:
            rospy.logwarn("Can not start %s for %s: %s" % (binary, master.masteruri, utf8(me)))

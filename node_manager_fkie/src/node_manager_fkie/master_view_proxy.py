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
from python_qt_binding.QtCore import QRegExp, Qt, Signal
from python_qt_binding.QtGui import QKeySequence
from rosgraph.names import is_legal_name
import getpass
import os
import re
import roslib
import rospy
import socket
import sys
import time
import traceback
import uuid
import xmlrpclib

from master_discovery_fkie.common import get_hostname
from master_discovery_fkie.master_info import NodeInfo
from node_manager_fkie.common import masteruri_from_ros, get_packages, package_name, resolve_paths, utf8
from node_manager_fkie.default_cfg_handler import DefaultConfigHandler
from node_manager_fkie.detailed_msg_box import MessageBox, DetailedError
from node_manager_fkie.html_delegate import HTMLDelegate
from node_manager_fkie.launch_config import LaunchConfig  # , LaunchConfigException
from node_manager_fkie.launch_server_handler import LaunchServerHandler
from node_manager_fkie.node_tree_model import NodeTreeModel, NodeItem, GroupItem, HostItem
from node_manager_fkie.parameter_dialog import ParameterDialog, MasterParameterDialog, ServiceDialog
from node_manager_fkie.parameter_handler import ParameterHandler
from node_manager_fkie.parameter_list_model import ParameterModel, ParameterNameItem, ParameterValueItem
from node_manager_fkie.progress_queue import ProgressQueue, InteractionNeededError  # , ProgressThread
from node_manager_fkie.select_dialog import SelectDialog
from node_manager_fkie.service_list_model import ServiceModel, ServiceItem
from node_manager_fkie.start_handler import AdvRunCfg
from node_manager_fkie.supervised_popen import SupervisedPopen
from node_manager_fkie.topic_list_model import TopicModel, TopicItem
import node_manager_fkie as nm
try:
    from python_qt_binding.QtGui import QAction, QFileDialog, QMenu, QShortcut, QWidget
    from python_qt_binding.QtGui import QApplication, QVBoxLayout
except:
    from python_qt_binding.QtWidgets import QAction, QFileDialog, QMenu, QShortcut, QWidget
    from python_qt_binding.QtWidgets import QApplication, QVBoxLayout


try:
    from python_qt_binding.QtGui import QItemSelection, QItemSelectionModel, QItemSelectionRange, QSortFilterProxyModel
except:
    from python_qt_binding.QtCore import QItemSelection, QItemSelectionModel, QItemSelectionRange, QSortFilterProxyModel


class LaunchArgsSelectionRequest(Exception):
    ''' Request needed to set the args of a launchfile from another thread.
    @param args: a dictionary with args and values
    @type args: dict
    @param error: an error description
    @type error: str
    '''

    def __init__(self, launchfile, args, error):
        Exception.__init__(self)
        self.launchfile = launchfile
        self.args_dict = args
        self.error = error

    def __str__(self):
        return "LaunchArgsSelectionRequest for " + utf8(self.args_dict) + "::" + repr(self.error)


class MasterViewProxy(QWidget):
    '''
    This class stores the informations about a ROS master and shows it on request.
    '''

    updateHostRequest = Signal(str)
    host_description_updated = Signal(str, str, str)
    '''@ivar: the signal is emitted on description changes and contains the
  ROS Master URI, host address and description a parameter.'''

    capabilities_update_signal = Signal(str, str, str, list)
    '''@ivar: the signal is emitted if a description with capabilities is received
  and has the ROS master URI, host address, the name of the default_cfg node and a list with
  descriptions (U{multimaster_msgs_fkie.srv.ListDescription<http://docs.ros.org/api/multimaster_msgs_fkie/html/srv/ListDescription.html>} Response) as parameter.'''
    remove_config_signal = Signal(str)
    '''@ivar: the signal is emitted if a default_cfg was removed'''

    description_signal = Signal(str, str, bool)
    '''@ivar: the signal is emitted to show a description (title, description)'''

    request_xml_editor = Signal(list, str)
    '''@ivar: the signal to open a xml editor dialog (list with files, search text)'''

    stop_nodes_signal = Signal(str, list)
    '''@ivar: the signal is emitted to stop on masteruri the nodes described in the list.'''

    robot_icon_updated = Signal(str, str)
    '''@ivar: the signal is emitted, if the robot icon was changed by a configuration (masteruri, path)'''

    loaded_config = Signal(str, object)
    '''@ivar: the signal is emitted, after a launchfile is successful loaded (launchfile, LaunchConfig)'''

    save_profile_signal = Signal(str)
    '''@ivar: the signal is emitted, to save profile. (masteruri) If masteruri is empty, save all masters else only for this master.'''

    DIAGNOSTIC_LEVELS = {0: 'OK',
                         1: 'WARN',
                         2: 'ERROR',
                         3: 'STALE',
                         4: 'UNKNOWN',
                         5: 'UNKNOWN'}

    def __init__(self, masteruri, parent=None):
        '''
        Creates a new master.
        @param masteruri: the URI of the ROS master
        @type masteruri: C{str}
        '''
        QWidget.__init__(self, parent)
        self.setObjectName(' - '.join(['MasterViewProxy', masteruri]))
        self.masteruri = masteruri
        self.mastername = masteruri
        try:
            self.mastername = get_hostname(self.masteruri)
        except:
            pass

        self._tmpObjects = []
        self.__master_state = None
        self.__master_info = None
        self.__force_update = False
        self.__configs = dict()  # [file name] : LaunchConfig or tuple(ROS node name, ROS service uri, ROS master URI) : ROS nodes
        self.__online = False
        self.__run_id = ''
#    self.rosconfigs = dict() # [launch file path] = LaunchConfig()
        self.__in_question = []  # stores the changed files, until the user is interacted
#    self.__uses_confgs = dict() # stores the decisions of the user for used configuration to start of node
        '''@ivar: stored the question dialogs for changed files '''
        self._stop_ignores = ['rosout', rospy.get_name(), 'node_manager', 'master_discovery', 'master_sync', 'default_cfg', 'zeroconf']
        ''' @todo: detect the names of master_discovery and master_sync ndoes'''

        self.__echo_topics_dialogs = dict()  # [topic name] = EchoDialog
        '''@ivar: stores the open EchoDialogs '''
        self.__last_info_text = None
        self.__use_sim_time = False
        self.__current_user = nm.settings().host_user(self.mastername)
        self.__robot_icons = []
        self.__current_robot_icon = None
        self.__current_parameter_robot_icon = ''
        self.__republish_params = {}  # { topic : params, created by dialog}
        self.__last_selection = 0
        self._on_stop_kill_roscore = False
        self._on_stop_poweroff = False
        self._start_nodes_after_load_cfg = dict()
        # store the running_nodes to update to duplicates after load a launch file
        self.__running_nodes = dict()  # dict (node name : masteruri)
        self.default_cfg_handler = DefaultConfigHandler()
        self.default_cfg_handler.node_list_signal.connect(self.on_default_cfg_nodes_retrieved)
        self.default_cfg_handler.description_signal.connect(self.on_default_cfg_descr_retrieved)
        self.default_cfg_handler.err_signal.connect(self.on_default_cfg_err)

        self.__launch_servers = {}  # uri : (pid, nodes)
        self.launch_server_handler = LaunchServerHandler()
        self.launch_server_handler.launch_server_signal.connect(self.on_launch_server_retrieved)
        self.launch_server_handler.error_signal.connect(self.on_launch_server_err)

        self.masterTab = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MasterTab.ui')
        loadUi(ui_file, self.masterTab)
        tabLayout = QVBoxLayout(self)
        tabLayout.setContentsMargins(0, 0, 0, 0)
        tabLayout.addWidget(self.masterTab)
        self._progress_queue_prio = ProgressQueue(self.masterTab.progressPrioFrame, self.masterTab.progressPrioBar, self.masterTab.progressCancelPrioButton, 'Prio Master - %s' % self.mastername)
        self._progress_queue = ProgressQueue(self.masterTab.progressFrame, self.masterTab.progressBar, self.masterTab.progressCancelButton, 'Master - %s' % self.mastername)

        # setup the node view
        self.node_tree_model = NodeTreeModel(nm.nameres().address(self.masteruri), self.masteruri)
        self.node_proxy_model = NodesSortFilterProxyModel(self)
        self.node_proxy_model.setSourceModel(self.node_tree_model)
        self.masterTab.nodeTreeView.setModel(self.node_proxy_model)
        self.node_tree_model.hostInserted.connect(self.on_host_inserted)
        for i, (_, width) in enumerate(NodeTreeModel.header):  # _:=name
            self.masterTab.nodeTreeView.setColumnWidth(i, width)
        self.nodeNameDelegate = HTMLDelegate()
        self.masterTab.nodeTreeView.setItemDelegateForColumn(0, self.nodeNameDelegate)
        self.masterTab.nodeTreeView.collapsed.connect(self.on_node_collapsed)
        self.masterTab.nodeTreeView.expanded.connect(self.on_node_expanded)
        sm = self.masterTab.nodeTreeView.selectionModel()
        sm.selectionChanged.connect(self.on_node_selection_changed)
        self.masterTab.nodeTreeView.activated.connect(self.on_node_activated)
        self.masterTab.nodeTreeView.clicked.connect(self.on_node_clicked)
#    self.masterTab.nodeTreeView.setAcceptDrops(True)
#    self.masterTab.nodeTreeWidget.setSortingEnabled(True)

        # setup the topic view
        self.topic_model = TopicModel()
        self.topic_proxyModel = TopicsSortFilterProxyModel(self)
        self.topic_proxyModel.setSourceModel(self.topic_model)
        self.masterTab.topicsView.setModel(self.topic_proxyModel)
#    self.masterTab.topicsView.setModel(self.topic_model)
        for i, (_, width) in enumerate(TopicModel.header):  # _:=name
            self.masterTab.topicsView.setColumnWidth(i, width)
        self.topicNameDelegate = HTMLDelegate()
        self.topicTypeDelegate = HTMLDelegate()
        self.masterTab.topicsView.setItemDelegateForColumn(0, self.topicNameDelegate)
        self.masterTab.topicsView.setItemDelegateForColumn(3, self.topicTypeDelegate)
        sm = self.masterTab.topicsView.selectionModel()
        sm.selectionChanged.connect(self.on_topic_selection_changed)
        self.masterTab.topicsView.activated.connect(self.on_topic_activated)
        self.masterTab.topicsView.clicked.connect(self.on_topic_clicked)
        self.masterTab.topicsView.setSortingEnabled(True)

        # setup the service view
        self.service_model = ServiceModel()
        self.service_proxyModel = ServicesSortFilterProxyModel(self)
        self.service_proxyModel.setSourceModel(self.service_model)
        self.masterTab.servicesView.setModel(self.service_proxyModel)
        for i, (_, width) in enumerate(ServiceModel.header):  # _:=name
            self.masterTab.servicesView.setColumnWidth(i, width)
        self.serviceNameDelegate = HTMLDelegate()
        self.serviceTypeDelegate = HTMLDelegate()
        self.masterTab.servicesView.setItemDelegateForColumn(0, self.serviceNameDelegate)
        self.masterTab.servicesView.setItemDelegateForColumn(1, self.serviceTypeDelegate)
        sm = self.masterTab.servicesView.selectionModel()
        sm.selectionChanged.connect(self.on_service_selection_changed)
        self.masterTab.servicesView.activated.connect(self.on_service_activated)
        self.masterTab.servicesView.clicked.connect(self.on_service_clicked)
        self.masterTab.servicesView.setSortingEnabled(True)

        # setup the parameter view
        self.parameter_model = ParameterModel()
        self.parameter_model.itemChanged.connect(self._on_parameter_item_changed)
        self.parameter_proxyModel = ParameterSortFilterProxyModel(self)
        self.parameter_proxyModel.setSourceModel(self.parameter_model)
        self.masterTab.parameterView.setModel(self.parameter_proxyModel)
        for i, (_, width) in enumerate(ParameterModel.header):  # _:=name
            self.masterTab.parameterView.setColumnWidth(i, width)
        self.parameterNameDelegate = HTMLDelegate()
        self.masterTab.parameterView.setItemDelegateForColumn(0, self.parameterNameDelegate)
        sm = self.masterTab.parameterView.selectionModel()
        sm.selectionChanged.connect(self.on_parameter_selection_changed)
        self.masterTab.parameterView.setSortingEnabled(True)

#    self.parameter_proxyModel.filterAcceptsRow = _filterParameterAcceptsRow
#    self.masterTab.parameterView.activated.connect(self.on_service_activated)

        # connect the buttons
        self.masterTab.startButton.clicked.connect(self.on_start_clicked)
        self.masterTab.stopButton.clicked.connect(self.on_stop_clicked)
#    self.masterTab.stopContextButton.toggled.connect(self.on_stop_context_toggled)
        self.masterTab.ioButton.clicked.connect(self.on_io_clicked)
        self.masterTab.logButton.clicked.connect(self.on_log_clicked)
        self.masterTab.logDeleteButton.clicked.connect(self.on_log_delete_clicked)
        self.masterTab.dynamicConfigButton.clicked.connect(self.on_dynamic_config_clicked)
        self.masterTab.editConfigButton.clicked.connect(self.on_edit_config_clicked)
        self.masterTab.editRosParamButton.clicked.connect(self.on_edit_rosparam_clicked)
        self.masterTab.saveButton.clicked.connect(self.on_save_clicked)
        self.masterTab.closeCfgButton.clicked.connect(self.on_close_clicked)

        self.masterTab.echoTopicButton.clicked.connect(self.on_topic_echo_clicked)
        self.masterTab.hzTopicButton.clicked.connect(self.on_topic_hz_clicked)
        self.masterTab.hzSshTopicButton.clicked.connect(self.on_topic_hz_ssh_clicked)
        self.masterTab.pubTopicButton.clicked.connect(self.on_topic_pub_clicked)
        self.masterTab.pubStopTopicButton.clicked.connect(self.on_topic_pub_stop_clicked)

        self.masterTab.callServiceButton.clicked.connect(self.on_service_call_clicked)
        self.masterTab.nodeFilterInput.textChanged.connect(self.on_node_filter_changed)
        self.masterTab.topicFilterInput.textChanged.connect(self.on_topic_filter_changed)
        self.masterTab.serviceFilterInput.textChanged.connect(self.on_service_filter_changed)
        self.masterTab.parameterFilterInput.textChanged.connect(self.on_parameter_filter_changed)
        self.masterTab.getParameterButton.clicked.connect(self.on_get_parameter_clicked)
        self.masterTab.addParameterButton.clicked.connect(self.on_add_parameter_clicked)
        self.masterTab.deleteParameterButton.clicked.connect(self.on_delete_parameter_clicked)
        self.masterTab.saveParameterButton.clicked.connect(self.on_save_parameter_clicked)

        # create a handler to request the parameter
        self.parameterHandler = ParameterHandler()
        self.parameterHandler.parameter_list_signal.connect(self._on_param_list)
        self.parameterHandler.parameter_values_signal.connect(self._on_param_values)
        self.parameterHandler.delivery_result_signal.connect(self._on_delivered_values)
        # create a handler to request sim parameter
        self.parameterHandler_sim = ParameterHandler()
#    self.parameterHandler_sim.parameter_list_signal.connect(self._on_param_list)
        self.parameterHandler_sim.parameter_values_signal.connect(self._on_sim_param_values)
#    self.parameterHandler_sim.delivery_result_signal.connect(self._on_delivered_values)

        self._shortcut_kill_node = QShortcut(QKeySequence(self.tr("Ctrl+Backspace", "Kill selected node")), self)
        self._shortcut_kill_node.activated.connect(self.on_kill_nodes)
        self._shortcut_kill_node = QShortcut(QKeySequence(self.tr("Ctrl+Delete", "Removes the registration of selected nodes from ROS master")), self)
        self._shortcut_kill_node.activated.connect(self.on_unregister_nodes)

        self.masterTab.ioButton.setEnabled(True)
        self.masterTab.tabWidget.currentChanged.connect(self.on_tab_current_changed)
        self._shortcut_screen_show_all = QShortcut(QKeySequence(self.tr("Shift+S", "Show all available screens")), self)
        self._shortcut_screen_show_all.activated.connect(self.on_show_all_screens)
        self._shortcut_screen_kill = QShortcut(QKeySequence(self.tr("Shift+Backspace", "Kill Screen")), self)
        self._shortcut_screen_kill.activated.connect(self.on_kill_screens)

        self.loaded_config.connect(self._apply_launch_config)

        # set the shortcuts
        self._shortcut1 = QShortcut(QKeySequence(self.tr("Alt+1", "Select first group")), self)
        self._shortcut1.activated.connect(self.on_shortcut1_activated)
        self._shortcut2 = QShortcut(QKeySequence(self.tr("Alt+2", "Select second group")), self)
        self._shortcut2.activated.connect(self.on_shortcut2_activated)
        self._shortcut3 = QShortcut(QKeySequence(self.tr("Alt+3", "Select third group")), self)
        self._shortcut3.activated.connect(self.on_shortcut3_activated)
        self._shortcut4 = QShortcut(QKeySequence(self.tr("Alt+4", "Select fourth group")), self)
        self._shortcut4.activated.connect(self.on_shortcut4_activated)
        self._shortcut5 = QShortcut(QKeySequence(self.tr("Alt+5", "Select fifth group")), self)
        self._shortcut5.activated.connect(self.on_shortcut5_activated)

        self._shortcut_collapse_all = QShortcut(QKeySequence(self.tr("Alt+C", "Collapse all groups")), self)
        self._shortcut_collapse_all.activated.connect(self.on_shortcut_collapse_all)
        self._shortcut_expand_all = QShortcut(QKeySequence(self.tr("Alt+E", "Expand all groups")), self)
        self._shortcut_expand_all.activated.connect(self.masterTab.nodeTreeView.expandAll)
        self._shortcut_run = QShortcut(QKeySequence(self.tr("Alt+R", "run selected nodes")), self)
        self._shortcut_run.activated.connect(self.on_start_clicked)
        self._shortcut_stop = QShortcut(QKeySequence(self.tr("Alt+S", "stop selected nodes")), self)
        self._shortcut_stop.activated.connect(self.on_stop_clicked)

#        self._shortcut_copy = QShortcut(QKeySequence(self.tr("Ctrl+C", "copy selected values to clipboard")), self)
#        self._shortcut_copy.activated.connect(self.on_copy_c_pressed)
#        self._shortcut_copy = QShortcut(QKeySequence(self.tr("Ctrl+X", "copy selected alternative values to clipboard")), self)
#        self._shortcut_copy.activated.connect(self.on_copy_x_pressed)

#    print "================ create", self.objectName()
#
#     def __del__(self):
#        print "    Destroy mester view proxy", self.objectName(), " ..."
#        print "    ", self.objectName(), "destroyed"

    def stop(self):
        print "  Shutdown master", self.masteruri, "..."
        self.default_cfg_handler.stop()
        self.launch_server_handler.stop()
        self._progress_queue_prio.stop()
        self._progress_queue.stop()
        if self._on_stop_kill_roscore:
            self.killall_roscore()
        for ps in self.__echo_topics_dialogs.values():
            try:
                ps.terminate()
            except:
                pass
        print "  Master", self.masteruri, " is down!"

    @property
    def current_user(self):
        return self.__current_user

    @current_user.setter
    def current_user(self, user):
        self.__current_user = user
        nm.settings().set_host_user(self.mastername, user)

    @property
    def is_local(self):
        return nm.is_local(get_hostname(self.masteruri))

    @property
    def online(self):
        '''
        The online meens that master is discovered and master_info was received.
        '''
        return self.__online

    @online.setter
    def online(self, state):
        self.__online = state
        self._start_process_queue()

    @property
    def master_state(self):
        return self.__master_state

    @master_state.setter
    def master_state(self, master_state):
        self.__master_state = master_state

    @property
    def master_info(self):
        return self.__master_info

    @master_info.setter
    def master_info(self, master_info):
        '''
        Sets the new master information. To determine whether a node is running the
        PID and his URI are needed. The PID of remote nodes (host of the ROS master
        and the node are different) will be not determine by discovering. Thus this
        information must be obtain from other MasterInfo object and stored while
        updating.
        @param master_info: the mater information object
        @type master_info: U{master_discovery_fkie.msg.MasterInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#module-master_discovery_fkie.master_info>}
        '''
        try:
            update_result = (set(), set(), set(), set(), set(), set(), set(), set(), set())
            if self.__master_info is None:
                if (master_info.masteruri == self.masteruri):
                    self.__master_info = master_info
                    update_result[0].update(self.__master_info.node_names)
                    update_result[3].update(self.__master_info.topic_names)
                    update_result[6].update(self.__master_info.service_names)
            else:
                update_result = self.__master_info.updateInfo(master_info)
#         print "MINFO", self.__master_info.listedState()
            # we receive the master info from remove nodes first -> skip
            if self.__master_info is None:
                return
            try:
                if (master_info.masteruri == self.masteruri):
                    self.update_system_parameter()
                    self.online = True
                # request the info of new remote nodes
                hosts2update = set([get_hostname(self.__master_info.getNode(nodename).uri) for nodename in update_result[0]])
                hosts2update.update([get_hostname(self.__master_info.getService(nodename).uri) for nodename in update_result[6]])
                for host in hosts2update:
                    if host != get_hostname(self.masteruri):
                        self.updateHostRequest.emit(host)
            except:
                pass
#      cputimes = os.times()
#      cputime_init = cputimes[0] + cputimes[1]
            # update nodes in the model
            if update_result[0] or update_result[1] or update_result[2] or self.__force_update:
                self.updateRunningNodesInModel(self.__master_info)
                self.on_node_selection_changed(None, None)
            # Updates the topic view based on the current master information.
            if update_result[3] or update_result[4] or update_result[5] or self.__force_update:
                self.topic_model.updateModelData(self.__master_info.topics, update_result[3], update_result[4], update_result[5])
                self.on_topic_selection_changed(None, None)
            # Updates the service view based on the current master information.
            if update_result[6] or update_result[7] or update_result[8] or self.__force_update:
                self.service_model.updateModelData(self.__master_info.services, update_result[6], update_result[7], update_result[8])
                self.on_service_selection_changed(None, None)
                # update the default configuration
                self.updateDefaultConfigs(self.__master_info)
            self.__force_update = False
#      cputimes = os.times()
#      cputime = cputimes[0] + cputimes[1] - cputime_init
#      print "  update on ", self.__master_info.mastername if not self.__master_info is None else self.__master_state.name, cputime
        except:
            print traceback.format_exc(1)

    def _start_process_queue(self):
        if self.online and self.master_info is not None:
            self._progress_queue.start()

    def _start_process_queue_prio(self):
        if self.online and self.master_info is not None:
            self._progress_queue_prio.start()

    @property
    def use_sim_time(self):
        return self.__use_sim_time

    def in_process(self):
        return self._progress_queue.count() > 0 or self._progress_queue_prio.count() > 0

    def force_next_update(self):
        self.__force_update = True

    def update_system_parameter(self):
        self.parameterHandler_sim.requestParameterValues(self.masteruri, ["/run_id", "/use_sim_time", "/robot_icon", "/roslaunch/uris"])

    def markNodesAsDuplicateOf(self, running_nodes):
        '''
        Marks all nodes, which are not running and in a given list as a duplicates nodes.
        @param running_nodes: The list with names of running nodes
        @type running_nodes: C{[str]}
        '''
        # store the running_nodes to update to duplicates after load a launch file
        self.__running_nodes = running_nodes
        self.node_tree_model.markNodesAsDuplicateOf(running_nodes, (self.master_info is not None and self.master_info.getNodeEndsWith('master_sync')))

    def getRunningNodesIfSync(self):
        '''
        Returns the list with all running nodes, which are registered by this ROS
        master. Also the nodes, which are physically running on remote hosts.
        @return: The list with names of running nodes
        @rtype: C{[str]}
        '''
        if self.master_info is not None and self.master_info.getNodeEndsWith('master_sync'):
            return self.master_info.node_names
        return []

    def getRunningNodesIfLocal(self, remove_system_nodes=False):
        '''
        Returns the list with all running nodes, which are running (has process) on this host.
        The nodes registered on this ROS master, but running on remote hosts are not
        returned.
        @return: The dictionary with names of running nodes and their masteruri
        @rtype: C{dict(str:str)}
        '''
        result = dict()
        if self.master_info is not None:
            for _, node in self.master_info.nodes.items():  # _:=name
                if node.isLocal:
                    if not remove_system_nodes or not self._is_in_ignore_list(node.name):
                        result[node.name] = self.master_info.masteruri
        return result

    def updateRunningNodesInModel(self, master_info):
        '''
        Creates the dictionary with ExtendedNodeInfo objects and updates the nodes view.
        @param master_info: the mater information object
        @type master_info: U{master_discovery_fkie.msg.MasterInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#module-master_discovery_fkie.master_info>}
        '''
        if master_info is not None:
            self.node_tree_model.updateModelData(master_info.nodes)
            self.updateButtons()

    def getNode(self, node_name):
        '''
        @param node_name: The name of the node.
        @type node_name: str
        @return: The list the nodes with given name.
        @rtype: []
        '''
        return self.node_tree_model.getNode("%s" % node_name, self.masteruri)

    def updateButtons(self):
        '''
        Updates the enable state of the buttons depending of the selection and
        running state of the selected node.
        '''
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        has_running = False
        has_stopped = False
        for node in selectedNodes:
            if node.uri is not None:
                has_running = True
            else:
                has_stopped = True
        self.masterTab.startButton.setEnabled(True)
        self.masterTab.stopButton.setEnabled(True)
#    self.masterTab.ioButton.setEnabled(has_running or has_stopped)
        self.masterTab.logButton.setEnabled(True)
#    self.masterTab.logButton.setEnabled(has_running or has_stopped)
        self.masterTab.logDeleteButton.setEnabled(has_running or has_stopped)
        # test for available dynamic reconfigure services
        if self.master_info is not None:
            dyn_cfg_available = False
#      dyncfgServices = [srv for srv_name, srv in self.master_info.services.items() if (srv_name.endswith('/set_parameters'))]
            for n in selectedNodes:
                for srv_name, srv in self.master_info.services.items():
                    if (srv_name.endswith('/set_parameters')) and n.name in srv.serviceProvider:
                        dyn_cfg_available = True
                        break
            self.masterTab.dynamicConfigButton.setEnabled(dyn_cfg_available)
        # the configuration is only available, if only one node is selected
        cfg_enable = False
        if len(selectedNodes) >= 1:
            cfg_enable = len(self._getCfgChoises(selectedNodes[0], True)) > 0
        self.masterTab.editConfigButton.setEnabled(cfg_enable and len(selectedNodes) == 1)
#    self.startNodesAtHostAct.setEnabled(cfg_enable)
        self.masterTab.editRosParamButton.setEnabled(len(selectedNodes) == 1)
        # self.masterTab.saveButton.setEnabled(len(self.launchfiles) > 1)
        # enable the close button only for local configurations
        self.masterTab.closeCfgButton.setEnabled(True)
#    self.masterTab.closeCfgButton.setEnabled(len([path for path, _ in self.__configs.items() if (isinstance(path, tuple) and path[2] == self.masteruri) or not isinstance(path, tuple)]) > 0) #_:=cfg

    def hasLaunchfile(self, path):
        '''
        @param path: the launch file
        @type path: C{str}
        @return: C{True} if the given launch file is open
        @rtype: C{boolean}
        '''
        return path in self.launchfiles

    @property
    def default_cfgs(self):
        '''
        Returns the copy of the dictionary with default configurations on this host
        @rtype: C{[str(ROS node name)]}
        '''
        result = []
        for (c, _) in self.__configs.items():  # _:=cfg
            if isinstance(c, tuple):
                result.append(c[0])
        return result

    @property
    def launchfiles(self):
        '''
        Returns the copy of the dictionary with loaded launch files on this host
        @rtype: C{dict(str(file) : L{LaunchConfig}, ...)}
        '''
        result = dict()
        for (c, cfg) in self.__configs.items():
            if not isinstance(c, tuple):
                result[c] = cfg
        return result

    @launchfiles.setter
    def launchfiles(self, launchfile):
        '''
        Loads the launch file. If this file is already loaded, it will be reloaded.
        After successful load the node view will be updated.
        @param launchfile: the launch file path
        @type launchfile: C{str}
        '''
        lfile = launchfile
        argv = []
        if isinstance(launchfile, tuple):
            lfile, argv = launchfile
        self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                            'Loading %s' % os.path.basename(lfile),
                                            self._load_launchfile,
                                            (lfile, argv))
        self._start_process_queue_prio()

    def _load_launchfile(self, launchfile, argv_forced=[], pqid=None):
        '''
        This method will be called in another thread. The configuration parameter
        of the launch file will be requested using `LaunchArgsSelectionRequest` and
        `InteractionNeededError`. After the file is successful loaded a
        `loaded_config` signal will be emitted.
        '''
        stored_argv = None
        if launchfile in self.__configs:
            # close the current loaded configuration with the same name
            stored_argv = self.__configs[launchfile].argv
        # load launch configuration
        try:
            # test for required args
            launchConfig = LaunchConfig(launchfile, masteruri=self.masteruri)
            loaded = False
            # if the launch file currently open these args will be used
            if stored_argv is None:
                if argv_forced:
                    # if the parameter already requested `argv_forced` is filled: load!
                    loaded = launchConfig.load(argv_forced)
                else:
                    # get the list with needed launch args
                    req_args = launchConfig.getArgs()
                    if req_args:
                        params = dict()
                        arg_dict = launchConfig.argvToDict(req_args)
                        for arg in arg_dict.keys():
                            params[arg] = ('string', [arg_dict[arg]])
                        # request the args: the dialog must run in the main thread of Qt
                        req = LaunchArgsSelectionRequest(launchfile, params, 'Needs input for args')
                        raise nm.InteractionNeededError(req, self._load_launchfile, (launchfile,))
            # load the launch file with args of currently open launch file
            if not loaded or stored_argv is not None:
                launchConfig.load(req_args if stored_argv is None else stored_argv)
            # update the names of the hosts stored in the launch file
            for _, machine in launchConfig.Roscfg.machines.items():  # _:=name
                if machine.name:
                    nm.nameres().add_info(machine.name, machine.address)
            # do not load if the loadings process was canceled
            if self._progress_queue_prio.has_id(pqid):
                self.loaded_config.emit(launchfile, launchConfig)
        except InteractionNeededError:
            raise
        except Exception as e:
            err_text = ''.join([os.path.basename(launchfile), ' loading failed!'])
            err_details = ''.join([err_text, '\n\n', e.__class__.__name__, ": ", utf8(e)])
            rospy.logwarn("Loading launch file: %s", err_details)
            raise DetailedError("Loading launch file", err_text, err_details)
#      MessageBox.warning(self, "Loading launch file", err_text, err_details)
        except:
            print traceback.format_exc(3)

    def _apply_launch_config(self, launchfile, launchConfig):
        stored_roscfg = None
        expanded_items = None
        if launchfile in self.__configs:
            # store expanded items
            expanded_items = self._get_expanded_groups()
            # close the current loaded configuration with the same name
            self.removeConfigFromModel(launchfile)
            stored_roscfg = self.__configs[launchfile].Roscfg
            del self.__configs[launchfile]
        try:
            # add launch file object to the list
            self.__configs[launchfile] = launchConfig
            self.appendConfigToModel(launchfile, launchConfig.Roscfg)
#            self.masterTab.tabWidget.setCurrentIndex(0)
            # get the descriptions of capabilities and hosts
            try:
                robot_descr = launchConfig.getRobotDescr()
                capabilities = launchConfig.getCapabilitiesDesrc()
                for (host, caps) in capabilities.items():
                    if not host:
                        host = nm.nameres().mastername(self.masteruri)
                    host_addr = nm.nameres().address(host)
                    self.node_tree_model.addCapabilities(self.masteruri, host_addr, launchfile, caps)
                for (host, descr) in robot_descr.items():
                    if not host:
                        host = nm.nameres().mastername(self.masteruri)
                    host_addr = nm.nameres().address(host)
                    tooltip = self.node_tree_model.updateHostDescription(self.masteruri, host_addr, descr['type'], descr['name'], descr['description'])
                    self.host_description_updated.emit(self.masteruri, host_addr, tooltip)
            except:
                import traceback
                print traceback.format_exc()

            # by this call the name of the host will be updated if a new one is defined in the launch file
            self.updateRunningNodesInModel(self.__master_info)
            # detect files changes
            if stored_roscfg and self.__configs[launchfile].Roscfg:
                stored_values = [(name, utf8(p.value)) for name, p in stored_roscfg.params.items()]
                new_values = [(name, utf8(p.value)) for name, p in self.__configs[launchfile].Roscfg.params.items()]
                # detect changes parameter
                paramset = set(name for name, _ in (set(new_values) - set(stored_values)))  # _:=value
                # detect new parameter
                paramset |= (set(self.__configs[launchfile].Roscfg.params.keys()) - set(stored_roscfg.params.keys()))
                # detect removed parameter
                paramset |= (set(stored_roscfg.params.keys()) - set(self.__configs[launchfile].Roscfg.params.keys()))
                # detect new nodes
                stored_nodes = [roslib.names.ns_join(item.namespace, item.name) for item in stored_roscfg.nodes]
                new_nodes = [roslib.names.ns_join(item.namespace, item.name) for item in self.__configs[launchfile].Roscfg.nodes]
                nodes2start = set(new_nodes) - set(stored_nodes)
                # determine the nodes of the changed parameter
                for p in paramset:
                    for n in new_nodes:
                        if p.startswith(n):
                            nodes2start.add(n)
                # detect changes in the arguments and remap
                for n in stored_roscfg.nodes:
                    for new_n in self.__configs[launchfile].Roscfg.nodes:
                        if n.name == new_n.name and n.namespace == new_n.namespace:
                            if n.args != new_n.args or n.remap_args != new_n.remap_args:
                                nodes2start.add(roslib.names.ns_join(n.namespace, n.name))
                # filter out anonymous nodes
                nodes2start = [n for n in nodes2start if not re.search(r"\d{3,6}_\d{10,}", n)]
                # restart nodes
                if nodes2start:
                    restart, ok = SelectDialog.getValue('Restart nodes?', "Select nodes to restart <b>@%s</b>:" % self.mastername, nodes2start, False, True, '', self)
                    if ok:
                        self.stop_nodes_by_name(restart)
                        self.start_nodes_by_name(restart, launchfile, True)
            # set the robot_icon
            if launchfile in self.__robot_icons:
                self.__robot_icons.remove(launchfile)
            self.__robot_icons.insert(0, launchfile)
            self.markNodesAsDuplicateOf(self.__running_nodes)
            # expand items to restore old view
            if expanded_items is not None:
                self._expand_groups(expanded_items)
#      print "MASTER:", launchConfig.Roscfg.master
#      print "NODES_CORE:", launchConfig.Roscfg.nodes_core
#      for n in launchConfig.Roscfg.nodes:
#        n.__slots__ = []
#      print "NODES:", pickle.dumps(launchConfig.Roscfg.nodes)
#
            # print "ROSLAUNCH_FILES:", launchConfig.Roscfg.roslaunch_files
#           # list of resolved node names. This is so that we can check for naming collisions
            # print "RESOLVED_NAMES:", launchConfig.Roscfg.resolved_node_names
#
#      print "TESTS:", launchConfig.Roscfg.tests
#      print "MACHINES:", launchConfig.Roscfg.machines
        #        print "PARAMS:", launchConfig.Roscfg.params
#        print "CLEAR_PARAMS:", launchConfig.Roscfg.clear_params
#      print "EXECS:", launchConfig.Roscfg.executables
#
#        # for tools like roswtf
#      print "ERRORS:", launchConfig.Roscfg.config_errors
#
#      print "M:", launchConfig.Roscfg.m
            if launchfile in self._start_nodes_after_load_cfg:
                self.start_nodes_by_name(self._start_nodes_after_load_cfg[launchfile], launchfile, True)
                del self._start_nodes_after_load_cfg[launchfile]
        except Exception as e:
            err_text = ''.join([os.path.basename(launchfile), ' loading failed!'])
            err_details = ''.join([err_text, '\n\n', e.__class__.__name__, ": ", utf8(e)])
            rospy.logwarn("Loading launch file: %s", err_details)
            MessageBox.warning(self, "Loading launch file", err_text, err_details)
        except:
            print traceback.format_exc(3)
        self.update_robot_icon(True)

    def reload_global_parameter_at_next_start(self, launchfile):
        try:
            self.__configs[launchfile].global_param_done.remove(self.masteruri)
            # self.on_node_selection_changed(None, None, True)
        except:
            pass

    def _get_expanded_groups(self):
        '''
        Returns a list of group names, which are expanded.
        '''
        result = []
        try:
            for r in range(self.masterTab.nodeTreeView.model().rowCount()):
                index_host = self.masterTab.nodeTreeView.model().index(r, 0)
                if index_host.isValid() and self.masterTab.nodeTreeView.isExpanded(index_host):
                    if self.masterTab.nodeTreeView.model().hasChildren(index_host):
                        for c in range(self.masterTab.nodeTreeView.model().rowCount(index_host)):
                            index_cap = self.masterTab.nodeTreeView.model().index(c, 0, index_host)
                            if index_cap.isValid() and self.masterTab.nodeTreeView.isExpanded(index_cap):
                                model_index = self.node_proxy_model.mapToSource(index_cap)
                                item = self.node_tree_model.itemFromIndex(model_index)
                                if isinstance(item, (GroupItem, HostItem)):
                                    result.append(item.name)
        except:
            print traceback.format_exc(3)
        return result

    def _expand_groups(self, groups=None):
        '''
        Expands all groups, which are in the given list. If no list is given,
        expands all groups of expanded hosts.
        '''
        try:
            for r in range(self.masterTab.nodeTreeView.model().rowCount()):
                index_host = self.masterTab.nodeTreeView.model().index(r, 0)
                if index_host.isValid() and self.masterTab.nodeTreeView.isExpanded(index_host):
                    if self.masterTab.nodeTreeView.model().hasChildren(index_host):
                        for c in range(self.masterTab.nodeTreeView.model().rowCount(index_host)):
                            index_cap = self.masterTab.nodeTreeView.model().index(c, 0, index_host)
                            if index_cap.isValid():
                                model_index = self.node_proxy_model.mapToSource(index_cap)
                                item = self.node_tree_model.itemFromIndex(model_index)
                                if isinstance(item, (GroupItem, HostItem)):
                                    if groups is None or item.name in groups:
                                        self.masterTab.nodeTreeView.setExpanded(index_cap, True)
        except:
            print traceback.format_exc(3)

    def update_robot_icon(self, force=False):
        '''
        Update the current robot icon. If the icon was changed a `robot_icon_updated`
        signal will be emitted.
        :return: the path to the current robot icon
        :rtype: str
        '''
        for l in self.__robot_icons:
            try:
                icon = self.__configs[l].get_robot_icon()
                if icon:
                    if icon != self.__current_robot_icon or force:
                        self.__current_robot_icon = icon
                        self.robot_icon_updated.emit(self.masteruri, icon)
                    return icon
            except:
                pass
        self.__current_robot_icon = self.__current_parameter_robot_icon
        self.robot_icon_updated.emit(self.masteruri, utf8(self.__current_robot_icon))
        return self.__current_robot_icon

    def appendConfigToModel(self, launchfile, rosconfig):
        '''
        Update the node view
        @param launchfile: the launch file path
        @type launchfile: C{str}
        @param rosconfig: the configuration
        @type rosconfig: L{LaunchConfig}
        '''
        hosts = dict()  # dict(addr : dict(node : [config]) )
        addr = nm.nameres().address(self.masteruri)
        masteruri = self.masteruri
        for n in rosconfig.nodes:
            if n.machine_name and not n.machine_name == 'localhost':
                if n.machine_name not in rosconfig.machines:
                    raise Exception(''.join(["ERROR: unknown machine [", n.machine_name, "]"]))
                addr = rosconfig.machines[n.machine_name].address
                masteruri = nm.nameres().masteruri(n.machine_name)
            node = roslib.names.ns_join(n.namespace, n.name)
            if (masteruri, addr) not in hosts:
                hosts[(masteruri, addr)] = dict()
            hosts[(masteruri, addr)][node] = launchfile
        # add the configurations for each host separately
        for ((masteruri, addr), nodes) in hosts.items():
            self.node_tree_model.appendConfigNodes(masteruri, addr, nodes)
        self.updateButtons()

    def removeConfigFromModel(self, launchfile):
        '''
        Update the node view after removed configuration.
        @param launchfile: the launch file path
        @type launchfile: C{str}
        '''
        if isinstance(launchfile, tuple):
            self.remove_config_signal.emit(launchfile[0])
        self.node_tree_model.removeConfigNodes(launchfile)
        self.updateButtons()

    def updateDefaultConfigs(self, master_info):
        '''
        Updates the default configuration view based on the current master information.
        @param master_info: the mater information object
        @type master_info: U{master_discovery_fkie.msg.MasterInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#module-master_discovery_fkie.master_info>}
        '''
        if self.__master_info is None:
            return
        default_cfgs = []
        for name in self.__master_info.service_names:
            if name.endswith('list_nodes'):
                srv = self.__master_info.getService(name)
                default_cfgs.append((roslib.names.namespace(name).rstrip(roslib.names.SEP), srv.uri, srv.masteruri))
        # remove the node contained in default configuration form the view
        removed = list(set([c for c in self.__configs.keys() if isinstance(c, tuple)]) - set(default_cfgs))
        if removed:
            for r in removed:
                self.node_tree_model.removeConfigNodes(r)
#        service = self.__master_info.getService(roslib.names.ns_join(r[0], 'list_nodes'))
                if r[2] == self.masteruri:
                    self.remove_config_signal.emit(r[0])
                del self.__configs[r]
        if len(self.__configs) == 0:
            address = nm.nameres().address(master_info.masteruri)
            tooltip = self.node_tree_model.updateHostDescription(master_info.masteruri, address, '', '', '')
            self.host_description_updated.emit(master_info.masteruri, address, tooltip)
        # request the nodes of new default configurations
        added = list(set(default_cfgs) - set(self.__configs.keys()))
        for (name, uri, _) in added:  # _:= masteruri
            self.default_cfg_handler.requestNodeList(uri, roslib.names.ns_join(name, 'list_nodes'))
            # request the description
            descr_service = self.__master_info.getService(roslib.names.ns_join(name, 'description'))
            if descr_service is not None:
                self.default_cfg_handler.requestDescriptionList(descr_service.uri, descr_service.name)
        self.updateButtons()

    def on_default_cfg_nodes_retrieved(self, service_uri, config_name, nodes):
        '''
        Handles the new list with nodes from default configuration service.
        @param service_uri: the URI of the service provided the default configuration
        @type service_uri: C{str}
        @param config_name: the name of default configuration service
        @type config_name: C{str}
        @param nodes: the name of the nodes with name spaces
        @type nodes: C{[str]}
        '''
        # remove the current state
        masteruri = self.masteruri
        if self.__master_info is not None:
            service = self.__master_info.getService(config_name)
            if service is not None:
                masteruri = service.masteruri
        cfg_name = roslib.names.namespace(config_name).rstrip(roslib.names.SEP)
        key = (cfg_name, service_uri, masteruri)
#    if self.__configs.has_key(key):
#      self.node_tree_model.removeConfigNodes(key)
        # add the new config
        node_cfgs = dict()
        for n in nodes:
            node_cfgs[n] = key
        host = get_hostname(service_uri)
        host_addr = nm.nameres().address(host)
        if host_addr is None:
            host_addr = host
        self.node_tree_model.appendConfigNodes(masteruri, host_addr, node_cfgs)
        self.__configs[key] = nodes
        # start nodes in the queue
        if cfg_name in self._start_nodes_after_load_cfg:
            self.start_nodes_by_name(self._start_nodes_after_load_cfg[cfg_name], roslib.names.ns_join(cfg_name, 'run'), True)
            del self._start_nodes_after_load_cfg[cfg_name]
        self.updateButtons()

    def on_default_cfg_descr_retrieved(self, service_uri, config_name, items):
        '''
        Handles the description list from default configuration service.
        Emits a Qt signal L{host_description_updated} to notify about a new host
        description and a Qt signal L{capabilities_update_signal} to notify about a capabilities
        update.
        @param service_uri: the URI of the service provided the default configuration
        @type service_uri: C{str}
        @param config_name: the name of default configuration service
        @type config_name: C{str}
        @param items: list with descriptions
        @type items: C{[U{multimaster_msgs_fkie.srv.ListDescription<http://docs.ros.org/api/multimaster_msgs_fkie/html/srv/ListDescription.html>} Response]}
        '''
        if items:
            masteruri = self.masteruri
            if self.__master_info is not None:
                service = self.__master_info.getService(config_name)
                if service is not None:
                    masteruri = service.masteruri
            key = (roslib.names.namespace(config_name).rstrip(roslib.names.SEP), service_uri, masteruri)
            host = get_hostname(service_uri)
            host_addr = nm.nameres().address(host)
            # add capabilities
            caps = dict()
            for c in items[0].capabilities:
                if c.namespace not in caps:
                    caps[c.namespace] = dict()
                caps[c.namespace][c.name.decode(sys.getfilesystemencoding())] = {'type': c.type, 'images': [resolve_paths(i) for i in c.images], 'description': resolve_paths(c.description.replace("\\n ", "\n").decode(sys.getfilesystemencoding())), 'nodes': list(c.nodes)}
            if host_addr is None:
                host_addr = get_hostname(key[1])
            self.node_tree_model.addCapabilities(masteruri, host_addr, key, caps)
            # set host description
            tooltip = self.node_tree_model.updateHostDescription(masteruri, host_addr, items[0].robot_type, items[0].robot_name.decode(sys.getfilesystemencoding()), resolve_paths(items[0].robot_descr.decode(sys.getfilesystemencoding())))
            self.host_description_updated.emit(masteruri, host_addr, tooltip)
            self.capabilities_update_signal.emit(masteruri, host_addr, roslib.names.namespace(config_name).rstrip(roslib.names.SEP), items)

    def on_default_cfg_err(self, service_uri, service, msg):
        '''
        Handles the error messages from default configuration service.
        @param service_uri: the URI of the service provided the default configuration
        @type service_uri: C{str}
        @param service: the name of default configuration service
        @type service: C{str}
        @param msg: the error message
        @type msg: C{str}
        '''
        pass
#    MessageBox.warning(self, 'Error while call %s'%service,
#                              utf8(msg),
#                              buttons=MessageBox.Ok)

    @property
    def launch_servers(self):
        return self.__launch_servers

    def has_launch_server(self):
        '''
        Returns `True` if the there are roslaunch server, which have no `master` as
        node or or have other nodes as `rosout-#` inside.
        '''
        for _, (_, nodes) in self.__launch_servers.items():  # _:= uri, pid
            if not self._is_master_launch_server(nodes):
                return True
        return False

    def _is_master_launch_server(self, nodes):
        if 'master' in nodes and len(nodes) < 3:
            return True
        return False

    def on_launch_server_retrieved(self, serveruri, pid, nodes):
        '''
        Handles the info about roslaunch server.
        Emits a Qt signal L{host_description_updated} to notify about a new host
        description and a Qt signal L{capabilities_update_signal} to notify about a capabilities
        update.
        @param serveruri: the URI of the roslaunch server
        @type serveruri: C{str}
        @param pid: the process id of the roslaunch server
        @type pid: C{str}
        @param nodes: list with nodes handled by the roslaunch server
        @type nodes: C{[L{str}]}
        '''
        self.__launch_servers[serveruri] = (pid, nodes)

    def on_launch_server_err(self, serveruri, msg):
        '''
        Handles the error messages from launch server hanlder.
        @param serveruri: the URI of the launch server
        @type serveruri: C{str}
        @param msg: the error message
        @type msg: C{str}
        '''
        try:
            del self.__launch_servers[serveruri]
        except:
            pass

    def on_remove_all_launch_server(self):
        '''
        Kill all running launch server. The coresponding URIS are removed by master_monitor.
        '''
        for lsuri, (pid, nodes) in self.__launch_servers.items():
            try:
                if not self._is_master_launch_server(nodes):
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   ''.join(['kill roslaunch ', lsuri, '(', utf8(pid), ')']),
                                                   nm.starter().kill,
                                                   (get_hostname(lsuri), pid, False, self.current_user))
                    self.launch_server_handler.updateLaunchServerInfo(lsuri, delayed_exec=3.0)
            except Exception as e:
                rospy.logwarn("Error while kill roslaunch %s: %s", utf8(lsuri), utf8(e))
                raise DetailedError("Kill error",
                                    ''.join(['Error while kill roslaunch ', lsuri]),
                                    utf8(e))
        self._start_process_queue()

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Handling of the view activities                  %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def on_node_activated(self, index):
        '''
        Depending of the state of the node, it will be run or the screen output will
        be open.
        @param index: The index of the activated node
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        '''
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes(), False)
        if not selectedNodes:
            return
        has_running = False
        has_stopped = False
        has_invalid = False
        for node in selectedNodes:
            if node.uri is not None:
                has_running = True
                if node.pid is None:
                    has_invalid = True
            else:
                has_stopped = True

        if has_stopped:
            self.on_start_clicked()
        elif has_running and not has_invalid:
            self.on_io_clicked()
        else:
            self.on_log_clicked()

    def on_node_clicked(self, index):
        if time.time() - self.__last_selection > 1.:
            self.on_node_selection_changed(None, None, True)

    def on_topic_activated(self, index):
        '''
        @param index: The index of the activated topic
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        '''
        self.on_topic_echo_clicked()

    def on_topic_clicked(self, index):
        if time.time() - self.__last_selection > 1.:
            self.on_topic_selection_changed(None, None, True)

    def on_service_activated(self, index):
        '''
        @param index: The index of the activated service
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        '''
        self.on_service_call_clicked()

    def on_service_clicked(self, index):
        if time.time() - self.__last_selection > 1.:
            self.on_service_selection_changed(None, None, True)

    def on_host_inserted(self, item):
        if item == (self.masteruri, nm.nameres().hostname(get_hostname(self.masteruri))):
            index = self.node_tree_model.indexFromItem(item)
            model_index = self.node_proxy_model.mapFromSource(index)
            if model_index.isValid():
                self.masterTab.nodeTreeView.expand(model_index)
#        self.masterTab.nodeTreeView.expandAll()

    def on_node_collapsed(self, index):
        if not index.parent().isValid():
            self.masterTab.nodeTreeView.selectionModel().clear()

    def on_node_expanded(self, index):
        pass

    def _create_html_list(self, title, items, list_type=None, name=''):
        '''
        :param list_type: LAUNCH, TOPIC, NODE, SERVICE
        :type list_type: str
        '''
        result = ''
        if items:
            result = '%s<b><u>%s</u></b>' % (result, title)
            if len(items) > 1:
                result = '%s <span style="color:gray;">[%d]</span>' % (result, len(items))
            result = '%s<br><ul><span></span>' % result
            items.sort()
            for i in items:
                item = i
                # reduce the displayed name
                item_name = i
                if name:
                    if item_name.startswith(name):
                        item_name = item_name.replace('%s%s' % (name, roslib.names.SEP), '~', 1)
                    ns = roslib.names.namespace(name)
                    if item_name.startswith(ns) and ns != roslib.names.SEP:
                        item_name = item_name.replace(ns, '', 1)
                if list_type in ['NODE']:
                    item = '<a href="node://%s%s">%s</a>' % (self.mastername, i, item_name)
                elif list_type in ['TOPIC_PUB', 'TOPIC_SUB']:
                    # determine the count of publisher or subscriber
                    count = None
                    try:
                        tpc = self.__master_info.getTopic(i)
                        if list_type == 'TOPIC_SUB':
                            count = len(tpc.publisherNodes)
                            if name not in tpc.subscriberNodes:
                                count = None
                        else:
                            count = len(tpc.subscriberNodes)
                            if name not in tpc.publisherNodes:
                                count = None
                    except:
                        pass
                    # add the count
                    if count is not None:
                        item = '<a href="topic://%s">%s</a>' % (i, item_name)
                        item += '   <a href="topicecho://%s%s"><span style="color:gray;"><i>echo</i></span></a>' % (self.mastername, i)
                        item = '<span style="color:gray;">_%d_/ </span>%s' % (count, item)
                    else:
                        item = '<a>%s</a>' % (item_name)
                        item = '<span style="color:red;">!sync </span>%s' % (item)
                elif list_type == 'SERVICE':
                    try:
                        srv = self.__master_info.getService(i)
                        if name in srv.serviceProvider:
                            item = '<a href="service://%s%s">%s</a>' % (self.mastername, i, item_name)
                            item += '   <a href="servicecall://%s%s"><span style="color:gray;"><i>call</i></span></a>' % (self.mastername, i)
                        else:
                            item = '<span style="color:red;">!sync </span>%s' % (item_name)
                    except:
                        item = '<span style="color:red;">!sync </span>%s' % (item_name)
                elif list_type == 'LAUNCH':
                    item = '<a href="launch://%s">%s</a>' % (i, item_name)
                    if i in self.__configs and self.masteruri in self.__configs[i].global_param_done:
                        item += '%s<br><a href="reload-globals://%s"><font color="#339900">reload global parameter @next start</font></a>' % (item, i)
                result += '\n%s<br>' % (item)
            result += '</ul>'
        return result

    def on_tab_current_changed(self, index):
        tab_name = self.masterTab.tabWidget.currentWidget().objectName()
        if tab_name == 'tabTopics':
            # select the topics of the selected node in the "Topic" view
            selections = self.masterTab.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                selected_topics = self.topic_model.index_from_names(node.published, node.subscribed)
                for s in selected_topics:
                    self.masterTab.topicsView.selectionModel().select(self.topic_proxyModel.mapFromSource(s), QItemSelectionModel.Select)
        elif tab_name == 'tabServices':
            # select the services of the selected node in the "Services" view
            selections = self.masterTab.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                selected_services = self.service_model.index_from_names(node.services)
                for s in selected_services:
                    self.masterTab.servicesView.selectionModel().select(self.service_proxyModel.mapFromSource(s), QItemSelectionModel.Select)

    def _is_current_tab_name(self, tab_name):
        return (self.masterTab.tabWidget.currentWidget().objectName() == tab_name)

    def on_node_selection_changed(self, selected, deselected, force_emit=False, node_name=''):
        '''
        updates the Buttons, create a description and emit L{description_signal} to
        show the description of host, group or node.
        '''
        if selected is not None:
            # it is a workaround to avoid double updates a after click on an item
            self.__last_selection = time.time()
        selectedGroups = []
        if node_name and self.master_info is not None:
            # get node by name
            selectedNodes = self.getNode(node_name)
            if not selectedNodes or selectedNodes[0] is None:
                if node_name:
                    self.description_signal.emit(node_name, "<b>%s</b> not found" % node_name, True if selected or deselected or force_emit else False)
                return
            selectedHosts = []
            selections = []
        else:
            # get node by selected items
            if not self._is_current_tab_name('tabNodes'):
                return
            selections = self.masterTab.nodeTreeView.selectionModel().selectedIndexes()
            selectedHosts = self.hostsFromIndexes(selections)
            selectedNodes = self.nodesFromIndexes(selections)
            selectedGroups = self.groupsFromIndexes(selections)
        self.masterTab.topicsView.selectionModel().clear()
        self.masterTab.servicesView.selectionModel().clear()
        name = ''
        text = ''
        # add control buttons for more then one selected node
        if len(selectedNodes) > 1 or selectedGroups > 0:
            restartable_nodes = [sn for sn in selectedNodes if len(sn.cfgs) > 0 and not self._is_in_ignore_list(sn.name)]
            restartable_nodes_with_launchfiles = [sn for sn in selectedNodes if sn.has_launch_cfgs(sn.cfgs) > 0 and not self._is_in_ignore_list(sn.name)]
            killable_nodes = [sn for sn in selectedNodes if sn.node_info.pid is not None and not self._is_in_ignore_list(sn.name)]
            unregisterble_nodes = [sn for sn in selectedNodes if sn.node_info.pid is None and sn.node_info.uri is not None and sn.node_info.isLocal and not self._is_in_ignore_list(sn.name)]
            # add description for multiple selected nodes
            if restartable_nodes or killable_nodes or unregisterble_nodes:
                text += '<b>Selected nodes:</b><br>'
            if restartable_nodes:
                text += '<a href="restart-node://all_selected_nodes" title="Restart %s selected nodes"><img src=":icons/sekkyumu_restart_24.png" alt="restart">[%d]</a>' % (len(restartable_nodes), len(restartable_nodes))
                text += '&nbsp;<a href="restart-node-g://all_selected_nodes" title="Reload global parameter and restart %s selected nodes"><img src=":icons/sekkyumu_restart_g_24.png" alt="restart">[%d]</a>' % (len(restartable_nodes), len(restartable_nodes))
            if killable_nodes:
                # text += '&nbsp;<a href="kill-node://all_selected_nodes" title="Kill %s selected nodes"><img src=":icons/sekkyumu_kill_24.png" alt="kill">[%d]</a>' % (len(killable_nodes), len(killable_nodes))
                text += '&nbsp;<a href="kill-screen://all_selected_nodes" title="Kill %s screens of selected nodes"><img src=":icons/sekkyumu_kill_screen_24.png" alt="killscreen">[%d]</a>' % (len(killable_nodes), len(killable_nodes))
            if restartable_nodes_with_launchfiles:
                text += '&nbsp;<a href="start-node-at-host://all_selected_nodes" title="Start %s nodes at another host"><img src=":icons/sekkyumu_start_athost_24.png" alt="start@host">[%d]</a>' % (len(restartable_nodes_with_launchfiles), len(restartable_nodes_with_launchfiles))
                text += '&nbsp;<a href="start-node-adv://all_selected_nodes" title="Start %s nodes with additional options, e.g. loglevel"><img src=":icons/sekkyumu_play_alt_24.png" alt="play alt">[%d]</a>' % (len(restartable_nodes_with_launchfiles), len(restartable_nodes_with_launchfiles))
            if unregisterble_nodes:
                text += '<br><a href="unregister-node://all_selected_nodes">unregister [%d]</a>' % len(unregisterble_nodes)
        # add host description, if only the one host is selected
        if len(selectedHosts) == 1:  # and len(selections) / 2 == 1:
            host = selectedHosts[0]
            name = '%s - Robot' % host.name
            text += host.generateDescription()
            text += '<br>'
        else:
            # add group description, if only the one group is selected
            if len(selectedGroups) == 1 and len(selections) / 2 == 1:
                group = selectedGroups[0]
                name = '%s - Group' % group.name
                text += group.generateDescription()
                text += '<br>'
        # add node description for one selected node
        if len(selectedHosts) != 1 and len(selectedNodes) == 1 and len(selectedGroups) == 0:
            node = selectedNodes[0]
            text = self.get_node_description(node_name, node)
            name = node.name
        if (self._is_current_tab_name('tabNodes') and self.__last_info_text != text) or force_emit:
            self.__last_info_text = text
            self.description_signal.emit(name, text, True if selected or deselected or force_emit else False)
        self.updateButtons()

    def get_node_description(self, node_name, node=None):
        text = ''
        if node_name and node is None and self.master_info is not None:
            # get node by name
            selectedNodes = self.getNode(node_name)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
        # add node description for one selected node
        if node is not None:
            # create description for a node
            ns, sep, name = node.name.rpartition(rospy.names.SEP)
            text = '<font size="+1"><b><span style="color:gray;">%s%s</span><b>%s</b></font><br>' % (ns, sep, name)
            launches = [c for c in node.cfgs if not isinstance(c, tuple)]
            default_cfgs = [c[0] for c in node.cfgs if isinstance(c, tuple)]
            if launches or default_cfgs:
                text += '<a href="restart-node://%s" title="Restart node"><img src=":icons/sekkyumu_restart_24.png" alt="restart"></a>' % node.name  # height="24" width="24"
                text += '&nbsp;<a href="restart-node-g://%s" title="Reload global parameter and restart node"><img src=":icons/sekkyumu_restart_g_24.png" alt="restart"></a>' % node.name  # height="24" width="24"
            # text += '&nbsp; <a href="kill-node://%s" title="Kill node with pid %s"><img src=":icons/sekkyumu_kill_24.png" alt="kill"></a>' % (node.name, node.pid)
            text += '&nbsp; <a href="kill-screen://%s" title="Kill screen of the node"><img src=":icons/sekkyumu_kill_screen_24.png" alt="killscreen"></a>' % node.name
            if launches:
                text += '&nbsp; <a href="start-node-at-host://%s"  title="Start node at another host"><img src=":icons/sekkyumu_start_athost_24.png" alt="start@host"></a>' % node.name
#        if node.node_info.pid is None or node.node_info.uri is None:
                text += '&nbsp; <a href="start-node-adv://%s" title="Start node with additional options, e.g. loglevel"><img src=":icons/sekkyumu_play_alt_24.png" alt="play alt"></a>' % node.name
            text += '&nbsp; <a href="copy-log-path://%s" title="copy log path to clipboard"><img src=":icons/crystal_clear_copy_log_path_24.png" alt="copy_log_path"></a>' % node.name
            text += '<dl>'
            text += '<dt><b>URI</b>: %s</dt>' % node.node_info.uri
            text += '<dt><b>PID</b>: %s</dt>' % node.node_info.pid
            text += '<dt><b>ORG.MASTERURI</b>: %s</dt>' % node.node_info.masteruri
            if not is_legal_name(node.name):
                text += '<dt><font color="#FF6600"><b>This node has an illegal <node> name.<br><a href="http://ros.org/wiki/Names">http://ros.org/wiki/Names</a><br>This will likely cause problems with other ROS tools.</b></font></dt>'
            if node.is_ghost:
                if node.name.endswith('master_sync') or node.name.endswith('node_manager'):
                    text += '<dt><font color="#FF9900"><b>This node is not synchronized by default. To get info about this node select the related host.</b></font></dt>'
                else:
                    text += '<dt><font color="#FF9900"><b>The node is running on remote host, but is not synchronized, because of filter or errors while sync, see log of <i>master_sync</i></b></font></dt>'
                    text += '<dt><font color="#FF9900"><i>Are you use the same ROS packages?</i></font></dt>'
            if node.has_running and node.node_info.pid is None and node.node_info.uri is None:
                text += '<dt><font color="#FF9900"><b>There are nodes with the same name on remote hosts running. These will be terminated, if you run this node! (Only if master_sync is running or will be started somewhere!)</b></font></dt>'
            if node.node_info.uri is not None and node.node_info.masteruri != self.masteruri:
                text += '<dt><font color="#339900"><b>synchronized</b></font></dt>'
            if node.node_info.pid is None and node.node_info.uri is not None:
                if not node.node_info.isLocal:
                    text += '<dt><font color="#FF9900"><b>remote nodes will not be ping, so they are always marked running</b></font>'
                else:
                    text += '<dt><font color="#CC0000"><b>the node does not respond: </b></font>'
                    text += '<a href="unregister-node://%s">unregister</a></dt>' % node.name
            if node.diagnostic_array and node.diagnostic_array[-1].level > 0:
                diag_status = node.diagnostic_array[-1]
                level_str = self.DIAGNOSTIC_LEVELS[diag_status.level]
                diag_color = '#FF6600'
                if diag_status.level == 2:
                    diag_color = '#CC0000'
                elif diag_status.level == 3:
                    diag_color = '#FFCC00'
                elif diag_status.level > 3:
                    diag_color = '#0000CC'
                text += '<dt><font color="%s"><b>%s: %s</b></font></dt>' % (diag_color, level_str, node.diagnostic_array[-1].message)
#        if len(node.diagnostic_array) > 1:
#          text += '<dt><font color="#FF6600"><a href="view_diagnostics://%s">view recent %d items</a></font></dt>'%(node.name, len(node.diagnostic_array))
            text += '</dl>'
            if nm.settings().transpose_pub_sub_descr:
                text += self._create_html_list('Subscribed Topics:', node.subscribed, 'TOPIC_SUB', node.name)
                text += self._create_html_list('Published Topics:', node.published, 'TOPIC_PUB', node.name)
            else:
                text += self._create_html_list('Published Topics:', node.published, 'TOPIC_PUB', node.name)
                text += self._create_html_list('Subscribed Topics:', node.subscribed, 'TOPIC_SUB', node.name)
            text += self._create_html_list('Services:', node.services, 'SERVICE', node.name)
            # set loaunch file paths
            text += self._create_html_list('Loaded Launch Files:', launches, 'LAUNCH')
            text += self._create_html_list('Default Configurations:', default_cfgs, 'NODE')
#      text += '<dt><a href="copy-log-path://%s">copy log path to clipboard</a></dt>'%node.name
        return text

    def on_topic_selection_changed(self, selected, deselected, force_emit=False, topic_name=''):
        '''
        updates the Buttons, create a description and emit L{description_signal} to
        show the description of selected topic
        '''
        if selected is not None:
            # it is a workaround to avoid double updates a after click on an item
            self.__last_selection = time.time()
        selectedTopics = []
        if topic_name and self.master_info is not None:
            selectedTopics = [self.master_info.getTopic("%s" % topic_name)]
            if len(selectedTopics) == 0:
                return
        else:
            if not self._is_current_tab_name('tabTopics'):
                return
            selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
            topics_selected = (len(selectedTopics) > 0)
            self.masterTab.echoTopicButton.setEnabled(topics_selected)
            self.masterTab.hzTopicButton.setEnabled(topics_selected)
            self.masterTab.hzSshTopicButton.setEnabled(topics_selected)
            self.masterTab.pubStopTopicButton.setEnabled(topics_selected)
        if len(selectedTopics) == 1:
            topic = selectedTopics[0]
            text = self.get_topic_description(topic_name, topic)
            info_text = '<div>%s</div>' % text
            if (self._is_current_tab_name('tabTopics') and self.__last_info_text != info_text) or force_emit:
                self.__last_info_text = info_text
                self.description_signal.emit(topic.name, info_text, True if selected or deselected or force_emit else False)

    def get_topic_description(self, topic_name, topic=None):
        text = ''
        if topic is None:
            selectedTopics = []
            if topic_name and self.master_info is not None:
                selectedTopics = [self.master_info.getTopic("%s" % topic_name)]
            else:
                selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
            if len(selectedTopics) == 1:
                topic = selectedTopics[0]
        if topic is not None:
            ns, sep, name = topic.name.rpartition(rospy.names.SEP)
            text = '<font size="+1"><b><span style="color:gray;">%s%s</span><b>%s</b></font><br>' % (ns, sep, name)
            text += '&nbsp;<a href="topicecho://%s%s" title="Show the content of the topic"><img src=":icons/sekkyumu_topic_echo_24.png" alt="echo"></a>' % (self.mastername, topic.name)
            text += '&nbsp;<a href="topichz://%s%s" title="Show only the receive rate of the topic.<br>All data is sent through the network"><img src=":icons/sekkyumu_topic_hz_24.png" alt="hz"></a>' % (self.mastername, topic.name)
            text += '&nbsp;<a href="topichzssh://%s%s" title="Show only the receive rate of the topic.<br>Uses an SSH connection to execute `rostopic hz` on remote host."><img src=":icons/sekkyumu_topic_echo_ssh_hz_24.png" alt="sshhz"></a>' % (self.mastername, topic.name)
            text += '&nbsp;<a href="topicpub://%s%s" title="Start a publisher for selected topic"><img src=":icons/sekkyumu_topic_pub_24.png" alt="pub"></a>' % (self.mastername, topic.name)
            if topic.name in self.__republish_params:
                text += '&nbsp;<a href="topicrepub://%s%s" title="Start a publisher with last parameters"><img src=":icons/sekkyumu_topic_repub_24.png" alt="repub"></a>' % (self.mastername, topic.name)
            topic_publisher = []
            topic_prefix = '/rostopic_pub%s_' % topic.name
            node_names = self.master_info.node_names
            for n in node_names:
                if n.startswith(topic_prefix):
                    topic_publisher.append(n)
            if topic_publisher:
                text += '&nbsp;<a href="topicstop://%s%s"><img src=":icons/sekkyumu_topic_pub_stop_24.png" alt="stop"> [%d]</a>' % (self.mastername, topic.name, len(topic_publisher))
            text += '<p>'
            if nm.settings().transpose_pub_sub_descr:
                text += self._create_html_list('Subscriber:', topic.subscriberNodes, 'NODE')
                text += self._create_html_list('Publisher:', topic.publisherNodes, 'NODE')
            else:
                text += self._create_html_list('Publisher:', topic.publisherNodes, 'NODE')
                text += self._create_html_list('Subscriber:', topic.subscriberNodes, 'NODE')
            text += '<b><u>Type:</u></b> %s' % self._href_from_msgtype(topic.type)
            text += '<dl>'
            try:
                mclass = roslib.message.get_message_class(topic.type)
                if mclass is not None:
                    for f in mclass.__slots__:
                        idx = mclass.__slots__.index(f)
                        idtype = mclass._slot_types[idx]
#            base_type = roslib.msgs.base_msg_type(idtype)
#            primitive = "unknown"
#            if base_type in roslib.msgs.PRIMITIVE_TYPES:
#              primitive = "primitive"
#            else:
#              try:
#                primitive =roslib.message.get_message_class(base_type)
# #                primitive = "class", list_msg_class.__slots__
#              except ValueError:
#                pass
                        text += '%s: <span style="color:gray;">%s</span><br>' % (f, idtype)
                    text += '<br>'
                    constants = {}
                    for m in dir(mclass):
                        if not m.startswith('_'):
                            if type(getattr(mclass, m)) in [str, int, bool, float]:
                                constants[m] = getattr(mclass, m)
                    if constants:
                        text += '<b><u>Constants:</u></b><br>'
                        for n in sorted(constants.iterkeys()):
                            text += '%s: <span style="color:gray;">%s</span><br>' % (n, constants[n])
            except ValueError:
                pass
            text += '</dl>'
        return text

    def _href_from_msgtype(self, msg_type):
        result = msg_type
        if msg_type:
            result = '<a href="http://ros.org/doc/api/%s.html">%s</a>' % (msg_type.replace('/', '/html/msg/'), msg_type)
        return result

    def on_service_selection_changed(self, selected, deselected, force_emit=False, service_name=''):
        '''
        updates the Buttons, create a description and emit L{description_signal} to
        show the description of selected service
        '''
        if selected is not None:
            # it is a workaround to avoid double updates a after click on an item
            self.__last_selection = time.time()
        if service_name and self.master_info is not None:
            # get service by name
            selectedServices = [self.master_info.getService("%s" % service_name)]
            if selectedServices[0] is None:
                return
        else:
            # get service by selected items
            selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
            self.masterTab.callServiceButton.setEnabled(len(selectedServices) > 0)
            if not self._is_current_tab_name('tabServices'):
                return
        if len(selectedServices) == 1:
            service = selectedServices[0]
            ns, sep, name = service.name.rpartition(rospy.names.SEP)
            text = '<font size="+1"><b><span style="color:gray;">%s%s</span><b>%s</b></font><br>' % (ns, sep, name)
            text += '<a href="servicecall://%s%s" title="call service"><img src=":icons/sekkyumu_call_service_24.png" alt="call"></a>' % (self.mastername, service.name)
            text += '<dl>'
            text += '<dt><b>URI</b>: %s</dt>' % service.uri
            text += '<dt><b>ORG.MASTERURI</b>: %s</dt>' % service.masteruri
            text += self._create_html_list('Provider:', service.serviceProvider, 'NODE')
            if service.masteruri != self.masteruri:
                text += '<dt><font color="#339900"><b>synchronized</b></font></dt>'
            text += '</dl>'
            try:
                service_class = service.get_service_class(nm.is_local(get_hostname(service.uri)))
                text += '<h4>%s</h4>' % self._href_from_svrtype(service_class._type)
                text += '<b><u>Request:</u></b>'
                text += '<dl><dt>%s</dt></dl>' % service_class._request_class.__slots__
                text += '<b><u>Response:</u></b>'
                text += '<dl><dt>%s</dt></dl>' % service_class._response_class.__slots__
            except:
                text += '<h4><font color=red>Unknown service type</font></h4>'
                if service.isLocal:
                    text += '<font color=red>Unable to communicate with service, is provider node running?</font>'
                else:
                    text += '<font color=red>Try to refresh <b>all</b> hosts. Is provider node running?</font>'
            info_text = '<div>%s</div>' % text
            self._is_current_tab_name('tabServices')
            if (self._is_current_tab_name('tabServices') and self.__last_info_text != info_text) or force_emit:
                self.__last_info_text = info_text
                self.description_signal.emit(service.name, info_text, True if selected or deselected or force_emit else False)

    def _href_from_svrtype(self, srv_type):
        result = srv_type
        if srv_type:
            result = '<a href="http://ros.org/doc/api/%s.html">%s</a>' % (srv_type.replace('/', '/html/srv/'), srv_type)
        return result

    def on_parameter_selection_changed(self, selected, deselected):
        selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
        self.masterTab.deleteParameterButton.setEnabled(len(selectedParameter) > 0)
        self.masterTab.saveParameterButton.setEnabled(len(selectedParameter) > 0)

    def hostsFromIndexes(self, indexes, recursive=True):
        result = []
        for index in indexes:
            if index.column() == 0:
                model_index = self.node_proxy_model.mapToSource(index)
                item = self.node_tree_model.itemFromIndex(model_index)
                if item is not None:
                    if isinstance(item, HostItem):
                        result.append(item)
        return result

    def groupsFromIndexes(self, indexes, recursive=True):
        result = []
        for index in indexes:
            if index.column() == 0 and index.parent().isValid():
                model_index = self.node_proxy_model.mapToSource(index)
                item = self.node_tree_model.itemFromIndex(model_index)
                if item is not None:
                    if isinstance(item, GroupItem):
                        result.append(item)
        return result

    def nodesFromIndexes(self, indexes, recursive=True):
        result = []
        for index in indexes:
            if index.column() == 0:
                model_index = self.node_proxy_model.mapToSource(index)
                item = self.node_tree_model.itemFromIndex(model_index)
                res = self._nodesFromItems(item, recursive)
                for r in res:
                    if r not in result:
                        result.append(r)
        return result

    def _nodesFromItems(self, item, recursive):
        result = []
        if item is not None:
            if isinstance(item, (GroupItem, HostItem)):
                if recursive:
                    for j in range(item.rowCount()):
                        result[len(result):] = self._nodesFromItems(item.child(j), recursive)
            elif isinstance(item, NodeItem):
                if item not in result:
                    result.append(item)
        return result

    def topicsFromIndexes(self, indexes):
        result = []
        for index in indexes:
            model_index = self.topic_proxyModel.mapToSource(index)
            item = self.topic_model.itemFromIndex(model_index)
            if item is not None and isinstance(item, TopicItem):
                result.append(item.topic)
        return result

    def servicesFromIndexes(self, indexes):
        result = []
        for index in indexes:
            model_index = self.service_proxyModel.mapToSource(index)
            item = self.service_model.itemFromIndex(model_index)
            if item is not None and isinstance(item, ServiceItem):
                result.append(item.service)
        return result

    def parameterFromIndexes(self, indexes):
        result = []
        for index in indexes:
            model_index = self.parameter_proxyModel.mapToSource(index)
            item = self.parameter_model.itemFromIndex(model_index)
            if item is not None and isinstance(item, ParameterValueItem):
                result.append((item.name, item.value))
        return result

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Handling of the button activities                %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def on_start_clicked(self):
        '''
        Starts the selected nodes. If for a node more then one configuration is
        available, the selection dialog will be show.
        '''
        cursor = self.cursor()
        self.masterTab.startButton.setEnabled(False)
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            self.start_nodes(selectedNodes)
        finally:
            self.setCursor(cursor)
            self.masterTab.startButton.setEnabled(True)

    def on_start_alt_clicked(self):
        '''
        Starts the selected nodes with additional options.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            self.start_nodes(selectedNodes, force=True, use_adv_cfg=True)
        finally:
            self.setCursor(cursor)

    def start_node(self, node, force, config, force_host=None, logging=None):

        if node is None:
            raise DetailedError("Start error", 'None is not valid node name!')
        if node.pid is None or force:
            # start the node using launch configuration
            if config is None:
                raise DetailedError("Start error",
                                    'Error while start %s:\nNo configuration found!' % node.name)
            if isinstance(config, LaunchConfig):
                try:
                    nm.starter().runNode(AdvRunCfg(node.name, config, force_host, self.masteruri, logging=logging, user=self.current_user))
                except socket.error as se:
                    rospy.logwarn("Error while start '%s': %s\n\n Start canceled!", node.name, utf8(se))
                    raise DetailedError("Start error",
                                        'Error while start %s\n\nStart canceled!' % node.name,
                                        '%s' % utf8(se))
                    return False
                except nm.InteractionNeededError as _:
                    raise
                except (Exception, nm.StartException) as e:
                    print type(e)
                    print traceback.format_exc(3)
                    rospy.logwarn("Error while start '%s': %s" % (node.name, utf8(e)))
                    raise DetailedError("Start error", 'Error while start %s' % node.name, '%s' % utf8(e))
            elif isinstance(config, (str, unicode)):
                # start with default configuration
                from multimaster_msgs_fkie.srv import Task
                try:
                    nm.starter().callService(self.master_info.getService(config).uri, config, Task, [node.name])
                except (Exception, nm.StartException) as e:
                    rospy.logwarn("Error while call a service of node '%s': %s" % (node.name, utf8(e)))
                    raise DetailedError("Service error",
                                        'Error while call a service of node %s [%s]' % (node.name, self.master_info.getService(config).uri),
                                        '%s' % utf8(e))

    def start_nodes(self, nodes, force=False, force_host=None, use_adv_cfg=False):
        '''
        Internal method to start a list with nodes
        @param nodes: the list with nodes to start
        @type nodes: C{[L{NodeItem}, ...]}
        @param force: force the start of the node, also if it is already started.
        @type force: C{bool}
        @param force_host: force the start of the node at specified host.
        @type force_host: C{str}
        '''
        cfg_choices = dict()
        cfg_nodes = dict()
        has_launch_files = False
        for node in nodes:
            # do not start node, if it is in ingnore list and multiple nodes are selected
            if (node.pid is None or (node.pid is not None and force)) and not node.is_ghost:
                # test for duplicate nodes
                if node.uri is None and node.has_running:
                    ret = MessageBox.question(self, 'Question', ''.join(['Some nodes, e.g. ', node.name, ' are already running on another host. If you start this node the other node will be terminated.\n Do you want proceed?']), buttons=MessageBox.Yes | MessageBox.No)
                    if ret == MessageBox.No:
                        return
                # determine the used configuration
                if node.next_start_cfg is not None:
                    lcfg = node.next_start_cfg
                    if node.next_start_cfg in self.launchfiles:
                        lcfg = self.launchfiles[node.next_start_cfg]
                    cfg_nodes[node.name] = lcfg
                    node.launched_cfg = lcfg
                    node.next_start_cfg = None
                else:
                    choices = self._getCfgChoises(node)
                    ch_keys = choices.keys()
                    if ch_keys:
                        ch_keys.sort()
                        choises_str = utf8(ch_keys)
                        if choises_str not in cfg_choices.keys():
                            choice, ok = self._getUserCfgChoice(choices, node.name)
                            if choice is not None:
                                cfg_choices[choises_str] = choices[choice]
                                cfg_nodes[node.name] = choices[choice]
                                node.launched_cfg = choices[choice]
                                if isinstance(choices[choice], LaunchConfig):
                                    has_launch_files = True
                            elif ok:
                                MessageBox.warning(self, "Start error",
                                                   'Error while start %s:\nNo configuration selected!' % node.name)
                            else:
                                break
                        else:
                            cfg_nodes[node.name] = cfg_choices[choises_str]
                            node.launched_cfg = cfg_choices[choises_str]

        # get the advanced configuration
        logging = None
        diag_canceled = False
        if use_adv_cfg and has_launch_files:
            log_params = {'Level': ('string', nm.settings().logging.get_alternatives('loglevel')),
                          'Level (roscpp)': ('string', nm.settings().logging.get_alternatives('loglevel_roscpp')),
                          'Level (super)': ('string', nm.settings().logging.get_alternatives('loglevel_superdebug')),
                          'Format': ('string', nm.settings().logging.get_alternatives('console_format'))
                          }
            params = {'Logging': ('dict', log_params)}
            dia = ParameterDialog(params)
            dia.setFilterVisible(False)
            dia.setWindowTitle('Start with parameters')
            dia.resize(480, 120)
            dia.setFocusField('Level')
            diag_canceled = not dia.exec_()
            if not diag_canceled:
                try:
                    params = dia.getKeywords()
                    nm.settings().logging.loglevel = params['Logging']['Level']
                    nm.settings().logging.loglevel_roscpp = params['Logging']['Level (roscpp)']
                    nm.settings().logging.loglevel_superdebug = params['Logging']['Level (super)']
                    nm.settings().logging.console_format = params['Logging']['Format']
                    nm.settings().store_logging()
                    logging = nm.settings().logging
                except Exception, e:
                    diag_canceled = True
                    MessageBox.warning(self, "Get advanced start parameter",
                                       'Error while parse parameter',
                                       utf8(e))
        if not diag_canceled:
            # put into the queue and start
            for node in nodes:
                if node.name in cfg_nodes:
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   ''.join(['start ', node.node_info.name]),
                                                   self.start_node,
                                                   (node.node_info, force, cfg_nodes[node.node_info.name], force_host, logging))
        self._start_process_queue()

    def start_nodes_by_name(self, nodes, cfg, force=False):
        '''
        Start nodes given in a list by their names.
        @param nodes: a list with full node names
        @type nodes: C{[str]}
        '''
        result = []
        if self.master_info is not None:
            for n in nodes:
                node_items = self.getNode(n)
                if node_items:
                    node_item = node_items[0]
#                    node_item.addConfig(cfg)
                    if isinstance(cfg, tuple):
                        node_item.next_start_cfg = cfg[0]
                    else:
                        node_item.next_start_cfg = cfg
                elif cfg:
                    node_info = NodeInfo(n, self.masteruri)
                    node_item = NodeItem(node_info)
#                    node_item.addConfig(cfg)
                    if isinstance(cfg, tuple):
                        node_item.next_start_cfg = cfg[0]
                    else:
                        node_item.next_start_cfg = cfg
                if node_item is not None:
                    result.append(node_item)
        self.start_nodes(result, force)

    def start_nodes_after_load_cfg(self, cfg_name, nodes, force=False):
        '''
        Start nodes after the given configuration is loaded and applied to the model.
        :param cfg_name: the name of the cnofiguration
        :type cfg_name: str
        :param nodes: the list of node names
        :type nodes: list of strings
        '''
        if cfg_name not in self._start_nodes_after_load_cfg:
            self._start_nodes_after_load_cfg[cfg_name] = set(nodes)
        else:
            self._start_nodes_after_load_cfg[cfg_name].update(set(nodes))

    def start_nodes_after_load_cfg_clear(self):
        '''
        Clears the list with nodes which should be startet after a launch file is loaded.
        '''
        self._start_nodes_after_load_cfg = dict()

    def on_force_start_nodes(self, reset_global_param=False):
        '''
        Starts the selected nodes (also if it already running). If for a node more then one configuration is
        available, the selection dialog will be show.
        '''
        cursor = self.cursor()
        self.masterTab.startButton.setEnabled(False)
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            self.stop_nodes(selectedNodes)
            if reset_global_param:
                # reset config to load global parameter
                for node in selectedNodes:
                    for cfg in node.cfgs:
                        if cfg in self.launchfiles:
                            self.reload_global_parameter_at_next_start(cfg)
            self.start_nodes(selectedNodes, True)
        finally:
            self.setCursor(cursor)
            self.masterTab.startButton.setEnabled(True)

    def on_start_nodes_at_host(self):
        '''
        Starts the selected nodes on an another host.
        '''
        cursor = self.cursor()
        self.masterTab.startButton.setEnabled(False)
        params = {'Host': ('string', 'localhost')}
        dia = ParameterDialog(params)
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start node on...')
        dia.resize(350, 120)
        dia.setFocusField('host')
        if dia.exec_():
            try:
                params = dia.getKeywords()
                host = params['Host']
                self.setCursor(Qt.WaitCursor)
                try:
                    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
                    self.start_nodes(selectedNodes, True, host)
                finally:
                    self.setCursor(cursor)
            except Exception, e:
                MessageBox.warning(self, "Start error",
                                   'Error while parse parameter',
                                   utf8(e))
        self.masterTab.startButton.setEnabled(True)

    def _getDefaultCfgChoises(self, node):
        result = {}
        for c in node.cfgs:
            if isinstance(c, tuple):
                result[' '.join(['[default]', c[0]])] = roslib.names.ns_join(c[0], 'run')
        return result

    def _getCfgChoises(self, node, ignore_defaults=False):
        result = {}
        for c in node.cfgs:
            if c:
                if not isinstance(c, tuple):
                    launch = self.launchfiles[c]
                    result[''.join([utf8(launch.LaunchName), ' [', utf8(launch.PackageName), ']'])] = self.launchfiles[c]
                elif not ignore_defaults:
                    result[' '.join(['[default]', c[0]])] = roslib.names.ns_join(c[0], 'run')
        return result

    def _getUserCfgChoice(self, choices, nodename):
        value = None
        ok = False
        # Open selection
        if len(choices) == 1:
            value = choices.keys()[0]
            ok = True
        elif len(choices) > 0:
            items, ok = SelectDialog.getValue('Configuration selection', 'Select configuration to launch <b>%s</b>' % nodename, choices.keys(), True)
            if items:
                value = items[0]
        return value, ok

    def on_stop_clicked(self):
        '''
        Stops the selected and running nodes. If the node can't be stopped using his
        RPC interface, it will be unregistered from the ROS master using the masters
        RPC interface.
        '''
        key_mod = QApplication.keyboardModifiers()
        if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
            self.masterTab.stopButton.showMenu()
        else:
            cursor = self.cursor()
            self.setCursor(Qt.WaitCursor)
            try:
                selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
                self.stop_nodes(selectedNodes)
            finally:
                self.setCursor(cursor)

    def stop_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            try:
                rospy.loginfo("Stop node '%s'[%s]", utf8(node.name), utf8(node.uri))
                nm.filewatcher().rem_binary(node.name)
                # 'print "STOP set timeout", node
                socket.setdefaulttimeout(10)
                # 'print "STOP create xmlrpc", node
                p = xmlrpclib.ServerProxy(node.uri)
                # 'print "STOP send stop", node
                p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', self.mastername]))
                # 'print "STOP stop finished", node
            except Exception, e:
                rospy.logwarn("Error while stop node '%s': %s", utf8(node.name), utf8(e))
                if utf8(e).find(' 111') == 1:
                    raise DetailedError("Stop error",
                                        ''.join(['Error while stop node ', node.name]),
                                        utf8(e))
            finally:
                socket.setdefaulttimeout(None)
        elif isinstance(node, NodeItem) and node.is_ghost:
            # since for ghost nodes no info is available, emit a signal to handle the
            # stop message in other master_view_proxy
            self.stop_nodes_signal.emit(node.masteruri, [node.name])
        return True

    def stop_nodes(self, nodes, force=False):
        '''
        Internal method to stop a list with nodes
        @param nodes: the list with nodes to stop
        @type nodes: C{[U{master_discovery_fkie.NodeInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#master_discovery_fkie.master_info.NodeInfo>}, ...]}
        '''
        # put into the queue and start the que handling
        for node in nodes:
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'stop %s' % node.name,
                                           self.stop_node,
                                           (node, (len(nodes) == 1) or force))
        self._start_process_queue()

    def stop_nodes_by_name(self, nodes, force=False, ignore=[]):
        '''
        Stop nodes given in a list by their names.
        @param nodes: a list with full node names
        @type nodes: C{[str]}
        '''
        result = []
        if self.master_info is not None:
            for n in nodes:
                if n not in ignore:
                    node = self.master_info.getNode(n)
                    if node is not None:
                        result.append(node)
        self.stop_nodes(result, force)

    def kill_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            pid = node.pid
            if pid is None:
                # try to get the process id of the node
                try:
                    socket.setdefaulttimeout(10)
                    rpc_node = xmlrpclib.ServerProxy(node.uri)
                    _, _, pid = rpc_node.getPid(rospy.get_name())  # _:=code, msg
                except:
                    pass
                finally:
                    socket.setdefaulttimeout(None)
            # kill the node
            if pid is not None:
                try:
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   ''.join(['kill ', node.name, '(', utf8(pid), ')']),
                                                   nm.starter().kill,
                                                   (self.getHostFromNode(node), pid, False, self.current_user))
                    self._start_process_queue()
                except Exception as e:
                    rospy.logwarn("Error while kill the node %s: %s", utf8(node.name), utf8(e))
                    raise DetailedError("Kill error",
                                        ''.join(['Error while kill the node ', node.name]),
                                        utf8(e))
        return True

    def killall_roscore(self):
        host = get_hostname(self.masteruri)
        if host:
            try:
                if not nm.is_local(self.mastername):
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   'killall roscore on %s' % host,
                                                   nm.starter().killall_roscore,
                                                   (host, self.current_user))
                    self._start_process_queue()
                else:
                    nm.starter().killall_roscore(host, self.current_user)
            except Exception as e:
                rospy.logwarn("Error while killall roscore on %s: %s" % (host, utf8(e)))
                raise DetailedError("Killall roscore error",
                                    'Error while killall roscore',
                                    '%s' % utf8(e))
        return True

    def on_kill_nodes(self):
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())

        # put into the queue and start the que handling
        for node in selectedNodes:
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           ''.join(['kill ', node.name]),
                                           self.kill_node,
                                           (node, (len(selectedNodes) == 1)))
        self._start_process_queue()

    def unregister_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            # stop the node?
            # try:
            #   p = xmlrpclib.ServerProxy(node.uri)
            #   p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', self.hostname]))
            # except Exception, e:
            #   rospy.logwarn("Error while stop node '%s': %s", utf8(node.name), utf8(e))
            #   self.masterTab.stopButton.setEnabled(False)
            # unregister all entries of the node from ROS master
            try:
                socket.setdefaulttimeout(10)
                master = xmlrpclib.ServerProxy(node.masteruri)
                master_multi = xmlrpclib.MultiCall(master)
#        master_multi.deleteParam(node.name, node.name)
                for p in node.published:
                    rospy.loginfo("unregister publisher '%s' [%s] from ROS master: %s", p, node.name, node.masteruri)
                    master_multi.unregisterPublisher(node.name, p, node.uri)
                for t in node.subscribed:
                    rospy.loginfo("unregister subscriber '%s' [%s] from ROS master: %s", t, node.name, node.masteruri)
                    master_multi.unregisterSubscriber(node.name, t, node.uri)
                if self.master_state is not None:
                    for s in node.services:
                        rospy.loginfo("unregister service '%s' [%s] from ROS master: %s", s, node.name, node.masteruri)
                        service = self.master_info.getService(s)
                        if not (service is None):
                            master_multi.unregisterService(node.name, s, service.uri)
                r = master_multi()
                for code, msg, _ in r:
                    if code != 1:
                        rospy.logwarn("unregistration failed: %s", msg)
            except Exception, e:
                rospy.logwarn("Error while unregister node %s: %s", utf8(node.name), utf8(e))
                raise DetailedError("Unregister error",
                                    ''.join(['Error while Unregister node ', node.name]),
                                    utf8(e))
            finally:
                socket.setdefaulttimeout(None)
        return True

    def on_unregister_nodes(self):
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        # put into the queue and start the que handling
        for node in selectedNodes:
            if node.pid is None or len(selectedNodes) == 1:
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               ''.join(['unregister node ', node.name]),
                                               self.unregister_node,
                                               (node, (len(selectedNodes) == 1)))
        self._start_process_queue()

    def on_stop_context_toggled(self, state):
        menu = QMenu(self)
        self.killAct = QAction("&Kill Node", self, shortcut=QKeySequence.New, statusTip="Kill selected node", triggered=self.kill_nodes)
        self.unregAct = QAction("&Unregister Nodes...", self, shortcut=QKeySequence.Open, statusTip="Open an existing file", triggered=self.unreg_nodes)
        menu.addAction(self.killAct)
        menu.addAction(self.unregAct)
        menu.exec_(self.masterTab.stopContextButton.pos())

    def getHostFromNode(self, node):
        '''
        If the node is running the host the node URI will be returned. Otherwise
        tries to get the host from the launch configuration. If the configuration
        contains no machine assignment for this node the host of the ROS master URI
        will be used.
        @param node:
        @type node: U{master_discovery_fkie.NodeInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#master_discovery_fkie.master_info.NodeInfo>}
        '''
        if node.uri is not None:
            return get_hostname(node.uri)
        # try to get it from the configuration
        for c in node.cfgs:
            if not isinstance(c, tuple):
                launch_config = self.__configs[c]
                item = launch_config.getNode(node.name)
                if item is not None and item.machine_name and not item.machine_name == 'localhost':
                    return launch_config.Roscfg.machines[item.machine_name].address
        # return the host of the assigned ROS master
        return get_hostname(node.masteruri)

    def on_io_clicked(self):
        '''
        Shows IO of the selected nodes.
        '''
        key_mod = QApplication.keyboardModifiers()
        if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
            self.masterTab.ioButton.showMenu()
        else:
            cursor = self.cursor()
            self.setCursor(Qt.WaitCursor)
            try:
                selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
                if selectedNodes:
                    ret = True
                    if len(selectedNodes) > 5:
                        ret = MessageBox.question(self, "Show IO", "You are going to open the IO of " + utf8(len(selectedNodes)) + " nodes at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
                        ret = (ret == MessageBox.Ok)
                    if ret:
                        for node in selectedNodes:
                            self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                                ''.join(['show IO of ', node.name]),
                                                                nm.screen().openScreen,
                                                                (node.name, self.getHostFromNode(node), False, self.current_user))
                        self._start_process_queue_prio()
                else:
                    self.on_show_all_screens()
            finally:
                self.setCursor(cursor)

    def on_kill_screens(self):
        '''
        Kills selected screens, if some available.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               ''.join(['kill screen of ', node.name]),
                                               nm.screen().killScreens,
                                               (node.name, self.getHostFromNode(node), False, self.current_user))
            self._start_process_queue()
        finally:
            self.setCursor(cursor)

    def on_show_all_screens(self):
        '''
        Shows all available screens.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            host = get_hostname(self.masteruri)
            sel_screen = []
            try:
                screens = nm.screen().getActiveScreens(host, auto_pw_request=True, user=self.current_user)
                sel_screen, _ = SelectDialog.getValue('Open screen', '', screens, False, False, self)  # _:=ok
            except Exception, e:
                rospy.logwarn("Error while get screen list: %s", utf8(e))
                MessageBox.warning(self, "Screen list error",
                                   ''.join(['Error while get screen list from ', host]),
                                   utf8(e))
            for screen in sel_screen:
                try:
                    if not nm.screen().openScreenTerminal(host, screen, screen, self.current_user):
                        pass
                except Exception, e:
                    rospy.logwarn("Error while show IO for %s: %s", utf8(screen), utf8(e))
                    MessageBox.warning(self, "Show IO error",
                                       ''.join(['Error while show IO ', screen, ' on ', host]),
                                       utf8(e))
        finally:
            self.setCursor(cursor)

    def on_log_clicked(self):
        '''
        Shows log files of the selected nodes.
        '''
        try:
            only_screen = True
            key_mod = QApplication.keyboardModifiers()
            if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
                only_screen = False
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            ret = True
            if len(selectedNodes) > 5:
                ret = MessageBox.question(self, "Show Log", "You are going to open the logs of " + utf8(len(selectedNodes)) + " nodes at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
                ret = (ret == MessageBox.Ok)
            if ret:
                for node in selectedNodes:
                    self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                        ''.join(['show log of ', node.name]),
                                                        nm.starter().openLog,
                                                        (node.name, self.getHostFromNode(node), self.current_user, only_screen))
                self._start_process_queue_prio()
        except Exception, e:
            print traceback.format_exc(1)
            rospy.logwarn("Error while show log: %s", utf8(e))
            MessageBox.warning(self, "Show log error",
                               'Error while show Log',
                               utf8(e))

    def on_log_path_copy(self):
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        nodenames = []
        for n in selectedNodes:
            nodenames.append(n.name)
        try:
            host = get_hostname(self.masteruri)
            path_on_host = nm.starter().get_log_path(host, nodenames, True)
            QApplication.clipboard().setText(''.join([getpass.getuser() if self.is_local else self.current_user, '@', host, ':', path_on_host]))
        except Exception as e:
            MessageBox.warning(self, "Get log path",
                               'Error while get log path',
                               utf8(e))
#    self._progress_queue.add2queue(utf8(uuid.uuid4()),
#                                   'Get log path',
#                                   nm.starter().get_log_path,
#                                   (get_hostname(self.masteruri), nodenames))
#    self._start_process_queue()

#  def on_log_show_selected(self):
#    try:
#      nm.screen().LOG_PATH.
#      screens = nm.screen().getActiveScreens(host, auto_pw_request=True)
#      sel_screen, ok = SelectDialog.getValue('Open log', '', screens, False, self)
#    except Exception, e:
#      rospy.logwarn("Error while get screen list: %s", utf8(e))
#      MessageBox.warning(self, "Screen list error",
#                         ''.join(['Error while get screen list from ', host]),
#                         utf8(e))

    def on_log_delete_clicked(self):
        '''
        Deletes log files of the selected nodes.
        '''
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                           ''.join(['delete Log of ', node.name]),
                                           nm.starter().deleteLog,
                                           (node.name, self.getHostFromNode(node), False, self.current_user))
        self._start_process_queue_prio()

    def on_dynamic_config_clicked(self):
        '''
        Opens the dynamic configuration dialogs for selected nodes.
        '''
        if self.master_info is not None:
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            for n in selectedNodes:
                try:
                    nodes = sorted([srv_name[:-len('/set_parameters')] for srv_name, srv in self.master_info.services.items() if (srv_name.endswith('/set_parameters') and n.name in srv.serviceProvider)])
                    items = []
                    if len(nodes) == 1:
                        items = nodes
                    elif len(nodes) > 1:
                        items, _ = SelectDialog.getValue('Dynamic configuration selection', '', [i for i in nodes])
                        if items is None:
                            items = []
                    if len(items) > 3:
                        ret = MessageBox.question(self, 'Start dynamic reconfigure', 'It will starts %s dynamic reconfigure nodes?\n\n Are you sure?' % utf8(len(items)), buttons=MessageBox.Yes | MessageBox.No)
                        if ret != MessageBox.Yes:
                            return
                    for node in items:
                        env = dict(os.environ)
                        env["ROS_MASTER_URI"] = utf8(self.master_info.masteruri)
                        rospy.loginfo("Start dynamic reconfiguration for '%s'" % node)
                        _ = SupervisedPopen(['rosrun', 'node_manager_fkie', 'dynamic_reconfigure', node, '__ns:=dynamic_reconfigure'], env=env, object_id=node, description='Start dynamic reconfiguration for %s failed' % node)
                except Exception, e:
                    rospy.logwarn("Start dynamic reconfiguration for '%s' failed: %s" % (n.name, utf8(e)))
                    MessageBox.warning(self, "Start dynamic reconfiguration error",
                                       'Start dynamic reconfiguration for %s failed!' % n.name,
                                       utf8(e))

    def on_edit_config_clicked(self):
        '''
        '''
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            choices = self._getCfgChoises(node, True)
            choice, ok = self._getUserCfgChoice(choices, node.name)
            config = choices[choice] if choices and choice else ''
            if ok and isinstance(config, LaunchConfig):
                # get the file, which include the node and the main configuration file
                node_cfg = config.getNode(node.name)
                files = [config.Filename]
                if node_cfg.filename not in files:
                    files.append(node_cfg.filename)
                self.request_xml_editor.emit(files, ''.join(['name="', os.path.basename(node.name), '"']))

    def on_edit_rosparam_clicked(self):
        '''
        '''
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            # set the parameter in the ROS parameter server
            try:
                inputDia = MasterParameterDialog(node.masteruri if node.masteruri is not None else self.masteruri, ''.join([node.name, roslib.names.SEP]), parent=self)
                inputDia.setWindowTitle(' - '.join([os.path.basename(node.name), "parameter"]))
                if node.has_launch_cfgs(node.cfgs):
                    inputDia.add_warning("The changes may not have any effect, because the launch file was also loaded as not 'default' and the parameter in the launch file will be reloaded on start of the ROS node.")
                inputDia.show()
            except:
                rospy.logwarn("Error on retrieve parameter for %s: %s", utf8(node.name), traceback.format_exc(1))

    def on_save_clicked(self):
        # save the profile
        self.save_profile_signal.emit('')

    def on_close_clicked(self):
        '''
        Opens a dialog to select configurations to close or stop all nodes
        (with roscore) or shutdown the host.
        '''
        choices = dict()

        for path, _ in self.__configs.items():
            if isinstance(path, tuple):
                if path[2] == self.masteruri:
                    choices['DEFAULT CFG: %s' % path[0]] = path
            else:
                package = utf8(package_name(os.path.dirname(path))[0])
                choices['%s [%s]' % (os.path.basename(path), package)] = path
        cfg_items = choices.keys()
        cfg_items.sort()
        res = SelectDialog.getValue('Close/Stop/Shutdown', '',
                                    cfg_items, False, False,
                                    self, checkitem1='stop ROS',
                                    checkitem2='shutdown host')
        cfgs, _, stop_nodes, shutdown = res[0], res[1], res[2], res[3]
        # close configurations
        for config in cfgs:
            self._close_cfg(choices[config])
        if stop_nodes:
            self._on_stop_kill_roscore = True
            # stop all nodes, system nodes at the end
            ignore_nodes = [rospy.get_name(), '/master_discovery', '/rosout']
            self.stop_nodes_by_name(self.getRunningNodesIfLocal(), True, ignore_nodes)
            if shutdown:
                self.poweroff()
            else:
                self.stop_nodes_by_name(['/master_discovery'], True)
            self.stop_nodes_by_name(['/node_manager'], True)
        elif shutdown:
            self.poweroff()
        self.updateButtons()
        self.update_robot_icon()

    def poweroff(self):
        try:
            if nm.is_local(self.mastername):
                ret = MessageBox.warning(self, "ROS Node Manager",
                                         "Do you really want to shutdown localhost?",
                                         buttons=MessageBox.Ok | MessageBox.Cancel)
                if ret == MessageBox.Cancel:
                    return
            self._on_stop_poweroff = True
            # on shutdown stop only the /master_dsicovery node to remove it from lists
            # in other remote nodes
            self.stop_nodes_by_name(['/master_discovery'], True)
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'poweroff `%s`' % self.mastername,
                                           nm.starter().poweroff,
                                           ('%s' % self.mastername,))
            self._start_process_queue()
        except (Exception, nm.StartException), emsg:
            rospy.logwarn("Error while poweroff %s: %s", self.mastername, utf8(emsg))
            MessageBox.warning(self, "Run error",
                               'Error while poweroff %s' % self.mastername,
                               '%s' % utf8(emsg))

    def _close_cfg(self, cfg):
        try:
            self.removeConfigFromModel(cfg)
            if isinstance(cfg, tuple):
                if self.master_info is not None:
                    # close default configuration: stop the default_cfg node
                    node = self.master_info.getNode(cfg[0])
                    if node is not None:
                        self.stop_nodes([node])
            else:
                # remove from name resolution
                try:
                    for _, machine in self.__configs[cfg].Roscfg.machines.items():  # _:=name
                        if machine.name:
                            nm.nameres().remove_info(machine.name, machine.address)
                except:
                    pass
            del self.__configs[cfg]
        except:
            print traceback.format_exc(1)
            pass

    def on_topic_echo_clicked(self):
        '''
        Shows the output of the topic in a terminal.
        '''
        self._show_topic_output(False)

    def on_topic_hz_clicked(self):
        '''
        Shows the hz of the topic in a terminal.
        '''
        self._show_topic_output(True)

    def on_topic_hz_ssh_clicked(self):
        '''
        Shows the hz of the topic using ssh.
        '''
        self._show_topic_output(True, use_ssh=True)

    def on_topic_pub_clicked(self):
        selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
        if len(selectedTopics) > 0:
            for topic in selectedTopics:
                if not self._start_publisher(topic.name, topic.type):
                    break
        else:  # create a new topic
            # fill the input fields
            # determine the list all available message types
            root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
            packages = {}
            msg_types = []
            for p in root_paths:
                ret = get_packages(p)
                packages = dict(ret.items() + packages.items())
            for (p, direc) in packages.items():
                import rosmsg
                for f in rosmsg._list_types('%s/msg' % direc, 'msg', rosmsg.MODE_MSG):
                    msg_types.append("%s/%s" % (p, f))
            msg_types.sort()
            fields = {'Type': ('string', msg_types), 'Name': ('string', [''])}

            # create a dialog
            dia = ParameterDialog(fields, parent=self)
            dia.setWindowTitle('Publish to topic')
            dia.setFilterVisible(False)
            dia.resize(300, 95)
            dia.setFocusField('Name')
            if dia.exec_():
                params = dia.getKeywords()
                try:
                    if params['Name'] and params['Type']:
                        try:
                            self._start_publisher(params['Name'], params['Type'])
                        except Exception, e:
                            print traceback.format_exc(1)
                            rospy.logwarn("Publish topic '%s' failed: %s", utf8(params['Name']), utf8(e))
                            MessageBox.warning(self, "Publish topic error",
                                               ''.join(['Publish topic ', params['Name'], ' failed!']),
                                               utf8(e))
                    else:
                        MessageBox.warning(self, "Invalid name or type",
                                           "Can't publish to topic '%s' with type '%s'!" % (params['Name'], params['Type']))
                except (KeyError, ValueError), e:
                    MessageBox.warning(self, "Warning",
                                       'Error while add a parameter to the ROS parameter server',
                                       utf8(e))

    def start_publisher(self, topic_name, republish=False):
        '''
        Starts a publisher to given topic.
        '''
        if self.master_info is not None:
            topic = self.master_info.getTopic("%s" % topic_name)
            if topic is not None:
                self._start_publisher(topic.name, topic.type, republish)
            else:
                rospy.logwarn("Error while start publisher, topic not found: %s" % topic_name)

    def _start_publisher(self, topic_name, topic_type, republish=False):
        try:
            topic_name = roslib.names.ns_join(roslib.names.SEP, topic_name)
            mclass = roslib.message.get_message_class(topic_type)
            if mclass is None:
                MessageBox.warning(self, "Publish error",
                                   'Error while publish to %s' % topic_name,
                                   ''.join(['invalid message type: ', topic_type, '.\nIf this is a valid message type, perhaps you need to run "rosmake"']))
                return
            slots = mclass.__slots__
            types = mclass._slot_types
            default_topic_values = {}
            rate_values = ['once', 'latch', '1']
            if republish and topic_name in self.__republish_params:
                default_topic_values = self.__republish_params[topic_name][topic_type]
                rate_values = self.__republish_params[topic_name]['! Publish rate']
            args = ServiceDialog._params_from_slots(slots, types, default_topic_values)
            p = {'! Publish rate': ('string', rate_values), topic_type: ('dict', args)}
            dia = ParameterDialog(p)
            dia.setWindowTitle(''.join(['Publish to ', topic_name]))
            dia.showLoadSaveButtons()
            dia.resize(450, 300)
            dia.setFocusField('! Publish rate')

            if dia.exec_():
                params = dia.getKeywords()
                # store params for republish
                self.__republish_params[topic_name] = params
                rate = params['! Publish rate']
                opt_str = ''
                opt_name_suf = '__latch_'
                if rate == 'latch':
                    opt_str = ''
                elif rate == 'once' or rate == '-1':
                    opt_str = '--once'
                    opt_name_suf = '__once_'
                else:
                    try:
                        i = 0
                        try:
                            i = int(rate)
                        except:
                            i = float(rate)
                        if i > 0:
                            opt_str = ''.join(['-r ', rate])
                            opt_name_suf = '__%sHz_' % (utf8(rate).replace('.', '_'))
                    except:
                        pass
                # remove empty lists
                topic_params = dict()
                if topic_type in params:
                    topic_params = self._rem_empty_lists(params[topic_type])
                pub_cmd = ' '.join(['pub', topic_name, topic_type, '"', str(topic_params), '"', opt_str])
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               'start publisher for %s' % topic_name,
                                               nm.starter().runNodeWithoutConfig,
                                               (nm.nameres().address(self.masteruri), 'rostopic', 'rostopic', 'rostopic_pub%s%s%s' % (topic_name, opt_name_suf, str(rospy.Time.now())), [pub_cmd], self.masteruri, False, self.current_user))
                self._start_process_queue()
                return True
            else:
                return False
        except Exception, e:
            rospy.logwarn("Publish topic '%s' failed: %s", utf8(topic_name), utf8(e))
            MessageBox.warning(self, "Publish topic error",
                               ''.join(['Publish topic ', topic_name, ' failed!']),
                               utf8(e))
            print utf8(traceback.format_exc(1))
            return False

    def _rem_empty_lists(self, param_dict):
        result = dict()
        for key, value in param_dict.iteritems():
            if isinstance(value, dict):
                result[key] = self._rem_empty_lists(value)
            elif not (isinstance(value, list) and not value):
                result[key] = value
        return result

    def on_topic_pub_stop_clicked(self, topic_name=''):
        topic_names = []
        if topic_name:
            topic_names.append(topic_name)
        else:
            selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
            topic_names = ['%s' % topic.name for topic in selectedTopics]
        if self.master_info is not None:
            nodes2stop = []
            for topic in topic_names:
                topic_prefix = '/rostopic_pub%s_' % topic
                node_names = self.master_info.node_names
                for n in node_names:
                    if n.startswith(topic_prefix):
                        nodes2stop.append(n)
            self.stop_nodes_by_name(nodes2stop)

    def _show_topic_output(self, show_hz_only, use_ssh=False):
        '''
        Shows the output of the topic in a terminal.
        '''
        selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
        ret = True
        if len(selectedTopics) > 5:
            ret = MessageBox.question(self, "Show echo", "You are going to open the echo of " + utf8(len(selectedTopics)) + " topics at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
            ret = (ret == MessageBox.Ok)
        if ret:
            for topic in selectedTopics:
                self._add_topic_output2queue(topic, show_hz_only, use_ssh)

    def show_topic_output(self, topic_name, show_hz_only, use_ssh=False):
        '''
        Shows the topic output in a new window.
        '''
        if self.master_info is not None:
            topic = self.master_info.getTopic("%s" % topic_name)
            if topic is not None:
                self._add_topic_output2queue(topic, show_hz_only, use_ssh)
            else:
                rospy.logwarn("topic not found: %s" % topic_name)

    def _add_topic_output2queue(self, topic, show_hz_only, use_ssh=False):
        try:
                # connect to topic on remote host
            import shlex
            env = dict(os.environ)
            env["ROS_MASTER_URI"] = utf8(self.masteruri)
            namespace = rospy.names.namespace(topic.name)
            nodename = os.path.basename(topic.name)
            namespace = 'echo_%s%s%s%s' % ('hz_' if show_hz_only else '', 'ssh_' if use_ssh else '', utf8(get_hostname(self.masteruri)), namespace)
            cmd = 'rosrun node_manager_fkie node_manager --echo %s %s %s %s __name:=%s __ns:=%s' % (topic.name, topic.type, '--hz' if show_hz_only else '', '--ssh' if use_ssh else '', nodename, namespace)
            rospy.loginfo("Echo topic: %s" % cmd)
            ps = SupervisedPopen(shlex.split(cmd), env=env, stderr=None, close_fds=True, object_id=topic.name, description='Echo topic: %s' % topic.name)
            ps.finished.connect(self._topic_dialog_closed)
            self.__echo_topics_dialogs[topic.name] = ps
        except Exception, e:
            rospy.logwarn("Echo topic '%s' failed: %s" % (topic.name, utf8(e)))
            MessageBox.warning(self, "Echo of topic error",
                               'Echo of topic %s failed!' % topic.name,
                               '%s' % utf8(e))

    def _topic_dialog_closed(self, topic_name):
        if topic_name in self.__echo_topics_dialogs:
            del self.__echo_topics_dialogs[topic_name]

    def on_service_call_clicked(self):
        '''
        calls a service.
        '''
        selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
        for service in selectedServices:
            param = ServiceDialog(service, self)
            param.show()

    def service_call(self, service_name):
        service = self.master_info.getService(utf8(service_name))
        if service is not None:
            param = ServiceDialog(service, self)
            param.show()

    def on_node_filter_changed(self, text):
        '''
        Filter the displayed nodes
        '''
        self.node_proxy_model.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))

    def on_topic_filter_changed(self, text):
        '''
        Filter the displayed topics
        '''
        self.topic_proxyModel.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))

    def on_service_filter_changed(self, text):
        '''
        Filter the displayed services
        '''
        self.service_proxyModel.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))

    def on_parameter_filter_changed(self, text):
        '''
        Filter the displayed parameter
        '''
        self.parameter_proxyModel.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))

    def on_get_parameter_clicked(self):
        '''
        Requests parameter list from the ROS parameter server.
        '''
        self.parameterHandler.requestParameterList(self.masteruri)

    def on_add_parameter_clicked(self):
        '''
        Adds a parameter to the ROS parameter server.
        '''
        selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
        ns = '/'
        if selectedParameter:
            ns = roslib.names.namespace(selectedParameter[0][0])
        fields = {'name': ('string', ns), 'type': ('string', ['string', 'int', 'float', 'bool', 'list']), 'value': ('string', '')}
        newparamDia = ParameterDialog(fields, parent=self)
        newparamDia.setWindowTitle('Add new parameter')
        newparamDia.setFilterVisible(False)
        newparamDia.resize(400, 120)
        newparamDia.accepted.connect(self._on_add_parameter_accepted)
        newparamDia.setFocusField('name')
        newparamDia.show()
        newparamDia.raise_()
        newparamDia.activateWindow()

    def _on_add_parameter_accepted(self):
        if isinstance(self.sender(), ParameterDialog):
            params = self.sender().getKeywords()
            try:
                if params['type'] == 'int':
                    value = int(params['value'])
                elif params['type'] == 'float':
                    value = float(params['value'])
                elif params['type'] == 'bool':
                    value = bool(params['value'].lower() in ("yes", "true", "t", "1"))
                elif params['type'] == 'list':
                    try:
                        import yaml
                        value = [yaml.load(params['value'])]
                        # if there is no YAML, load() will return an
                        # empty string.  We want an empty dictionary instead
                        # for our representation of empty.
                        if value is None:
                            value = []
                    except yaml.MarkedYAMLError, e:
                        MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e), buttons=MessageBox.Ok)
                        return
                else:
                    value = params['value']
                self.parameterHandler.deliverParameter(self.masteruri, {params['name']: value})
                self.parameterHandler.requestParameterList(self.masteruri)
                self.sender().close()
            except (KeyError, ValueError), e:
                MessageBox.warning(self, "Warning",
                                   'Error while add a parameter to the ROS parameter server',
                                   utf8(e))

    def on_delete_parameter_clicked(self):
        '''
        Deletes the parameter from the ROS parameter server.
        '''
        selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
        try:
            socket.setdefaulttimeout(15)
            name = rospy.get_name()
            master = xmlrpclib.ServerProxy(self.masteruri)
            master_multi = xmlrpclib.MultiCall(master)
            for (key, _) in selectedParameter:  # _ := value
                master_multi.deleteParam(name, key)
            r = master_multi()
            for code, msg, parameter in r:
                if code != 1:
                    rospy.logwarn("Error on delete parameter '%s': %s", parameter, msg)
        except:
            rospy.logwarn("Error on delete parameter: %s", utf8(traceback.format_exc(1)))
            MessageBox.warning(self, "Warning",
                               'Error while delete a parameter to the ROS parameter server',
                               utf8(traceback.format_exc(1)))
        else:
            self.on_get_parameter_clicked()
        finally:
            socket.setdefaulttimeout(None)

    def on_save_parameter_clicked(self):
        '''
        Stores selected parameter to a file.
        '''
        selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
        if selectedParameter:
            # (fileName, filter)
            (fileName, _) = QFileDialog.getSaveFileName(self,
                                                        "Save parameter",
                                                        nm.settings().current_dialog_path,
                                                        "YAML files (*.yaml);;All files (*)")
            if fileName:
                nm.settings().current_dialog_path = os.path.dirname(fileName)
                try:
                    with open(fileName, 'w+') as f:
                        values = dict()
                        # convert ROS namespaces of parameters to YAML namespaces
                        for (key, value) in selectedParameter:
                            keys = key.strip(rospy.names.SEP).split(rospy.names.SEP)
                            curr_v = values
                            for k in keys:
                                if k in curr_v:
                                    curr_v = curr_v[k]
                                elif k != keys[-1]:
                                    curr_v[k] = dict()
                                    curr_v = curr_v[k]
                                else:
                                    curr_v[k] = value
                        import yaml
#            print yaml.dump(values, default_flow_style=False)
                        f.write(yaml.dump(values, default_flow_style=False))
                except Exception as e:
                    print utf8(traceback.format_exc(1))
                    MessageBox.warning(self, "Save parameter Error",
                                       'Error while save parameter',
                                       utf8(e))

    def _replaceDoubleSlash(self, liste):
        '''
        used to avoid the adding of \\ for each \ in a string of a list
        '''
        if liste and isinstance(liste, list):
            result = []
            for l in liste:
                val = l
                if isinstance(l, (str, unicode)):
                    val = l.replace("\\n", "\n")
#          result.append("".join([val]))
                elif isinstance(l, list):
                    val = self._replaceDoubleSlash(l)
                result.append(val)
            return result
        return liste

    def _on_parameter_item_changed(self, item):
        '''
        add changes to the ROS parameter server
        '''
        if isinstance(item, ParameterValueItem):
            try:
                if isinstance(item.value, bool):
                    value = bool(item.text().lower() in ("yes", "true", "t", "1"))
                elif isinstance(item.value, int):
                    value = int(item.text())
                elif isinstance(item.value, float):
                    value = float(item.text())
                elif isinstance(item.value, list):
                    try:
                        import yaml
                        value = yaml.load(item.text())
                        # if there is no YAML, load() will return an
                        # empty string.  We want an empty dictionary instead
                        # for our representation of empty.
                        if value is None:
                            value = []
                        value = self._replaceDoubleSlash(value)
                    except yaml.MarkedYAMLError, e:
                        MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e), buttons=MessageBox.Ok)
                        item.setText(utf8(item.value))
                        return
                else:
                    value = item.text()
                self.parameterHandler.deliverParameter(self.masteruri, {item.name: value})
                item.value = value
            except ValueError, e:
                MessageBox.warning(self, "Warning",
                                   'Error while add changes to the ROS parameter server',
                                   utf8(e))
                item.setText(item.value)

    def _on_param_list(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The list the parameter names.
        @type params: C{[str]}
        '''
        if code == 1:
            params.sort()
            self.parameterHandler.requestParameterValues(masteruri, params)

    def _on_param_values(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The dictionary the parameter names and request result.
        @type params: C{dict(paramName : (code, statusMessage, parameterValue))}
        '''
        if code == 1:
            result = {}
            for p, (code_n, _, val) in params.items():  # _ := msg_n
                if code_n == 1:
                    result[p] = val
                else:
                    result[p] = ''
                if p == '/use_sim_time':
                    self.__use_sim_time = (code_n == 1 and val)
            self.parameter_model.updateModelData(result)
        else:
            rospy.logwarn("Error on retrieve parameter from %s: %s", utf8(masteruri), utf8(msg))

    def _on_delivered_values(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The dictionary the parameter names and request result.
        @type params: C{dict(paramName : (code, statusMessage, parameterValue))}
        '''
        errmsg = ''
        if code == 1:
            for p, (code_n, msg, _) in params.items():  # _ := value
                if code_n != 1:
                    errmsg = '%s: %s\n%s' % (p, errmsg, msg)
        else:
            errmsg = msg if msg else 'Unknown error on set parameter'
        if errmsg:
            MessageBox.warning(self, "Warning",
                               'Error while delivering parameter to the ROS parameter server',
                               utf8(errmsg))

    def _on_sim_param_values(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The dictionary the parameter names and request result.
        @type params: C{dict(paramName : (code, statusMessage, parameterValue))}
        '''
        robot_icon_found = False
        if code == 1:
            for p, (code_n, _, val) in params.items():  # _ := msg_n
                if p == '/use_sim_time':
                    self.__use_sim_time = (code_n == 1 and val)
                elif p == '/robot_icon':
                    robot_icon_found = True
                    self.__current_parameter_robot_icon = val if code_n == 1 else ''
                    self.update_robot_icon()
                elif p.startswith('/roslaunch/uris'):
                    if code_n == 1:
                        for _, value in val.items():
                            self.launch_server_handler.updateLaunchServerInfo(value)
                elif p == "/run_id":
                    if self.__run_id != val:
                        self.__run_id = val
                        # you have to launch global parameter
                        for _, launch_cfg in self.__configs.items():
                            try:
                                launch_cfg.global_param_done.remove(masteruri)
                            except ValueError:
                                pass
        else:
            rospy.logwarn("Error on retrieve sim parameter value from %s: %s", utf8(masteruri), utf8(msg))
        if not robot_icon_found:
            self.__current_parameter_robot_icon = ''
            self.update_robot_icon()

    def _get_nm_masteruri(self):
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it. The 'materuri' attribute will be set to the requested value.
        @return: ROS master URI
        @rtype: C{str} or C{None}
        '''
        if not hasattr(self, '_nm_materuri') or self._nm_materuri is None:
            masteruri = masteruri_from_ros()
            master = xmlrpclib.ServerProxy(masteruri)
            _, _, self._nm_materuri = master.getUri(rospy.get_name())  # reuslt: code, message, self._nm_materuri
        return self._nm_materuri

    def append_diagnostic(self, diagnostic_status):
        nodes = self.getNode(diagnostic_status.name)
        for node in nodes:
            node.append_diagnostic_status(diagnostic_status)
        if nodes:
            # get node by selected items
            if self._is_current_tab_name('Nodes'):
                return
            selections = self.masterTab.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                if node.name == diagnostic_status.name:
                    self.on_node_selection_changed(None, None)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Shortcuts handling                               %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def select_host_block(self, index):
        '''
        Selects all nodes of a host with given index
        @param index: the index of the host in the tree model
        @type index: C{int}
        '''
        root = self.masterTab.nodeTreeView.model().index(index, 0)
        if not root.isValid():
            return
        self.masterTab.nodeTreeView.expand(root)
#    firstChild = root.child(0, 0)
        last_row_index = len(self.node_tree_model.header) - 1
#    lastChild = root.child(0, last_row_index)
        i = 0
        selection = QItemSelection()
        while root.child(i, 0).isValid():
            index = root.child(i, 0)
            model_index = self.node_proxy_model.mapToSource(index)
            item = self.node_tree_model.itemFromIndex(model_index)
            if item is not None and not self._is_in_ignore_list(item.name):
                selection.append(QItemSelectionRange(index, root.child(i, last_row_index)))
            i = i + 1
#    selection = QItemSelection(firstChild, lastChild)
        self.masterTab.nodeTreeView.selectionModel().select(selection, QItemSelectionModel.ClearAndSelect)

    def _is_in_ignore_list(self, name):
        for i in self._stop_ignores:
            if name.endswith(i):
                return True
        return False

    def on_shortcut1_activated(self):
        self.select_host_block(0)

    def on_shortcut2_activated(self):
        self.select_host_block(1)

    def on_shortcut3_activated(self):
        self.select_host_block(2)

    def on_shortcut4_activated(self):
        self.select_host_block(3)

    def on_shortcut5_activated(self):
        self.select_host_block(4)

    def on_shortcut_collapse_all(self):
        self.masterTab.nodeTreeView.selectionModel().clearSelection()
        self.masterTab.nodeTreeView.collapseAll()

    def on_copy_c_pressed(self):
        result = ''
        if self.masterTab.nodeTreeView.hasFocus():
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                try:
                    result = '%s %s' % (result, node.name)
                except Exception:
                    pass
        elif self.masterTab.topicsView.hasFocus():
            selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
            for topic in selectedTopics:
                try:
                    result = '%s %s' % (result, topic.name)
                except Exception:
                    pass
        elif self.masterTab.servicesView.hasFocus():
            selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
            for service in selectedServices:
                try:
                    result = '%s %s' % (result, service.name)
                except Exception:
                    pass
        elif self.masterTab.parameterView.hasFocus():
            selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
            for (name, _value) in selectedParameter:
                try:
                    result = '%s %s' % (result, name)
                except Exception:
                    pass
        QApplication.clipboard().setText(result.strip())

    def on_copy_x_pressed(self):
        result = ''
        if self.masterTab.nodeTreeView.hasFocus():
            selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                try:
                    result = '%s %s' % (result, node.pid)
                except Exception:
                    pass
        elif self.masterTab.topicsView.hasFocus():
            selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
            for topic in selectedTopics:
                try:
                    result = '%s %s' % (result, topic.type)
                except Exception:
                    pass
        elif self.masterTab.servicesView.hasFocus():
            selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
            for service in selectedServices:
                try:
                    result = '%s %s' % (result, service.type)
                except Exception:
                    pass
        elif self.masterTab.parameterView.hasFocus():
            selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
            for (_, value) in selectedParameter:
                try:
                    result = '%s %s' % (result, value)
                except Exception:
                    pass
        QApplication.clipboard().setText(result.strip())

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Filter handling                               %%%%%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


class NodesSortFilterProxyModel(QSortFilterProxyModel):

    def filterAcceptsRow(self, sourceRow, sourceParent):
        '''
        Perform filtering on column 0 (Name)
        '''
        if not self.filterRegExp().pattern():
            return True
        if (self.filterAcceptsRowItself(sourceRow, sourceParent)):
            return True
#         # accept if any of the parents is accepted on it's own merits
#         parent = sourceParent
#         while (parent.isValid()):
#             if (self.filterAcceptsRowItself(parent.row(), parent.parent())):
#                 return True
#             parent = parent.parent()
        # accept if any of the children is accepted on it's own merits
        if (self.hasAcceptedChildren(sourceRow, sourceParent)):
            return True
        return False

    def hasAcceptedChildren(self, sourceRow, sourceParent):
        index = self.sourceModel().index(sourceRow, 0, sourceParent)
        if not index.isValid():
            return False
        # check if there are children
        childCount = index.model().rowCount(index)
        if childCount == 0:
            return False
        for i in range(childCount):
            if (self.filterAcceptsRowItself(i, index)):
                return True
            # recursive call -> NOTICE that this is depth-first searching, you're probably better off with breadth first search...
            if (self.hasAcceptedChildren(i, index)):
                return True
        return False

    def filterAcceptsRowItself(self, sourceRow, sourceParent):
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
        item = self.sourceModel().data(index0)
        if item is not None:
            # skip groups
            if '{' not in item:
                regex = self.filterRegExp()
                return (regex.indexIn(self.sourceModel().data(index0)) != -1)
        return False


class TopicsSortFilterProxyModel(QSortFilterProxyModel):

    def filterAcceptsRow(self, sourceRow, sourceParent):
        '''
        Perform filtering on columns 0 and 3 (Name, Type)
        '''
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
        index3 = self.sourceModel().index(sourceRow, 3, sourceParent)
        regex = self.filterRegExp()
        return (regex.indexIn(self.sourceModel().data(index0, TopicItem.NAME_ROLE)) != -1 or
                regex.indexIn(self.sourceModel().data(index3)) != -1)


class ServicesSortFilterProxyModel(QSortFilterProxyModel):

    def filterAcceptsRow(self, sourceRow, sourceParent):
        '''
        Perform filtering on columns 0 and 1 (Name, Type)
        '''
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
#    index1 = self.sourceModel().index(sourceRow, 1, sourceParent)
        regex = self.filterRegExp()
        return (regex.indexIn(self.sourceModel().data(index0, ServiceItem.NAME_ROLE)) != -1 or
                regex.indexIn(self.sourceModel().data(index0, ServiceItem.TYPE_ROLE)) != -1)


class ParameterSortFilterProxyModel(QSortFilterProxyModel):

    def filterAcceptsRow(self, sourceRow, sourceParent):
        '''
        Perform filtering on columns 0 and 1 (Name, Value)
        '''
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
#    index1 = self.sourceModel().index(sourceRow, 1, sourceParent)
        index2 = self.sourceModel().index(sourceRow, 2, sourceParent)
        regex = self.filterRegExp()
        return (regex.indexIn(self.sourceModel().data(index0, ParameterNameItem.NAME_ROLE)) != -1 or
                regex.indexIn(self.sourceModel().data(index2, ParameterValueItem.VALUE_ROLE)) != -1)

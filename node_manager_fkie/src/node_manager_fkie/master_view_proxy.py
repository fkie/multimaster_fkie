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

from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import loadUi

import re
import os
import sys
import socket
import xmlrpclib
import threading
import time
from urlparse import urlparse

import rospy
import roslib

import node_manager_fkie as nm
from html_delegate import HTMLDelegate
from topic_list_model import TopicModel, TopicItem
from node_tree_model import NodeTreeModel, NodeItem, GroupItem, HostItem
from service_list_model import ServiceModel, ServiceItem
from parameter_list_model import ParameterModel, ParameterValueItem
from default_cfg_handler import DefaultConfigHandler
from launch_config import LaunchConfig, LaunchConfigException
from master_discovery_fkie.master_info import NodeInfo 
from parameter_dialog import ParameterDialog, MasterParameterDialog, ServiceDialog
from select_dialog import SelectDialog
from echo_dialog import EchoDialog
from parameter_handler import ParameterHandler
from detailed_msg_box import WarningMessageBox, DetailedError
from progress_queue import ProgressQueue, ProgressThread
from common import masteruri_from_ros, get_packages, package_name




class MasterViewProxy(QtGui.QWidget):
  '''
  This class stores the informations about a ROS master and shows it on request.
  '''

  updateHostRequest = QtCore.Signal(str)
  host_description_updated = QtCore.Signal(str, str, str)
  '''@ivar: the signal is emitted on description changes and contains the 
  ROS Master URI, host address and description a parameter.'''
  
  capabilities_update_signal = QtCore.Signal(str, str, str, list)
  '''@ivar: the signal is emitted if a description with capabilities is received 
  and has the ROS master URI, host address, the name of the default_cfg node and a list with 
  descriptions (L{default_cfg_fkie.Description}) as parameter.'''
  remove_config_signal  = QtCore.Signal(str)
  '''@ivar: the signal is emitted if a default_cfg was removed'''

  description_signal  = QtCore.Signal(str, str)
  '''@ivar: the signal is emitted to show a description (title, description)'''
  
  request_xml_editor = QtCore.Signal(list, str)
  '''@ivar: the signal to open a xml editor dialog (list with files, search text)'''

  def __init__(self, masteruri, parent=None):
    '''
    Creates a new master.
    @param masteruri: the URI of the ROS master
    @type masteruri: C{str}
    '''
    QtGui.QWidget.__init__(self, parent)
    self.setObjectName(' - '.join(['MasterViewProxy', masteruri]))
    self.masteruri = masteruri
    self.mastername = masteruri
    try:
      o = urlparse(self.masteruri)
      self.mastername = o.hostname
    except:
      pass
    try:
      self.__current_path = os.environ('HOME')
    except:
      self.__current_path = os.getcwd()

    self._tmpObjects = []
    self.__master_state = None
    self.__master_info = None
    self.__configs = dict() # [file name] = LaunchConfig
#    self.rosconfigs = dict() # [launch file path] = LaunchConfig()
    self.__in_question = [] # stores the changed files, until the user is interacted
#    self.__uses_confgs = dict() # stores the decisions of the user for used configuration to start of node
    '''@ivar: stored the question dialogs for changed files '''
    self._stop_ignores = ['rosout', rospy.get_name(), 'node_manager', 'master_discovery', 'master_sync', 'default_cfg', 'zeroconf']
    ''' @todo: detect the names of master_discovery and master_sync ndoes'''
    
    self.__echo_topics_dialogs = dict() # [topic name] = EchoDialog
    '''@ivar: stores the open EchoDialogs '''
    self.__last_info_type = None # {Node, Topic, Service}
    self.__last_info_text = None
    self.__use_sim_time = False

    self.default_cfg_handler = DefaultConfigHandler()
    self.default_cfg_handler.node_list_signal.connect(self.on_default_cfg_nodes_retrieved)
    self.default_cfg_handler.description_signal.connect(self.on_default_cfg_descr_retrieved)
    self.default_cfg_handler.err_signal.connect(self.on_default_cfg_err)
    
    self.masterTab = QtGui.QWidget()
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MasterTab.ui')
    loadUi(ui_file, self.masterTab)
    tabLayout = QtGui.QVBoxLayout(self)
    tabLayout.setContentsMargins(0, 0, 0, 0)
    tabLayout.addWidget(self.masterTab)
    self._progress_queue = ProgressQueue(self.masterTab.progressFrame, self.masterTab.progressBar, self.masterTab.progressCancelButton)
    
    # setup the node view
    self.node_tree_model = NodeTreeModel(nm.nameres().address(self.masteruri), self.masteruri)
    self.node_tree_model.hostInserted.connect(self.on_host_inserted)
    self.masterTab.nodeTreeView.setModel(self.node_tree_model)
    for i, (name, width) in enumerate(NodeTreeModel.header):
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
    self.topic_proxyModel =  TopicsSortFilterProxyModel(self)
    self.topic_proxyModel.setSourceModel(self.topic_model)
    self.masterTab.topicsView.setModel(self.topic_proxyModel)
#    self.masterTab.topicsView.setModel(self.topic_model)
    for i, (name, width) in enumerate(TopicModel.header):
      self.masterTab.topicsView.setColumnWidth(i, width)
    self.topicNameDelegate = HTMLDelegate()
    self.masterTab.topicsView.setItemDelegateForColumn(0, self.topicNameDelegate)
    sm = self.masterTab.topicsView.selectionModel()
    sm.selectionChanged.connect(self.on_topic_selection_changed)
    self.masterTab.topicsView.activated.connect(self.on_topic_activated)
    self.masterTab.topicsView.clicked.connect(self.on_topic_clicked)
    self.masterTab.topicsView.setSortingEnabled(True)
#    self.topic_proxyModel.filterAcceptsRow = _filterTopicsAcceptsRow

    # setup the service view
    self.service_model = ServiceModel()
    self.service_proxyModel =  ServicesSortFilterProxyModel(self)
    self.service_proxyModel.setSourceModel(self.service_model)
    self.masterTab.servicesView.setModel(self.service_proxyModel)
#    self.masterTab.servicesView.setModel(self.service_model)
    for i, (name, width) in enumerate(ServiceModel.header):
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
#    self.service_proxyModel.filterAcceptsRow = _filterServiceAcceptsRow
    
    # setup the parameter view
    self.parameter_model = ParameterModel()
    self.parameter_model.itemChanged.connect(self._on_parameter_item_changed)
    self.parameter_proxyModel =  ParameterSortFilterProxyModel(self)
    self.parameter_proxyModel.setSourceModel(self.parameter_model)
    self.masterTab.parameterView.setModel(self.parameter_proxyModel)
    for i, (name, width) in enumerate(ParameterModel.header):
      self.masterTab.parameterView.setColumnWidth(i, width)
    self.parameterNameDelegate = HTMLDelegate()
    self.masterTab.parameterView.setItemDelegateForColumn(0, self.parameterNameDelegate)
#    self.parameterEditor = QtGui.QStyledItemDelegate()
#    factory = QtGui.QItemEditorFactory.defaultFactory()
#    self.parameterEditor.setItemEditorFactory(factory)
#    self.masterTab.parameterView.setItemDelegateForColumn(1, self.parameterEditor)
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
    self.masterTab.pubTopicButton.clicked.connect(self.on_topic_pub_clicked)
    self.masterTab.pubStopTopicButton.clicked.connect(self.on_topic_pub_stop_clicked)

    self.masterTab.callServiceButton.clicked.connect(self.on_service_call_clicked)
    self.masterTab.topicFilterInput.textChanged.connect(self.on_topic_filter_changed)
    self.masterTab.serviceFilterInput.textChanged.connect(self.on_service_filter_changed)
    self.masterTab.parameterFilterInput.textChanged.connect(self.on_parameter_filter_changed)
    self.masterTab.getParameterButton.clicked.connect(self.on_get_parameter_clicked)
    self.masterTab.addParameterButton.clicked.connect(self.on_add_parameter_clicked)
    self.masterTab.deleteParameterButton.clicked.connect(self.on_delete_parameter_clicked)

    #create a handler to request the parameter
    self.parameterHandler = ParameterHandler()
    self.parameterHandler.parameter_list_signal.connect(self._on_param_list)
    self.parameterHandler.parameter_values_signal.connect(self._on_param_values)
    self.parameterHandler.delivery_result_signal.connect(self._on_delivered_values)
    #create a handler to request sim paramter
    self.parameterHandler_sim = ParameterHandler()
#    self.parameterHandler_sim.parameter_list_signal.connect(self._on_param_list)
    self.parameterHandler_sim.parameter_values_signal.connect(self._on_sim_param_values)
#    self.parameterHandler_sim.delivery_result_signal.connect(self._on_delivered_values)


    # creates a start menu
    start_menu = QtGui.QMenu(self)
    self.forceStartNodesAct = QtGui.QAction("&Force start node", self, statusTip="Force the start of selected node", triggered=self.on_force_start_nodes)
    start_menu.addAction(self.forceStartNodesAct)
    self.startNodesAtHostAct = QtGui.QAction("&Start node on host(Loaded config needed)", self, statusTip="Start node on other host", triggered=self.on_start_nodes_at_host)
    start_menu.addAction(self.startNodesAtHostAct)
    self.masterTab.startButton.setMenu(start_menu)

    # creates a stop menu
    stop_menu = QtGui.QMenu(self)
    self.killNodesAct = QtGui.QAction("&Kill Node", self, shortcut=QtGui.QKeySequence(QtCore.Qt.CTRL + QtCore.Qt.Key_Backspace), statusTip="Kill selected node", triggered=self.on_kill_nodes)
    self.unregNodesAct = QtGui.QAction("&Unregister Nodes", self, shortcut=QtGui.QKeySequence(QtCore.Qt.CTRL + QtCore.Qt.Key_Delete), statusTip="Removes the registration of selected nodes from ROS master", triggered=self.on_unregister_nodes)
    stop_menu.addAction(self.killNodesAct)
    stop_menu.addAction(self.unregNodesAct)
    self.masterTab.stopButton.setMenu(stop_menu)

    # creates a screen menu
    screen_menu = QtGui.QMenu(self)
    self.killScreensAct = QtGui.QAction("&Kill Screen", self, shortcut=QtGui.QKeySequence(QtCore.Qt.SHIFT + QtCore.Qt.Key_Backspace), statusTip="Kill available screens", triggered=self.on_kill_screens)
    screen_menu.addAction(self.killScreensAct)
    self.showAllScreensAct = QtGui.QAction("&Show all available screens", self, shortcut=QtGui.QKeySequence(QtCore.Qt.SHIFT + QtCore.Qt.Key_S), statusTip="Shows all available screens", triggered=self.on_show_all_screens)
    screen_menu.addAction(self.showAllScreensAct)
    self.masterTab.ioButton.setMenu(screen_menu)
    self.masterTab.ioButton.setEnabled(True)
    
    # create log menu
    log_menu = QtGui.QMenu(self)
    self.logCopyPathAct = QtGui.QAction("&Copy log path to clipboard", self, statusTip="Copy log path to clipboard", triggered=self.on_log_path_copy)
#    self.logShowSelectedAct = QtGui.QAction("&Select a log to show", self, statusTip="Select a log file to show (only local)", triggered=self.on_log_show_selected)
    log_menu.addAction(self.logCopyPathAct)
#    self.logCopyPathAct.setEnabled(nm.is_local(nm.nameres().getHostname(self.masteruri)))
#    log_menu.addAction(self.logShowSelectedAct)
    self.masterTab.logButton.setMenu(log_menu)

    # set the shortcuts
    self._shortcut1 = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+1", "Select first group")), self)
    self._shortcut1.activated.connect(self.on_shortcut1_activated)
    self._shortcut2 = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+2", "Select second group")), self)
    self._shortcut2.activated.connect(self.on_shortcut2_activated)
    self._shortcut3 = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+3", "Select third group")), self)
    self._shortcut3.activated.connect(self.on_shortcut3_activated)
    self._shortcut4 = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+4", "Select fourth group")), self)
    self._shortcut4.activated.connect(self.on_shortcut4_activated)
    self._shortcut5 = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+5", "Select fifth group")), self)
    self._shortcut5.activated.connect(self.on_shortcut5_activated)

    self._shortcut_collapse_all = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+C", "Collapse all groups")), self)
    self._shortcut_collapse_all.activated.connect(self.on_shortcut_collapse_all)
    self._shortcut_expand_all = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+E", "Expand all groups")), self)
    self._shortcut_expand_all.activated.connect(self.masterTab.nodeTreeView.expandAll)
    self._shortcut_run = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+R", "run selected nodes")), self)
    self._shortcut_run.activated.connect(self.on_start_clicked)
    self._shortcut_stop = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Alt+S", "stop selected nodes")), self)
    self._shortcut_stop.activated.connect(self.on_stop_clicked)

    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+C", "copy selected nodes to clipboard")), self.masterTab.nodeTreeView)
    self._shortcut_copy.activated.connect(self.on_copy_node_clicked)
    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+C", "copy selected topics to clipboard")), self.masterTab.topicsView)
    self._shortcut_copy.activated.connect(self.on_copy_topic_clicked)
    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+C", "copy selected services to clipboard")), self.masterTab.servicesView)
    self._shortcut_copy.activated.connect(self.on_copy_service_clicked)
    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+C", "copy selected parameter to clipboard")), self.masterTab.parameterView)
    self._shortcut_copy.activated.connect(self.on_copy_parameter_clicked)

#    print "================ create", self.objectName()
#
#  def __del__(self):
#    for ps in self.__echo_topics_dialogs.values():
#      try:
#        ps.terminate()
#      except:
#        pass
#    print "*********** destroy", self.objectName()

  def stop(self):
    print "  Shutdown master", self.masteruri, "..."
    self.default_cfg_handler.stop()
    self._progress_queue.stop()
    for ps in self.__echo_topics_dialogs.values():
      try:
        ps.terminate()
      except:
        pass
    print "  Master", self.masteruri, " is down!"
    
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
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
    '''
    try:
      update_nodes = False
      update_others = False
      if (master_info.masteruri == self.masteruri):
        self.mastername = master_info.mastername
        # store process pid's of remote nodes
        nodepids = []
        if not self.__master_info is None:
          nodepids = [(n.name, n.pid, n.uri, n.masteruri) for nodename, n in self.__master_info.nodes.items() if not n.isLocal]
        self.__master_info = master_info
        # restore process pid's of remote nodes
        if not self.__master_info is None:
          for nodename, pid, uri, muri in nodepids:
            node = self.__master_info.getNode(nodename)
            if not node is None:
              node.pid = pid
              node.masteruri = muri
        # request master info updates for new remote nodes
        nodepids2 = [(n.name, n.pid, n.uri, n.masteruri) for nodename, n in self.__master_info.nodes.items() if not n.isLocal]
        node2update = list(set(nodepids2)-set(nodepids))
        hosts2update = list(set([nm.nameres().getHostname(uri) for nodename, pid, uri, muri in node2update]))
        for host in hosts2update:
          self.updateHostRequest.emit(host)
        update_nodes = True
        update_others = True
      else:
        if not (self.__master_info is None or master_info is None):
          # update the process id of the remote nodes
          for nodename, node in master_info.nodes.items():
            n = self.__master_info.getNode(nodename)
            if not n is None and n.pid != node.pid and not n.isLocal and node.isLocal:
              update_nodes = True
              n.pid = node.pid
          # update the service information of the remote services
          for servicename, service in master_info.services.items():
            s = self.__master_info.getService(servicename)
            if not s is None and (s.uri != service.uri or s.type != service.type ) and s.masteruri == service.masteruri:
              update_others = True
              s.uri = service.uri
              s.type = service.type
          # remove remote node information, if node is not more running
          for nodename, node in self.__master_info.nodes.items():
            if not node.isLocal and node.masteruri == master_info.masteruri and master_info.getNode(nodename) is None:
              node.masteruri = self.masteruri

#      cputimes = os.times()
#      cputime_init = cputimes[0] + cputimes[1]
      if update_nodes:
#        print "    updateRunningNodesInModel", time.time()
        self.updateRunningNodesInModel(self.__master_info)
#        print "    updateRunningNodesInModel total", time.time()
      if update_others:
#        print "    updateTopicsListModel", time.time(), "topic count:", len(self.__master_info.topics)
        self.updateTopicsListModel(self.__master_info)
#        print "    updateServiceListModel", time.time(), "services count:", len(self.__master_info.services)
        self.updateServiceListModel(self.__master_info)
#        print "    updateDefaultConfigs", time.time()
        self.updateDefaultConfigs(self.__master_info)
#        print "    updateDefaultConfigs", time.time()
#      cputimes = os.times()
#      cputime = cputimes[0] + cputimes[1] - cputime_init
#      print "  update on ", self.__master_info.mastername if not self.__master_info is None else self.__master_state.name, cputime
      self.on_node_selection_changed(None, None)
      self.on_topic_selection_changed(None, None)
      self.on_service_selection_changed(None, None)
      self.parameterHandler_sim.requestParameterValues(self.masteruri, ["/use_sim_time"])
    except:
      import traceback
      print traceback.format_exc()

  @property
  def use_sim_time(self):
    return self.__use_sim_time

  def markNodesAsDuplicateOf(self, running_nodes):
    '''
    Marks all nodes, which are not running and in a given list as a duplicates nodes. 
    @param running_nodes: The list with names of running nodes
    @type running_nodes: C{[str]}
    '''
    self.node_tree_model.markNodesAsDuplicateOf(running_nodes)

  def getRunningNodesIfSync(self):
    '''
    Returns the list with all running nodes, which are registered by this ROS 
    master. Also the nodes, which are physically running on remote hosts.
    @return running_nodes: The list with names of running nodes
    @rtype running_nodes: C{[str]}
    '''
    if not self.master_info is None and self.master_info.getNodeEndsWith('master_sync'):
      return self.master_info.node_names
    return []

  def getRunningNodesIfLocal(self):
    '''
    Returns the list with all running nodes, which are running (has process) on this host.
    The nodes registered on this ROS master, but running on remote hosts are not 
    returned.
    @return running_nodes: The list with names of running nodes
    @rtype running_nodes: C{[str]}
    '''
    result = []
    if not self.master_info is None:
      for name, node in self.master_info.nodes.items():
        if node.isLocal:
          result.append(node.name)
    return result


  def updateRunningNodesInModel(self, master_info):
    '''
    Creates the dictionary with ExtendedNodeInfo objects and updates the nodes view.
    @param master_info: the mater information object
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
    '''
    if not master_info is None:
      self.node_tree_model.updateModelData(master_info.nodes)
      self.updateButtons()

  def getNode(self, node_name):
    '''
    @param node_name: The name of the node.
    @type node_name: str
    @return: The list the nodes with given name.
    @rtype: []
    '''
    return self.node_tree_model.getNode(node_name)

  def updateButtons(self):
    '''
    Updates the enable state of the buttons depending of the selection and 
    running state of the selected node.
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    has_running = False
    has_stopped = False
    has_invalid = False
    for node in selectedNodes:
      if not node.uri is None:
        has_running = True
        if node.pid is None:
          has_invalid = True
      else:
        has_stopped = True 
#    self.masterTab.startButton.setEnabled(has_stopped or has_invalid)
#    self.masterTab.stopButton.setEnabled(has_running or has_invalid)
    self.masterTab.startButton.setEnabled(True)
    self.masterTab.stopButton.setEnabled(True)
#    self.masterTab.ioButton.setEnabled(has_running or has_stopped)
    self.masterTab.logButton.setEnabled(True)
#    self.masterTab.logButton.setEnabled(has_running or has_stopped)
    self.masterTab.logDeleteButton.setEnabled(has_running or has_stopped)
    # test for available dynamic reconfigure services
    if not self.master_info is None:
      dyncfgServices = [s[:-len('/set_parameters')] for s in self.master_info.services.keys() if (s.endswith('/set_parameters'))]
      dyncfgNodes = [s for n in selectedNodes for s in dyncfgServices if s.startswith((n.name))]
      self.masterTab.dynamicConfigButton.setEnabled(len(dyncfgNodes))
    # the configuration is only available, if only one node is selected
    cfg_enable = False
    if len(selectedNodes) >= 1:
      cfg_enable = len(self._getCfgChoises(selectedNodes[0], True)) > 0
    self.masterTab.editConfigButton.setEnabled(cfg_enable and len(selectedNodes) == 1)
    self.startNodesAtHostAct.setEnabled(cfg_enable)
    self.masterTab.editRosParamButton.setEnabled(len(selectedNodes) == 1)
    self.masterTab.saveButton.setEnabled(len(self.launchfiles) > 1)
    # enable the close button only for local configurations
    self.masterTab.closeCfgButton.setEnabled(len([path for path, cfg in self.__configs.items() if (isinstance(path, tuple) and path[2] == self.masteruri) or not isinstance(path, tuple)]) > 0)

  def updateTopicsListModel(self, master_info):
    '''
    Updates the topic view based on the current master information.
    @param master_info: the mater information object
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
    '''
    if not master_info is None:
      self.topic_model.updateModelData(master_info.topics)

  def updateServiceListModel(self, master_info):
    '''
    Updates the service view based on the current master information.
    @param master_info: the mater information object
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
    '''
    if not master_info is None:
      self.service_model.updateModelData(master_info.services)

  def hasLaunchfile(self, path):
    '''
    @param path: the launch file
    @type path: C{str}
    @return: C{True} if the given launch file is open
    @rtype: C{boolean}
    '''
    return self.launchfiles.has_key(path)

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
#    QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
    stored_roscfg = None
    stored_argv = None
    if self.__configs.has_key(launchfile):
      # close the configuration
      stored_argv = self.__configs[launchfile].argv
      self.removeConfigFromModel(launchfile)
      stored_roscfg = self.__configs[launchfile].Roscfg
      del self.__configs[launchfile]
#      # remove machine entries
#      try:
#        for name, machine in self.__configs[launchfile].Roscfg.machines.items():
#          if machine.name:
#            nm.nameres().remove(name=machine.name, host=machine.address)
#      except:
#        import traceback
#        print traceback.format_exc()
    #load launch config
    try:
      # test for requerid args
      launchConfig = LaunchConfig(launchfile, masteruri=self.masteruri)
      loaded = False
      if stored_argv is None:
        req_args = launchConfig.getArgs()
        if req_args:
          params = dict()
          arg_dict = launchConfig.argvToDict(req_args)
          for arg in arg_dict.keys():
            params[arg] = ('string', [arg_dict[arg]])
          inputDia = ParameterDialog(params)
          inputDia.setFilterVisible(False)
          inputDia.setWindowTitle(''.join(['Enter the argv for ', launchfile]))
          if inputDia.exec_():
            params = inputDia.getKeywords()
            argv = []
            for p,v in params.items():
              if v:
                argv.append(''.join([p, ':=', v]))
            loaded = launchConfig.load(argv)
          else:
            return
      if not loaded or not stored_argv is None:
        launchConfig.load(req_args if stored_argv is None else stored_argv)
      for name, machine in launchConfig.Roscfg.machines.items():
        if machine.name:
          nm.nameres().addInfo(machine.name, machine.address, machine.address)

      # add launch file object to the list
      self.__configs[launchfile] = launchConfig
      self.appendConfigToModel(launchfile, launchConfig.Roscfg)
      self.masterTab.tabWidget.setCurrentIndex(0)
      
      #get the descriptions of capabilities and hosts
      try:
        robot_descr = launchConfig.getRobotDescr()
        capabilities = launchConfig.getCapabilitiesDesrc()
        for (host, caps) in capabilities.items():
          if not host:
            host = nm.nameres().mastername(self.masteruri)
          host_addr = nm.nameres().address(host)
          self.node_tree_model.addCapabilities(nm.nameres().masteruri(host), host_addr, launchfile, caps)
        for (host, descr) in robot_descr.items():
          if not host:
            host = nm.nameres().mastername(self.masteruri)
          host_addr = nm.nameres().address(host)
          tooltip = self.node_tree_model.updateHostDescription(nm.nameres().masteruri(host), host_addr, descr['type'], descr['name'], descr['description'])
          self.host_description_updated.emit(self.masteruri, host_addr, tooltip)
      except:
        import traceback
        print traceback.print_exc()

      # by this call the name of the host will be updated if a new one is defined in the launch file
      self.updateRunningNodesInModel(self.__master_info)

      # detect files changes
      if stored_roscfg and self.__configs[launchfile].Roscfg:
        stored_values = [(name, str(p.value)) for name, p in stored_roscfg.params.items()]
        new_values = [(name, str(p.value)) for name, p in self.__configs[launchfile].Roscfg.params.items()]
        # detect changes parameter
        paramset = set(name for name, value in (set(new_values) - set(stored_values)))
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
        # filter out anonymous nodes
        nodes2start = [n for n in nodes2start if not re.search(r"\d{3,6}_\d{10,}", n)]
        # restart nodes
        print nodes2start
        if nodes2start:
          restart = SelectDialog.getValue('The parameter/nodes are changed. Restart follow nodes?', nodes2start, False, True, self)
          self.start_nodes_by_name(restart, launchfile, True)

#      print "MASTER:", launchConfig.Roscfg.master
#      print "NODES_CORE:", launchConfig.Roscfg.nodes_core
#      for n in launchConfig.Roscfg.nodes:
#        n.__slots__ = [] 
#      print "NODES:", pickle.dumps(launchConfig.Roscfg.nodes)
#        
#      print "ROSLAUNCH_FILES:", launchConfig.Roscfg.roslaunch_files
#        # list of resolved node names. This is so that we can check for naming collisions
#      print "RESOLVED_NAMES:", launchConfig.Roscfg.resolved_node_names
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
    except Exception as e:
      import os
      err_text = ''.join([os.path.basename(launchfile),' loading failed!'])
      err_details = ''.join([err_text, '\n\n', e.__class__.__name__, ": ", str(e)])
      rospy.logwarn("Loading launch file: %s", err_details)
      WarningMessageBox(QtGui.QMessageBox.Warning, "Loading launch file", err_text, err_details).exec_()
    except:
      import traceback
      print traceback.print_exc()

  def appendConfigToModel(self, launchfile, rosconfig):
    '''
    Update the node view 
    @param launchfile: the launch file path
    @type launchfile: C{str}
    @param rosconfig: the configuration
    @type rosconfig: L{LaunchConfig}
    '''
    hosts = dict() # dict(addr : dict(node : [config]) )
    for n in rosconfig.nodes:
      addr = nm.nameres().address(self.masteruri)
      masteruri = self.masteruri
      if n.machine_name and not n.machine_name == 'localhost':
        if not rosconfig.machines.has_key(n.machine_name):
          raise Exception(''.join(["ERROR: unknown machine [", n.machine_name,"]"]))
        addr = rosconfig.machines[n.machine_name].address
        masteruri = nm.nameres().masteruri(n.machine_name)
      node = roslib.names.ns_join(n.namespace, n.name)
      if not hosts.has_key((masteruri, addr)):
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
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
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
#        host = nm.nameres().address(r[1])
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
    for (name, uri, muri) in added:
      self.default_cfg_handler.requestNodeList(uri, roslib.names.ns_join(name, 'list_nodes'))
      #request the description
      descr_service = self.__master_info.getService(roslib.names.ns_join(name, 'description'))
      if not descr_service is None:
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
    if not self.__master_info is None:
      service = self.__master_info.getService(config_name)
      if not service is None:
        masteruri = service.masteruri
    key = (roslib.names.namespace(config_name).rstrip(roslib.names.SEP), service_uri, masteruri)
#    if self.__configs.has_key(key):
#      self.node_tree_model.removeConfigNodes(key)
    # add the new config
    node_cfgs = dict()
    for n in nodes:
      node_cfgs[n] = key
    host = nm.nameres().getHostname(service_uri)
    host_addr = nm.nameres().address(host)
    if host_addr is None:
      host_addr = host
    self.node_tree_model.appendConfigNodes(masteruri, host_addr, node_cfgs)
    self.__configs[key] = nodes
    self.updateButtons()

  def on_default_cfg_descr_retrieved(self, service_uri, config_name, items):
    '''
    Handles the description list from default configuration service.
    Emits a Qt signal L{host_description_updated} to notify about a new host 
    description and a Qt signal L{capabilities_update} to notify about a capabilities
    update.
    @param service_uri: the URI of the service provided the default configuration
    @type service_uri: C{str}
    @param config_name: the name of default configuration service
    @type config_name: C{str}
    @param items: list with descriptions
    @type items: C{[L{default_cfg_fkie.Description}]}
    '''
    if items:
      masteruri = self.masteruri
      if not self.__master_info is None:
        service = self.__master_info.getService(config_name)
        if not service is None:
          masteruri = service.masteruri
      key = (roslib.names.namespace(config_name).rstrip(roslib.names.SEP), service_uri, masteruri)
      host = nm.nameres().getHostname(service_uri)
      host_addr = nm.nameres().address(host)
      #add capabilities
      caps = dict()
      for c in items[0].capabilities:
        if not caps.has_key(c.namespace):
          caps[c.namespace] = dict()
        caps[c.namespace][c.name.decode(sys.getfilesystemencoding())] = { 'type' : c.type, 'images' : c.images, 'description' : c.description.replace("\\n ", "\n").decode(sys.getfilesystemencoding()), 'nodes' : list(c.nodes) }
      if host_addr is None:
        host_addr = nm.nameres().getHostname(key[1])
      self.node_tree_model.addCapabilities(masteruri, host_addr, key, caps)
      # set host description
      tooltip = self.node_tree_model.updateHostDescription(masteruri, host_addr, items[0].robot_type, items[0].robot_name.decode(sys.getfilesystemencoding()), items[0].robot_descr.decode(sys.getfilesystemencoding()))
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
#    QtGui.QMessageBox.warning(self, 'Error while call %s'%service,
#                              str(msg),
#                              QtGui.QMessageBox.Ok)

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Handling of the view activities                  %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_node_activated(self, index):
    '''
    Depending of the state of the node, it will be run or the screen output will
    be open. 
    @param index: The index of the activated node
    @type index: L{PySide.QtCore.QModelIndex}
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes(), False)
    if not selectedNodes:
      return
    has_running = False
    has_stopped = False
    has_invalid = False
    for node in selectedNodes:
      if not node.uri is None:
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
    self.__last_info_type = 'Node'
    self.on_node_selection_changed(None, None, True)


  def on_topic_activated(self, index):
    '''
    @param index: The index of the activated topic
    @type index: L{PySide.QtCore.QModelIndex}
    '''
    self.on_topic_echo_clicked()

  def on_topic_clicked(self, index):
    self.__last_info_type = 'Topic'
    self.on_topic_selection_changed(None, None, True)

  def on_service_activated(self, index):
    '''
    @param index: The index of the activated service
    @type index: L{PySide.QtCore.QModelIndex}
    '''
    self.on_service_call_clicked()

  def on_service_clicked(self, index):
    self.__last_info_type = 'Service'
    self.on_service_selection_changed(None, None, True)

  def on_host_inserted(self, item):
    if item.id == (self.masteruri, nm.nameres().getHostname(self.masteruri)):
      index = self.node_tree_model.indexFromItem(item)
      if index.isValid():
        self.masterTab.nodeTreeView.expand(index)
#    self.masterTab.nodeTreeView.expandAll()

  def on_node_collapsed(self, index):
    if not index.parent ().isValid():
      self.masterTab.nodeTreeView.selectionModel().clear()
    
  def on_node_expanded(self, index):
    pass

  def _create_html_list(self, title, items, type=None):
    '''
    
    :param type: LAUNCH, TOPIC, NODE, SERVICE
    
    :type type: str
    '''
    result = ''
    if items:
      result = ''.join([result, '<b><u>', title,'</u></b>'])
      if len(items) > 1:
        result = ''.join([result, ' [', str(len(items)),']'])
      result = ''.join([result, '<ul>'])
      for i in items:
        item = i
        if type == 'TOPIC':
#          item = ''.join([i, ' <a href="topic://', str(i),'">[echo] <a href="topichz://', str(i),'">[hz] ', '</a>'])
          item = ''.join([' <a href="topic://', str(i),'">', i, '</a>'])
        elif type == 'SERVICE':
          item = ''.join(['<a href="service://', str(i),'">', i, '</a>'])
        elif type == 'LAUNCH':
          item = ''.join(['<a href="launch://', str(i),'">', i, '</a>'])
        result = ''.join([result, '<li>', item, '</li>'])
      result = ''.join([result, '</ul>'])
    return result

  def on_node_selection_changed(self, selected, deselected, force_emit=False):
    '''
    updates the Buttons, create a description and emit L{description_signal} to
    show the description of host, group or node. 
    '''
    if self.masterTab.tabWidget.tabText(self.masterTab.tabWidget.currentIndex()) != 'Nodes':
      return
    selectedHosts = self.hostsFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    name = ''
    text = ''
    if len(selectedHosts) == 1:
      host = selectedHosts[0]
      name = ' - '.join([host.name, 'Robot'])
      text = host.generateDescription()
    else:
      selectedGroups = self.groupsFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
      if len(selectedGroups) == 1:
        group = selectedGroups[0]
        name = ' - '.join([group.name, 'Group'])
        text = group.generateDescription()
      else:
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        if len(selectedNodes) == 1:
          # create description for a node
          node = selectedNodes[0]
          ns, sep, name = node.name.rpartition(rospy.names.SEP)
          text = ''.join(['<font size="+1"><b>', '<span style="color:gray;">', str(ns), sep, '</span><b>', str(name), '</b></font><br>'])
          text = ''.join([text, '<a href="restart_node://', node.name,'">', 'restart</a> - '])
          text = ''.join([text, '<a href="kill_node://', node.name,'">', 'kill</a> - '])
          text = ''.join([text, '<a href="kill_screen://', node.name,'">', 'kill screen</a>  '])
          text = ''.join([text, '<p>'])
          text = ''.join([text, '<dl>'])
          text = ''.join([text, '<dt><b>URI</b>: ', str(node.node_info.uri), '</dt>'])
          text = ''.join([text, '<dt><b>PID</b>: ', str(node.node_info.pid), '</dt>'])
          text = ''.join([text, '<dt><b>ORG.MASTERURI</b>: ', str(node.node_info.masteruri), '</dt>'])
          if node.node_info.masteruri != self.masteruri and not node.node_info.uri is None:
            text = ''.join([text, '<dt><font color="#339900"><b>synchronized</b></font></dt>'])
          if node.node_info.pid is None and not node.node_info.uri is None:
            if not node.node_info.isLocal:
              text = ''.join([text, '<dt><font color="#FF9900"><b>remote nodes will not be ping, so they are always marked running</b></font>'])
            else:
              text = ''.join([text, '<dt><font color="#CC0000"><b>the node does not respond: </b></font>'])
              text = ''.join([text, '<a href="unregister_node://', node.name,'">', 'unregister</a></dt>'])
          text = ''.join([text, '</dl>'])
          text = ''.join([text, self._create_html_list('Published Topics:', node.published, 'TOPIC')])
          text = ''.join([text, self._create_html_list('Subscribed Topics:', node.subscribed, 'TOPIC')])
          text = ''.join([text, self._create_html_list('Services:', node.services, 'SERVICE')])
          launches = []
          default_cfgs = []
          for c in node.cfgs:
            if isinstance(c, tuple):
              default_cfgs.append(c[0])
            else:
              launches.append(c)
          text = ''.join([text, self._create_html_list('Loaded Launch Files:', launches, 'LAUNCH')])
          text = ''.join([text, self._create_html_list('Default Configurations:', default_cfgs)])
          text = ''.join([text, '<dt><a href="copy_log_path://', node.name,'">', 'copy log path to clipboard</a></dt>'])
          text = ''.join(['<div>', text, '</div>'])
          name = node.name

    if self.__last_info_type == 'Node' and (self.__last_info_text != text or force_emit):
      self.__last_info_text = text
      self.description_signal.emit(name, text)
    self.updateButtons()

  def on_topic_selection_changed(self, selected, deselected, force_emit=False):
    '''
    updates the Buttons, create a description and emit L{description_signal} to
    show the description of selected topic
    '''
    if self.masterTab.tabWidget.tabText(self.masterTab.tabWidget.currentIndex()) != 'Topics':
      return
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    topics_selected = (len(selectedTopics) > 0)
    self.masterTab.echoTopicButton.setEnabled(topics_selected)
    self.masterTab.hzTopicButton.setEnabled(topics_selected)
    self.masterTab.pubStopTopicButton.setEnabled(topics_selected)
    if len(selectedTopics) == 1:
      topic = selectedTopics[0]
      ns, sep, name = topic.name.rpartition(rospy.names.SEP)
      text = ''.join(['<h3>', '<span style="color:gray;">', str(ns), sep, '</span><b>', str(name), '</h3>'])
      text = ''.join([text, self._create_html_list('Publisher:', topic.publisherNodes, 'NODE')])
      text = ''.join([text, self._create_html_list('Subscriber:', topic.subscriberNodes, 'NODE')])
      text = ''.join([text, '<b><u>Type:</u></b> ', str(self._href_from_msgtype(topic.type))])
      text = ''.join([text, '<dl>'])
      try:
        mclass = roslib.message.get_message_class(topic.type)
        if not mclass is None:
          text = ''.join([text, '<ul>'])
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
##                primitive = "class", list_msg_class.__slots__
#              except ValueError:
#                pass
            text = ''.join([text, '<li>', str(f), ': ', str(idtype), '</li>'])
          text = ''.join([text, '</ul>'])
      except ValueError:
        pass
      text = ''.join([text, '</dl>'])
      info_text = ''.join(['<div>', text, '</div>'])
      if self.__last_info_type == 'Topic' and (self.__last_info_text != info_text or force_emit):
        self.__last_info_text = info_text
        self.description_signal.emit(topic.name, info_text)
  
  def _href_from_msgtype(self, type):
    result = type
    if type:
      result = ''.join(['<a href="http://ros.org/doc/api/', type.replace('/', '/html/msg/'), '.html">', type, '</a>'])
    return result

  def on_service_selection_changed(self, selected, deselected, force_emit=False):
    '''
    updates the Buttons, create a description and emit L{description_signal} to
    show the description of selected service
    '''
    if self.masterTab.tabWidget.tabText(self.masterTab.tabWidget.currentIndex()) != 'Services':
      return
    selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
    self.masterTab.callServiceButton.setEnabled(len(selectedServices) > 0)
    if len(selectedServices) == 1:
      service = selectedServices[0]
      ns, sep, name = service.name.rpartition(rospy.names.SEP)
      text = ''.join(['<h3>', '<span style="color:gray;">', str(ns), sep, '</span><b>', str(name), '</h3>'])
      text = ''.join([text, '<dl>'])
      text = ''.join([text, '<dt><b>URI</b>: ', str(service.uri), '</dt>'])
      text = ''.join([text, '<dt><b>ORG.MASTERURI</b>: ', str(service.masteruri), '</dt>'])
      if service.masteruri != self.masteruri:
        text = ''.join([text, '<dt><font color="#339900"><b>synchronized</b></font></dt>'])
      text = ''.join([text, '</dl>'])
      try:
        service_class = service.get_service_class(nm.is_local(nm.nameres().getHostname(service.uri)))
        text = ''.join([text, '<h4>', self._href_from_svrtype(service_class._type), '</h4>'])
        text = ''.join([text, '<b><u>', 'Request', ':</u></b>'])
        text = ''.join([text, '<dl><dt>', str(service_class._request_class.__slots__), '</dt></dl>'])
  
        text = ''.join([text, '<b><u>', 'Response', ':</u></b>'])
        text = ''.join([text, '<dl><dt>', str(service_class._response_class.__slots__), '</dt></dl>'])
      except:
        pass
      info_text = ''.join(['<div>', text, '</div>'])
      if self.__last_info_type == 'Service' and (self.__last_info_text != info_text or force_emit):
        self.__last_info_text = info_text
        self.description_signal.emit(service.name, info_text)

  def _href_from_svrtype(self, type):
    result = type
    if type:
      result = ''.join(['<a href="http://ros.org/doc/api/', type.replace('/', '/html/srv/'), '.html">', type, '</a>'])
    return result

  def on_parameter_selection_changed(self, selected, deselected):
    selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
    self.masterTab.deleteParameterButton.setEnabled(len(selectedParameter) > 0)

  def hostsFromIndexes(self, indexes, recursive=True):
    result = []
    for index in indexes:
      if index.column() == 0:
        item = self.node_tree_model.itemFromIndex(index)
        if not item is None:
          if isinstance(item, HostItem):
            result.append(item)
    return result

  def groupsFromIndexes(self, indexes, recursive=True):
    result = []
    for index in indexes:
      if index.column() == 0 and index.parent().isValid():
        item = self.node_tree_model.itemFromIndex(index)
        if not item is None:
          if isinstance(item, GroupItem):
            result.append(item)
    return result

  def nodesFromIndexes(self, indexes, recursive=True):
    result = []
    for index in indexes:
      if index.column() == 0:
        item = self.node_tree_model.itemFromIndex(index)
        res = self._nodesFromItems(item, recursive)
        for r in res:
          if not r in result:
            result.append(r)
    return result

  def _nodesFromItems(self, item, recursive):
    result = []
    if not item is None:
      if isinstance(item, (GroupItem, HostItem)):
        if recursive:
          for j in range(item.rowCount()):
            result[len(result):] = self._nodesFromItems(item.child(j), recursive)
      elif isinstance(item, NodeItem):
        if not item in result:
          result.append(item)
    return result

  def topicsFromIndexes(self, indexes):
    result = []
    for index in indexes:
      model_index = self.topic_proxyModel.mapToSource(index)
      item = self.topic_model.itemFromIndex(model_index)
      if not item is None and isinstance(item, TopicItem):
        result.append(item.topic)
    return result

  def servicesFromIndexes(self, indexes):
    result = []
    for index in indexes:
      model_index = self.service_proxyModel.mapToSource(index)
      item = self.service_model.itemFromIndex(model_index)
      if not item is None and isinstance(item, ServiceItem):
        result.append(item.service)
    return result

  def parameterFromIndexes(self, indexes):
    result = []
    for index in indexes:
      model_index = self.parameter_proxyModel.mapToSource(index)
      item = self.parameter_model.itemFromIndex(model_index)
      if not item is None and isinstance(item, ParameterValueItem):
        result.append((item.name, item.value))
    return result


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Handling of the button activities                %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_start_clicked(self):
    '''
    Starts the selected nodes. If for a node more then one configuration is 
    available, the selection dialog will be show.
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
      self.masterTab.startButton.showMenu()
    else:
      cursor = self.cursor()
      self.masterTab.startButton.setEnabled(False)
      self.setCursor(QtCore.Qt.WaitCursor)
      try:
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        self.start_nodes(selectedNodes)
      finally:
        self.setCursor(cursor)
        self.masterTab.startButton.setEnabled(True)

  def start_node(self, node, force, config, force_host=None):

    if node is None:
        raise DetailedError("Start error", 'None is not valid node name!') 
    if node.pid is None or force:
      # start the node using launch configuration
      if config is None:
        raise DetailedError("Start error", 
                          ''.join(['Error while start ', node.name, ':\nNo configuration found!']))
      if isinstance(config, LaunchConfig):
        try:
          nm.starter().runNode(node.name, config, force_host, self.masteruri)
        except socket.error as se:
          rospy.logwarn("Error while start '%s': %s\n\n Start canceled!", node.name, str(se))
          raise DetailedError("Start error", 
                              ''.join(['Error while start ', node.name, '\n\nStart canceled!']),
                              str(se))
          return False
        except nm.InteractionNeededError as ie:
          raise
        except (Exception, nm.StartException) as e:
          print type(e)
          import traceback
          print traceback.format_exc()
          rospy.logwarn("Error while start '%s': %s", node.name, str(e))
          raise DetailedError("Start error", ''.join(['Error while start ', node.name]), str(e))
      elif isinstance(config, (str, unicode)):
        # start with default configuration
        from multimaster_msgs_fkie.srv import Task
        try:
          nm.starter().callService(self.master_info.getService(config).uri, config, Task, [node.name])
        except (Exception, nm.StartException) as e:
#          socket_error =  (str(e).find("timeout") or str(e).find("113"))
          rospy.logwarn("Error while call a service of node '%s': %s", node.name, str(e))
          raise DetailedError("Service error", 
                              ''.join(['Error while call a service of node ', node.name, '[', self.master_info.getService(config).uri, ']']),
                              str(e))

  def start_nodes(self, nodes, force=False, force_host=None):
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
    for node in nodes:
      if node.pid is None or (not node.pid is None and force):
        # test for duplicate nodes
        if node.uri is None and self.node_tree_model.isDuplicateNode(node.name):
          ret = QtGui.QMessageBox.question(self, 'Question', ''.join(['Some nodes, e.g. ', node.name, ' are already running on another host. If you start this node the other node will be terminated.\n Do you want proceed?']), QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
          if ret == QtGui.QMessageBox.No:
            return
        # determine the used configuration
        choices = self._getCfgChoises(node)
        ch_keys = choices.keys()
        if ch_keys:
          ch_keys.sort()
          choises_str = str(ch_keys)
          if not choises_str in cfg_choices.keys():
            choice = self._getUserCfgChoice(choices, node.name)
            if not choice is None:
              cfg_choices[choises_str] = choices[choice]
              cfg_nodes[node.name] = choices[choice]
            else:
              res = WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                              ''.join(['Error while start ', node.name, ':\nNo configuration selected!'])).exec_()
          else:
            cfg_nodes[node.name] = cfg_choices[choises_str]

    # put into the queue and start
    for node in nodes:
      if node.name in cfg_nodes:
        self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                       ''.join(['start ', node.node_info.name]), 
                                       self.start_node, 
                                       (node.node_info, force, cfg_nodes[node.node_info.name], force_host))
    self._progress_queue.start()

  def start_nodes_by_name(self, nodes, cfg, force=False):
    '''
    Start nodes given in a list by their names.
    @param nodes: a list with full node names
    @type nodes: C{[str]}
    '''
    result = []
    if not self.master_info is None:
      for n in nodes:
        node_info = NodeInfo(n, self.masteruri)
        node_item = NodeItem(node_info)
        node_item.addConfig(cfg)
        result.append(node_item)
    self.start_nodes(result, force)

  def on_force_start_nodes(self):
    '''
    Starts the selected nodes (also if it already running). If for a node more then one configuration is 
    available, the selection dialog will be show.
    '''
    cursor = self.cursor()
    self.masterTab.startButton.setEnabled(False)
    self.setCursor(QtCore.Qt.WaitCursor)
    try:
      selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
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
    params = {'Host' : ('string', 'localhost') }
    dia = ParameterDialog(params)
    dia.setFilterVisible(False)
    dia.setWindowTitle('Start node on...')
    dia.resize(350,120)
    dia.setFocusField('host')
    if dia.exec_():
      try:
        params = dia.getKeywords()
        host = params['Host']
        self.setCursor(QtCore.Qt.WaitCursor)
        try:
          selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
          self.start_nodes(selectedNodes, True, host)
        finally:
          self.setCursor(cursor)
      except Exception, e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                          'Error while parse parameter',
                          str(e)).exec_()
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
          result[''.join([str(launch.LaunchName), ' [', str(launch.PackageName), ']'])] = self.launchfiles[c]
        elif not ignore_defaults:
          result[' '.join(['[default]', c[0]])] = roslib.names.ns_join(c[0], 'run')
    return result

  def _getUserCfgChoice(self, choices, nodename):
    value = None
    # Open selection
    if len(choices) == 1:
      value = choices.keys()[0]
    elif len(choices) > 0:
      items = SelectDialog.getValue('Configuration selection', choices.keys(), True)
      if items:
        value = items[0]
    return value

  def on_stop_clicked(self):
    '''
    Stops the selected and running nodes. If the node can't be stopped using his
    RPC interface, it will be unregistered from the ROS master using the masters
    RPC interface.
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
      self.masterTab.stopButton.showMenu()
    else:
      cursor = self.cursor()
      self.setCursor(QtCore.Qt.WaitCursor)
      try:
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        self.stop_nodes(selectedNodes)
      finally:
        self.setCursor(cursor)
  
  def stop_node(self, node, force=False):
    if not node is None and not node.uri is None and (not self._is_in_ignore_list(node.name) or force):
      try:
        #'print "STOP set timeout", node
        socket.setdefaulttimeout(10)
        #'print "STOP create xmlrpc", node
        p = xmlrpclib.ServerProxy(node.uri)
        #'print "STOP send stop", node
        p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', self.mastername]))
        #'print "STOP stop finished", node
      except Exception, e:
#            import traceback
#            formatted_lines = traceback.format_exc().splitlines()
        rospy.logwarn("Error while stop node '%s': %s", str(node.name), str(e))
        if str(e).find(' 111') == 1:
#          self.masterTab.stopButton.setEnabled(False)
          raise DetailedError("Stop error", 
                              ''.join(['Error while stop node ', node.name]),
                              str(e))
      finally:
        socket.setdefaulttimeout(None)
    return True
    
  def stop_nodes(self, nodes):
    '''
    Internal method to stop a list with nodes
    @param nodes: the list with nodes to stop
    @type nodes: C{[L{master_discovery_fkie.NodeInfo}, ...]}
    '''
    # put into the queue and start the que handling
    for node in nodes:
      self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                     ''.join(['stop ', node.name]), 
                                     self.stop_node, 
                                     (node, (len(nodes)==1)))
    self._progress_queue.start()

  def stop_nodes_by_name(self, nodes):
    '''
    Stop nodes given in a list by their names.
    @param nodes: a list with full node names
    @type nodes: C{[str]}
    '''
    result = []
    if not self.master_info is None:
      for n in nodes:
        node = self.master_info.getNode(n)
        if not node is None:
          result.append(node)
    self.stop_nodes(result)

  def kill_node(self, node, force=False):
    if not node is None and not node.uri is None and (not self._is_in_ignore_list(node.name) or force):
      pid = node.pid
      if pid is None:
        # try to get the process id of the node
        try:
          socket.setdefaulttimeout(10)
          rpc_node = xmlrpclib.ServerProxy(node.uri)
          code, msg, pid = rpc_node.getPid(rospy.get_name())
        except:
#            self.masterTab.stopButton.setEnabled(False)
          pass
        finally:
          socket.setdefaulttimeout(None)
      # kill the node
      if not pid is None:
        try:
          self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                         ''.join(['kill ', node.name, '(', str(pid), ')']), 
                                         nm.starter().kill, 
                                         (self.getHostFromNode(node), pid, False))
          self._progress_queue.start()
        except Exception as e:
          rospy.logwarn("Error while kill the node %s: %s", str(node.name), str(e))
          raise DetailedError("Kill error", 
                              ''.join(['Error while kill the node ', node.name]),
                              str(e))
    return True


  def on_kill_nodes(self):
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())

    # put into the queue and start the que handling
    for node in selectedNodes:
      self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                     ''.join(['kill ', node.name]), 
                                     self.kill_node, 
                                     (node, (len(selectedNodes)==1)))
    self._progress_queue.start()

  def unregister_node(self, node, force=False):
    if not node is None and not node.uri is None and (not self._is_in_ignore_list(node.name) or force):
      # stop the node?
#        try:
#          p = xmlrpclib.ServerProxy(node.uri)
#          p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', self.hostname]))
#        except Exception, e:
#          rospy.logwarn("Error while stop node '%s': %s", str(node.name), str(e))
#          self.masterTab.stopButton.setEnabled(False)
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
        if not self.master_state is None:
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
        rospy.logwarn("Error while unregister node %s: %s", str(node.name), str(e))
        raise DetailedError("Unregister error", 
                            ''.join(['Error while Unregister node ', node.name]),
                            str(e))
      finally:
        socket.setdefaulttimeout(None)
    return True
    
  def on_unregister_nodes(self):
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    # put into the queue and start the que handling
    for node in selectedNodes:
      self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                     ''.join(['unregister node ', node.name]), 
                                     self.unregister_node, 
                                     (node, (len(selectedNodes)==1)))
    self._progress_queue.start()

  def on_stop_context_toggled(self, state):
    menu = QtGui.QMenu(self)
    self.killAct = QtGui.QAction("&Kill Node", self, shortcut=QtGui.QKeySequence.New, statusTip="Kill selected node", triggered=self.kill_nodes)
    self.unregAct = QtGui.QAction("&Unregister Nodes...", self, shortcut=QtGui.QKeySequence.Open, statusTip="Open an existing file", triggered=self.unreg_nodes)
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
    @type node: L{master_discovery_fkie.NodeInfo}
    '''
    if not node.uri is None:
      return nm.nameres().getHostname(node.uri)
    # try to get it from the configuration
    for c in node.cfgs:
      if not isinstance(c, tuple):
        launch_config = self.__configs[c]
        item = launch_config.getNode(node.name)
        if not item is None and item.machine_name and not item.machine_name == 'localhost':
          return launch_config.Roscfg.machines[item.machine_name].address
    # return the host of the assigned ROS master
    return nm.nameres().getHostname(node.masteruri)
  
  def on_io_clicked(self):
    '''
    Shows IO of the selected nodes.
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
      self.masterTab.ioButton.showMenu()
    else:
      cursor = self.cursor()
      self.setCursor(QtCore.Qt.WaitCursor)
      try:
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        if selectedNodes:
          ret = True
          if len(selectedNodes) > 5:
            ret = QtGui.QMessageBox.question(self, "Show IO", "You are going to open the IO of "+ str(len(selectedNodes)) + " nodes at once\nContinue?", QtGui.QMessageBox.Ok, QtGui.QMessageBox.Cancel)
            ret = (ret == QtGui.QMessageBox.Ok)
          if ret:
            for node in selectedNodes:
              self._progress_queue.add2queue(str(self._progress_queue.count()),
                                             ''.join(['show IO of ', node.name]),
                                             nm.screen().openScreen,
                                             (node.name, self.getHostFromNode(node), False))
            self._progress_queue.start()
        else:
          self.on_show_all_screens()
      finally:
        self.setCursor(cursor)

  def on_kill_screens(self):
    '''
    Kills selected screens, if some available.
    '''
    cursor = self.cursor()
    self.setCursor(QtCore.Qt.WaitCursor)
    try:
      selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
      for node in selectedNodes:
        self._progress_queue.add2queue(str(self._progress_queue.count()),
                                       ''.join(['kill screen of ', node.name]),
                                       nm.screen().killScreens,
                                       (node.name, self.getHostFromNode(node), False))
      self._progress_queue.start()
    finally:
      self.setCursor(cursor)

  def on_show_all_screens(self):
    '''
    Shows all available screens.
    '''
    cursor = self.cursor()
    self.setCursor(QtCore.Qt.WaitCursor)
    try:
      host = nm.nameres().getHostname(self.masteruri)
      sel_screen = []
      try:
        screens = nm.screen().getActiveScreens(host, auto_pw_request=True)
        sel_screen = SelectDialog.getValue('Open screen', screens, False, False, self)
      except Exception, e:
        rospy.logwarn("Error while get screen list: %s", str(e))
        WarningMessageBox(QtGui.QMessageBox.Warning, "Screen list error", 
                          ''.join(['Error while get screen list from ', host]),
                          str(e)).exec_()
      for screen in sel_screen:
        try:
          if not nm.screen().openScreenTerminal(host, screen, screen):
  #          self.masterTab.ioButton.setEnabled(False)
            pass
        except Exception, e:
          rospy.logwarn("Error while show IO for %s: %s", str(screen), str(e))
          WarningMessageBox(QtGui.QMessageBox.Warning, "Show IO error", 
                            ''.join(['Error while show IO ', screen, ' on ', host]),
                            str(e)).exec_()
    finally:
      self.setCursor(cursor)


  def on_log_clicked(self):
    '''
    Shows log files of the selected nodes.
    '''
    try:
      key_mod = QtGui.QApplication.keyboardModifiers()
      if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
        self.masterTab.logButton.showMenu()
      else:
        selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
        ret = True
        if len(selectedNodes) > 5:
          ret = QtGui.QMessageBox.question(self, "Show Log", "You are going to open the logs of "+ str(len(selectedNodes)) + " nodes at once\nContinue?", QtGui.QMessageBox.Ok, QtGui.QMessageBox.Cancel)
          ret = (ret == QtGui.QMessageBox.Ok)
        if ret:
          for node in selectedNodes:
            self._progress_queue.add2queue(str(self._progress_queue.count()),
                                           ''.join(['show log of ', node.name]),
                                           nm.starter().openLog,
                                           (node.name, self.getHostFromNode(node)))
          self._progress_queue.start()
    except Exception, e:
      import traceback
      print traceback.format_exc()
      rospy.logwarn("Error while show log: %s", str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Show log error", 
                        'Error while show Log',
                        str(e)).exec_()

  def on_log_path_copy(self):
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    nodenames = []
    for n in selectedNodes:
      nodenames.append(n.name)
    try:
      host = nm.nameres().getHostname(self.masteruri)
      socket.setdefaulttimeout(3)
      path_on_host = nm.starter().copylogPath2Clipboards(host, nodenames, True)
      socket.setdefaulttimeout(None)
      user = nm.ssh().USER_DEFAULT
      try:
        user = nm.ssh().SSH_AUTH[host]
      except:
        pass
      QtGui.QApplication.clipboard().setText(''.join([user, '@', host, ':', path_on_host]))
    except Exception as e:
      WarningMessageBox(QtGui.QMessageBox.Warning, "Get log path", 
                        'Error while get log path',
                        str(e)).exec_()
#    self._progress_queue.add2queue(str(self._progress_queue.count()),
#                                   'Get log path',
#                                   nm.starter().copylogPath2Clipboards,
#                                   (nm.nameres().getHostname(self.masteruri), nodenames))
#    self._progress_queue.start()

#  def on_log_show_selected(self):
#    try:
#      nm.screen().LOG_PATH.
#      screens = nm.screen().getActiveScreens(host, auto_pw_request=True)
#      sel_screen = SelectDialog.getValue('Open log', screens, False, self)
#    except Exception, e:
#      rospy.logwarn("Error while get screen list: %s", str(e))
#      WarningMessageBox(QtGui.QMessageBox.Warning, "Screen list error", 
#                        ''.join(['Error while get screen list from ', host]),
#                        str(e)).exec_()

  def on_log_delete_clicked(self):
    '''
    Deletes log files of the selected nodes.
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    for node in selectedNodes:
      self._progress_queue.add2queue(str(self._progress_queue.count()),
                                     ''.join(['delete Log of ', node.name]),
                                     nm.starter().deleteLog,
                                     (node.name, self.getHostFromNode(node), False))
    self._progress_queue.start()

  def on_dynamic_config_clicked(self):
    '''
    Opens the dynamic configuration dialogs for selected nodes.
    '''
    if not self.master_info is None:
      selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
      for n in selectedNodes:
        try:
          nodes = sorted([s[:-len('/set_parameters')] for s in self.master_info.services.keys() if (s.endswith('/set_parameters') and s.startswith(str(n.name)))])
          node = None
          if len(nodes) == 1:
            node = nodes[0]
          elif len(nodes) > 1:
            items = SelectDialog.getValue('Dynamic configuration selection', [i for i in nodes], True)
            if items:
              node = items[0]
          if not node is None:
#            self.masterTab.dynamicConfigButton.setEnabled(False)
            import os, subprocess
            env = dict(os.environ)
            env["ROS_MASTER_URI"] = str(self.master_info.masteruri)
            ps = subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'reconfigure_gui', node], env=env)
            # wait for process to avoid 'defunct' processes
            thread = threading.Thread(target=ps.wait)
            thread.setDaemon(True)
            thread.start()
        except Exception, e:
          rospy.logwarn("Start dynamic reconfiguration for '%s' failed: %s", str(node), str(e))
          WarningMessageBox(QtGui.QMessageBox.Warning, "Start dynamic reconfiguration error", 
                            ''.join(['Start dynamic reconfiguration for ', str(node), ' failed!']),
                            str(e)).exec_()

  def on_edit_config_clicked(self):
    '''
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    for node in selectedNodes:
      choices = self._getCfgChoises(node, True)
      choice = self._getUserCfgChoice(choices, node.name)
      config = choices[choice] if choices and choice else ''
      if isinstance(config, LaunchConfig):
        # get the file, which include the node and the main configuration file
        node_cfg = config.getNode(node.name)
        files = [config.Filename]
        if not node_cfg.filename in files:
          files.append(node_cfg.filename)
        self.request_xml_editor.emit(files, ''.join(['name="', os.path.basename(node.name), '"']))

  def on_edit_rosparam_clicked(self):
    '''
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    for node in selectedNodes:
      # set the parameter in the ROS parameter server
      try:
        inputDia = MasterParameterDialog(node.masteruri if not node.masteruri is None else self.masteruri, ''.join([node.name, roslib.names.SEP]), parent=self)
        inputDia.setWindowTitle(' - '.join([os.path.basename(node.name), "parameter"]))
        if node.has_launch_cfgs(node.cfgs):
          inputDia.add_warning("The changes may not have any effect, because the launch file was also loaded as not 'default' and the parameter in the launch file will be reloaded on start of the ROS node.")
        inputDia.show()
      except:
        import traceback
        rospy.logwarn("Error on retrieve parameter for %s: %s", str(node.name), str(traceback.format_exc()))

  def on_save_clicked(self):
    (fileName, filter) = QtGui.QFileDialog.getSaveFileName(self,
                                                 "New launch file", 
                                                 self.__current_path, 
                                                 "Config files (*.launch);;All files (*)")
    if fileName:
      self.__current_path = os.path.dirname(fileName)
      try:
        (pkg, pkg_path) = package_name(os.path.dirname(fileName))
        if pkg is None:
          ret = WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                                  'The new file is not in a ROS package', buttons=QtGui.QMessageBox.Ok|QtGui.QMessageBox.Cancel).exec_()
          if ret == QtGui.QMessageBox.Cancel:
            return
        with open(fileName, 'w+') as f:
          f.write("<launch>\n")
          for lfile in self.launchfiles.keys():
            with open(lfile, 'r') as lf:
              f.write(re.sub('<\/?launch\ *\t*>', '', lf.read()))
          f.write("</launch>\n")
      except EnvironmentError as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                         'Error while create a new file',
                          str(e)).exec_()

  def on_close_clicked(self):
    '''
    Closes the open launch configurations. If more then one configuration is 
    open a selection dialog will be open.
    '''
    choices = dict()

    for path, cfg in self.__configs.items():
      if isinstance(path, tuple):
        if path[2] == self.masteruri:
          choices[''.join(['[', path[0], ']'])] = path
      else:
        choices[''.join([os.path.basename(path), '   [', str(package_name(os.path.dirname(path))[0]), ']'])] = path
    cfgs = []
    
#    if len(choices) > 1:
    cfgs = SelectDialog.getValue('Close configurations', choices.keys(), False, False, self)
#    elif len(choices) == 1:
#      cfgs = choices.values()[0]

    # close configurations
    for c in cfgs:
      self._close_cfg(choices[c])
    self.updateButtons()

  def _close_cfg(self, cfg):
    try:
      self.removeConfigFromModel(cfg)
      if isinstance(cfg, tuple):
        if not self.master_info is None:
          # close default configuration: stop the default_cfg node
          node = self.master_info.getNode(cfg[0])
          if not node is None:
            self.stop_nodes([node])
#      else:
#        # remove machine entries
#        try:
#          for name, machine in self.__configs[cfg].Roscfg.machines.items():
#            if machine.name:
#              nm.nameres().remove(name=machine.name, host=machine.address)
#        except:
#          import traceback
#          print traceback.format_exc()
      else:
        # remove from name resolution
        try:
          for name, machine in self.__configs[cfg].Roscfg.machines.items():
            if machine.name:
              nm.nameres().removeInfo(machine.name, machine.address)
        except:
          pass
      del self.__configs[cfg]
    except:
      import traceback
      print traceback.format_exc()
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

  def on_topic_pub_clicked(self):
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    if len(selectedTopics) > 0:
      for topic in selectedTopics:
        if not self._start_publisher(topic.name, topic.type):
          break
    else: # create a new topic
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
          for file in rosmsg._list_types('/'.join([direc, 'msg']), 'msg', rosmsg.MODE_MSG):
            msg_types.append("%s/%s"%(p, file))
      msg_types.sort()
      fields = {'Type' : ('string', msg_types), 'Name' : ('string', [''])}
      
      #create a dialog
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
              import traceback
              print traceback.format_exc()
              rospy.logwarn("Publish topic '%s' failed: %s", str(params['Name']), str(e))
              WarningMessageBox(QtGui.QMessageBox.Warning, "Publish topic error", 
                                ''.join(['Publish topic ', params['Name'], ' failed!']),
                                str(e)).exec_()
          else:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Invalid name or type", 
                                ''.join(["Can't publish to topic '", params['Name'], "' with type '", params['Type'], "'!"])).exec_()
        except (KeyError, ValueError), e:
          WarningMessageBox(QtGui.QMessageBox.Warning, "Warning", 
                            'Error while add a parameter to the ROS parameter server',
                            str(e)).exec_()

  def _start_publisher(self, topic_name, topic_type):
    try:
      topic_name = roslib.names.ns_join(roslib.names.SEP, topic_name)
      mclass = roslib.message.get_message_class(topic_type)
      if mclass is None:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Publish error", 
                          'Error while publish to %s'%topic_name,
                          ''.join(['invalid message type: ', topic_type,'.\nIf this is a valid message type, perhaps you need to run "rosmake"'])).exec_()
        return
      slots = mclass.__slots__
      types = mclass._slot_types
      args = ServiceDialog._params_from_slots(slots, types)
      p = { '! Publish rate' : ('string', ['once', 'latch', '1']), topic_type : ('dict', args) }
      dia = ParameterDialog(p)
      dia.setWindowTitle(''.join(['Publish to ', topic_name]))
      dia.resize(450,300)
      dia.setFocusField('! Publish rate')

      if dia.exec_():
        params = dia.getKeywords()
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
            i = int(rate)
            if i > 0:
              opt_str = ''.join(['-r ', rate])
              opt_name_suf = ''.join(['__', rate, 'Hz_'])
          except:
            pass
        # remove empty lists
        topic_params = self._rem_empty_lists(params[topic_type])
#        print params[topic_type]
#        print topic_params
        pub_cmd = ' '.join(['pub', topic_name, topic_type, '"', str(topic_params), '"', opt_str])
#        nm.starter().runNodeWithoutConfig(nm.nameres().address(self.masteruri), 'rostopic', 'rostopic', ''.join(['rostopic_pub', topic_name, opt_name_suf, str(rospy.Time.now())]), args=[pub_cmd], masteruri=self.masteruri)
        self._progress_queue.add2queue(str(self._progress_queue.count()), 
                                 ''.join(['start publisher for ', topic_name]), 
                                 nm.starter().runNodeWithoutConfig, 
                                 (nm.nameres().address(self.masteruri), 'rostopic', 'rostopic', ''.join(['rostopic_pub', topic_name, opt_name_suf, str(rospy.Time.now())]), [pub_cmd], self.masteruri))
        self._progress_queue.start()
        return True
      else:
        return False
    except Exception, e:
      rospy.logwarn("Publish topic '%s' failed: %s", str(topic_name), str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Publish topic error", 
                        ''.join(['Publish topic ', topic_name, ' failed!']),
                        str(e)).exec_()
      import traceback
      print traceback.format_exc()
      return False

  def _rem_empty_lists(self, param_dict):
    result = dict()
    for key, value in param_dict.iteritems():
      if isinstance(value, dict):
        result[key] = self._rem_empty_lists(value)
      elif not (isinstance(value, list) and not value):
        result[key] = value
    return result

  def on_topic_pub_stop_clicked(self):
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    if not self.master_info is None:
      nodes2stop = []
      for topic in selectedTopics:
        topic_prefix = ''.join(['/rostopic_pub', topic.name, '_'])
        node_names = self.master_info.node_names
        for n in node_names:
          
          if n.startswith(topic_prefix):
            nodes2stop.append(n)
      self.stop_nodes_by_name(nodes2stop)

  def _show_topic_output(self, show_hz_only):
    '''
    Shows the output of the topic in a terminal.
    '''
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    for topic in selectedTopics:
      self._add_topic_output2queue(topic, show_hz_only)

  def show_topic_output(self, topic_name, show_hz_only):
    '''
    Shows the topic output in a new window.
    '''
    if not self.master_info is None:
      topic = self.master_info.getTopic(str(topic_name))
      if not topic is None:
        self._add_topic_output2queue(topic, show_hz_only)
  
  def _add_topic_output2queue(self, topic, show_hz_only):
    try:
        # connect to topic on remote host
        import os, shlex, subprocess
        env = dict(os.environ)
        env["ROS_MASTER_URI"] = str(self.masteruri)
        cmd = ' '.join(['rosrun', 'node_manager_fkie', 'node_manager', '-t', topic.name, topic.type, '--hz' if show_hz_only else '', ''.join(['__name:=echo_','hz_' if show_hz_only else '',str(nm.nameres().getHostname(self.masteruri)), topic.name])])
        rospy.loginfo("Echo topic: %s", cmd)
        ps = subprocess.Popen(shlex.split(cmd), env=env, close_fds=True)
        self.__echo_topics_dialogs[topic.name] = ps
        # wait for process to avoid 'defunct' processes
        thread = threading.Thread(target=ps.wait)
        thread.setDaemon(True)
        thread.start()
    except Exception, e:
      rospy.logwarn("Echo topic '%s' failed: %s", str(topic.name), str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Echo of topic error", 
                        ''.join(['Echo of topic ', topic.name, ' failed!']),
                        str(e)).exec_()
  
  def _topic_dialog_closed(self, topic_name):
    if self.__echo_topics_dialogs.has_key(topic_name):
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
    service = self.master_info.getService(str(service_name))
    if not service is None:
      param = ServiceDialog(service, self)
      param.show()

  def on_topic_filter_changed(self, text):
    '''
    Filter the displayed topics
    '''
    self.topic_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.Wildcard))

  def on_service_filter_changed(self, text):
    '''
    Filter the displayed services
    '''
    self.service_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.Wildcard))

  def on_parameter_filter_changed(self, text):
    '''
    Filter the displayed parameter
    '''
    self.parameter_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.Wildcard))

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
    fields = {'name' : ('string', ['', ns]), 'type' : ('string', ['string', 'int', 'float', 'bool']), 'value': ('string', '')}
    newparamDia = ParameterDialog(fields, parent=self)
    newparamDia.setWindowTitle('Add new parameter')
    newparamDia.setFilterVisible(False)
    newparamDia.resize(300, 140)
    if newparamDia.exec_():
      params = newparamDia.getKeywords()
      try:
        if params['type'] == 'int':
          value = int(params['value'])
        elif params['type'] == 'float':
          value = float(params['value'])
        elif params['type'] == 'bool':
          value = bool(params['value'].lower() in ("yes", "true", "t", "1"))
        else:
          value = params['value']
        self.parameterHandler.deliverParameter(self.masteruri, {params['name'] : value})
        self.parameterHandler.requestParameterList(self.masteruri)
      except (KeyError, ValueError), e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Warning", 
                          'Error while add a parameter to the ROS parameter server',
                          str(e)).exec_()

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
      for (key, value) in selectedParameter:
        master_multi.deleteParam(name, key)
      r = master_multi()
      for code, msg, parameter in r:
        if code != 1:
          rospy.logwarn("Error on delete parameter '%s': %s", parameter, msg)
    except:
      import traceback
      rospy.logwarn("Error on delete parameter: %s", str(traceback.format_exc()))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Warning", 
                        'Error while delete a parameter to the ROS parameter server',
                        str(traceback.format_exc())).exec_()
    else:
      self.on_get_parameter_clicked()
    finally:
      socket.setdefaulttimeout(None)

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
          result.append("".join([val]))
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
          import yaml
          value = yaml.load(item.text())
          value = self._replaceDoubleSlash(value)
        else:
          value = item.text()
        self.parameterHandler.deliverParameter(self.masteruri, {item.name : value})
      except ValueError, e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Warning", 
                          'Error while add changes to the ROS parameter server',
                          str(e)).exec_()
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
    @type param: C{[str]}
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
    @type param: C{dict(paramName : (code, statusMessage, parameterValue))}
    '''
    if code == 1:
      result = {}
      for p, (code_n, msg_n, val) in params.items():
        if code_n == 1:
          result[p] = val
        else:
          result[p] = ''
        if p == '/use_sim_time':
          self.__use_sim_time = (code_n == 1 and val)
      self.parameter_model.updateModelData(result)
    else:
      rospy.logwarn("Error on retrieve parameter from %s: %s", str(masteruri), str(msg))

  def _on_delivered_values(self, masteruri, code, msg, params):
    '''
    @param masteruri: The URI of the ROS parameter server
    @type masteruri: C{str}
    @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
    @type code: C{int}
    @param msg: The message of the result. 
    @type msg: C{str}
    @param params: The dictionary the parameter names and request result.
    @type param: C{dict(paramName : (code, statusMessage, parameterValue))}
    '''
    errmsg = ''
    if code == 1:
      for p, (code_n, msg, val) in params.items():
        if code_n != 1:
          errmsg = '\n'.join([errmsg, msg])
    else:
      errmsg = msg if msg else 'Unknown error on set parameter'
    if errmsg:
      WarningMessageBox(QtGui.QMessageBox.Warning, "Warning", 
                        'Error while delivering parameter to the ROS parameter server',
                        errmsg).exec_()

  def _on_sim_param_values(self, masteruri, code, msg, params):
    '''
    @param masteruri: The URI of the ROS parameter server
    @type masteruri: C{str}
    @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
    @type code: C{int}
    @param msg: The message of the result. 
    @type msg: C{str}
    @param params: The dictionary the parameter names and request result.
    @type param: C{dict(paramName : (code, statusMessage, parameterValue))}
    '''
    if code == 1:
      result = {}
      for p, (code_n, msg_n, val) in params.items():
        if p == '/use_sim_time':
          self.__use_sim_time = (code_n == 1 and val)
    else:
      rospy.logwarn("Error on retrieve parameter from %s: %s", str(masteruri), str(msg))


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
      code, message, self._nm_materuri = master.getUri(rospy.get_name())
    return self._nm_materuri


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Shortcuts handling                               %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def select_host_block(self, index):
    '''
    Selects all nodes of a host with given index
    @param index: the index of the host in the tree model
    @type index: C{int}
    '''
    root = self.masterTab.nodeTreeView.model().index(index, 0);
    if not root.isValid():
      return
    self.masterTab.nodeTreeView.expand(root)
#    firstChild = root.child(0, 0)
    last_row_index = len(self.node_tree_model.header)-1
#    lastChild = root.child(0, last_row_index)
    i = 0
    selection = QtGui.QItemSelection()
    while root.child(i, 0).isValid():
      index = root.child(i, 0)
      item = self.node_tree_model.itemFromIndex(index)
      if not item is None and not self._is_in_ignore_list(item.name):
        selection.append(QtGui.QItemSelectionRange(index, root.child(i, last_row_index)))
      i = i + 1
#    selection = QtGui.QItemSelection(firstChild, lastChild)
    self.masterTab.nodeTreeView.selectionModel().select(selection, QtGui.QItemSelectionModel.ClearAndSelect)

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
    
  def on_copy_node_clicked(self):
    result = ''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    for node in selectedNodes:
      try:
        result = ' '.join([result, node.name])
      except Exception:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_topic_clicked(self):
    result = ''
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    for topic in selectedTopics:
      try:
        result = ' '.join([result, topic.name, topic.type])
      except Exception:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_service_clicked(self):
    result = ''
    selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
    for service in selectedServices:
      try:
        result = ' '.join([result, service.name, service.type])
      except Exception:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_parameter_clicked(self):
    result = ''
    selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
    for (key, value) in selectedParameter:
      try:
        result = ' '.join([result, key, str(value)])
      except Exception:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Filter handling                               %%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class TopicsSortFilterProxyModel(QtGui.QSortFilterProxyModel):
  def filterAcceptsRow(self, sourceRow, sourceParent):
    '''
    Perform filtering on columns 0 and 3 (Name, Type)
    '''
    index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
    index3 = self.sourceModel().index(sourceRow, 3, sourceParent)
  
    regex = self.filterRegExp()
    return (regex.indexIn(self.sourceModel().data(index0)) != -1
            or regex.indexIn(self.sourceModel().data(index3)) != -1)

class ServicesSortFilterProxyModel(QtGui.QSortFilterProxyModel):
  def filterAcceptsRow(self, sourceRow, sourceParent):
    '''
    Perform filtering on columns 0 and 1 (Name, Type)
    '''
    index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
    index1 = self.sourceModel().index(sourceRow, 1, sourceParent)
  
    regex = self.filterRegExp()
    return (regex.indexIn(self.sourceModel().data(index0)) != -1
            or regex.indexIn(self.sourceModel().data(index1)) != -1)

class ParameterSortFilterProxyModel(QtGui.QSortFilterProxyModel):
  def filterAcceptsRow(self, sourceRow, sourceParent):
    '''
    Perform filtering on columns 0 and 1 (Name, Value)
    '''
    index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
    index1 = self.sourceModel().index(sourceRow, 1, sourceParent)
  
    regex = self.filterRegExp()
    return (regex.indexIn(self.sourceModel().data(index0)) != -1
            or regex.indexIn(self.sourceModel().data(index1)) != -1)

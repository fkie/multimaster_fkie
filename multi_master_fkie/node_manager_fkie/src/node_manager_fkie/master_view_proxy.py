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

from PySide import QtGui
from PySide import QtCore
from PySide import QtUiTools

import os
import xmlrpclib
import socket
import threading
import time

import rospy
import roslib

import node_manager_fkie as nm
from html_delegate import HTMLDelegate
from topic_list_model import TopicModel, TopicItem
from node_tree_model import NodeTreeModel, ExtendedNodeInfo
from service_list_model import ServiceModel, ServiceItem
from parameter_list_model import ParameterModel, ParameterItem
from default_cfg_handler import DefaultConfigHandler
from launch_config import LaunchConfig, LaunchConfigException



class MasterViewProxy(QtGui.QWidget):
  '''
  This class stores the informations about a ROS master and shows it on request.
  '''

  updateHostRequest = QtCore.Signal(str)
  host_description_updated = QtCore.Signal(str, str)
  '''@ivar: the signal is emitted on description changes and contains the host 
  and description a parameter.'''
  
  def __init__(self, masteruri, parent=None):
    '''
    Creates a new master.
    @param masteruri: the URI of the ROS master
    @type masteruri: C{str}
    '''
    QtGui.QWidget.__init__(self, parent)
    self.masteruri = masteruri
    self.__master_state = None
    self.__master_info = None
    self.__launchfiles = dict()
    self.__default_configs = dict() # [(service name, service uri)] = nodes
    self.rosconfigs = dict() # [launch file path] = LaunchConfig()
    self.__in_question = []
    self._stop_ignores = ['/rosout', rospy.get_name(), '/master_discovery', '/master_sync', '/default_cfg']
    ''' @todo: detect the names of master_discovery and master_sync ndoes'''
    
    self.progressDialog = QtGui.QProgressDialog(self)
    self.progressDialog.setWindowModality(QtCore.Qt.WindowModal)
    
    self.default_cfg_handler = DefaultConfigHandler()
    self.default_cfg_handler.node_list_signal.connect(self.on_default_cfg_nodes_retrieved)
    self.default_cfg_handler.description_signal.connect(self.on_default_cfg_descr_retrieved)
    self.default_cfg_handler.err_signal.connect(self.on_default_cfg_err)
    
    loader = QtUiTools.QUiLoader()
    self.masterTab = loader.load(":/forms/MasterTab.ui")
    tabLayout = QtGui.QVBoxLayout(self)
    tabLayout.setContentsMargins(0, 0, 0, 0)
    tabLayout.addWidget(self.masterTab)
    
    # setup the node view
    self.node_tree_model = NodeTreeModel(self.masteruri)
    self.node_tree_model.hostInserted.connect(self.on_host_inserted)
    self.masterTab.nodeTreeView.setModel(self.node_tree_model)
    for i, (name, width) in enumerate(NodeTreeModel.header):
      self.masterTab.nodeTreeView.setColumnWidth(i, width)
    self.nodeNameDelegate = HTMLDelegate()
    self.masterTab.nodeTreeView.setItemDelegateForColumn(0, self.nodeNameDelegate)
    self.masterTab.nodeTreeView.collapsed.connect(self.on_node_collapsed)
    self.masterTab.nodeTreeView.expanded.connect(self.on_node_expanded)
    self.masterTab.nodeTreeView.selectionModel().selectionChanged.connect(self.on_node_selection_changed)
    self.masterTab.nodeTreeView.activated.connect(self.on_node_activated)
#    self.masterTab.nodeTreeView.setAcceptDrops(True)
#    self.masterTab.nodeTreeWidget.setSortingEnabled(True)

    # setup the topic view
    self.topic_model = TopicModel()
    self.topic_proxyModel =  QtGui.QSortFilterProxyModel(self)
    self.topic_proxyModel.setSourceModel(self.topic_model)
    self.masterTab.topicsView.setModel(self.topic_proxyModel)
#    self.masterTab.topicsView.setModel(self.topic_model)
    for i, (name, width) in enumerate(TopicModel.header):
      self.masterTab.topicsView.setColumnWidth(i, width)
    self.topicNameDelegate = HTMLDelegate()
    self.masterTab.topicsView.setItemDelegateForColumn(0, self.topicNameDelegate)
    self.masterTab.topicsView.selectionModel().selectionChanged.connect(self.on_topic_selection_changed)
    self.masterTab.topicsView.activated.connect(self.on_topic_activated)
    self.masterTab.topicsView.setSortingEnabled(True)
    self.topic_proxyModel.filterAcceptsRow = self._filterTopicsAcceptsRow

    # setup the service view
    self.service_model = ServiceModel()
    self.service_proxyModel =  QtGui.QSortFilterProxyModel(self)
    self.service_proxyModel.setSourceModel(self.service_model)
    self.masterTab.servicesView.setModel(self.service_proxyModel)
#    self.masterTab.servicesView.setModel(self.service_model)
    for i, (name, width) in enumerate(ServiceModel.header):
      self.masterTab.servicesView.setColumnWidth(i, width)
    self.serviceNameDelegate = HTMLDelegate()
    self.serviceTypeDelegate = HTMLDelegate()
    self.masterTab.servicesView.setItemDelegateForColumn(0, self.serviceNameDelegate)
    self.masterTab.servicesView.setItemDelegateForColumn(1, self.serviceTypeDelegate)
    self.masterTab.servicesView.selectionModel().selectionChanged.connect(self.on_service_selection_changed)
    self.masterTab.servicesView.activated.connect(self.on_service_activated)
    self.masterTab.servicesView.setSortingEnabled(True)
    self.service_proxyModel.filterAcceptsRow = self._filterServiceAcceptsRow
    
    # setup the parameter view
    self.parameter_model = ParameterModel()
    self.parameter_proxyModel =  QtGui.QSortFilterProxyModel(self)
    self.parameter_proxyModel.setSourceModel(self.parameter_model)
    self.masterTab.parameterView.setModel(self.parameter_proxyModel)
    for i, (name, width) in enumerate(ParameterModel.header):
      self.masterTab.parameterView.setColumnWidth(i, width)
    self.parameterNameDelegate = HTMLDelegate()
    self.masterTab.parameterView.setItemDelegateForColumn(0, self.parameterNameDelegate)
    self.masterTab.parameterView.selectionModel().selectionChanged.connect(self.on_parameter_selection_changed)
    self.masterTab.parameterView.setSortingEnabled(True)
    self.parameter_proxyModel.filterAcceptsRow = self._filterParameterAcceptsRow
#    self.masterTab.parameterView.activated.connect(self.on_service_activated)

    # connect the buttons
    self.masterTab.startButton.clicked.connect(self.on_start_clicked)
    self.masterTab.stopButton.clicked.connect(self.on_stop_clicked)
    self.masterTab.ioButton.clicked.connect(self.on_io_clicked)
    self.masterTab.logButton.clicked.connect(self.on_log_clicked)
    self.masterTab.logDeleteButton.clicked.connect(self.on_log_delete_clicked)
    self.masterTab.dynamicConfigButton.clicked.connect(self.on_dynamic_config_clicked)
    self.masterTab.editConfigButton.clicked.connect(self.on_edit_config_clicked)
    self.masterTab.closeNodeViewButton.clicked.connect(self.on_close_clicked)

    self.masterTab.echoTopicButton.clicked.connect(self.on_topic_echo_clicked)
    self.masterTab.callServiceButton.clicked.connect(self.on_service_call_clicked)
    self.masterTab.topicFilterInput.textChanged.connect(self.on_topic_filter_changed)
    self.masterTab.serviceFilterInput.textChanged.connect(self.on_service_filter_changed)
    self.masterTab.parameterFilterInput.textChanged.connect(self.on_parameter_filter_changed)
    self.masterTab.getParameterButton.clicked.connect(self.on_get_parameter_clicked)
    self.masterTab.deleteParameterButton.clicked.connect(self.on_delete_parameter_clicked)

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
  

  @property
  def master_state(self):
    return self.__master_state

  @master_state.setter
  def master_state(self, master_state):
    self.__master_state = master_state
    nm.nameres().add(name=master_state.name, masteruri=master_state.uri, host=nm.nameres().getHostname(master_state.uri))

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
    if (master_info.masteruri == self.masteruri):
      # store process pid's of remote nodes
      nodepids = []
      if not self.__master_info is None:
        nodepids = [(n.name, n.pid, n.uri) for nodename, n in self.__master_info.nodes.items() if not n.isLocal]
      self.__master_info = master_info
      # restore process pid's of remote nodes
      if not self.__master_info is None:
        for nodename, pid, uri in nodepids:
          node = self.__master_info.getNode(nodename)
          if not node is None:
            node.pid = pid
      # request master info updates for new remote nodes
      nodepids2 = [(n.name, n.pid, n.uri) for nodename, n in self.__master_info.nodes.items() if not n.isLocal]
      node2update = list(set(nodepids2)-set(nodepids))
      hosts2update = list(set([nm.nameres().getHostname(uri) for nodename, pid, uri in node2update]))
      for host in hosts2update:
        self.updateHostRequest.emit(host)
    else:
      if not (self.__master_info is None or master_info is None):
        for nodename, node in master_info.nodes.items():
          if node.isLocal:
            n = self.__master_info.getNode(nodename)
            if not n is None:
              n.pid = node.pid
    self.updateRunningNodesInModel(self.__master_info)
    self.updateTopicsListModel(self.__master_info)
    self.updateServiceListModel(self.__master_info)
    self.updateDefaultConfigs(self.__master_info)
  
  def show_ros_names(self, value):
    '''
    @param value: show the as ROS names or as their description.
    @type value: C{bool}
    '''
    self.node_tree_model.show_ros_names(value)
  
  def updateRunningNodesInModel(self, master_info):
    '''
    Creates the dictionary with ExtendedNodeInfo objects and updates the nodes view.
    @param master_info: the mater information object
    @type master_info: L{master_discovery_fkie.msg.MasterInfo}
    '''
    if not master_info is None:
      nodes_extended = dict()
      for n in master_info.node_names:
        node = master_info.getNode(n)
        if not node is None:
          nodes_extended[node.name] = ExtendedNodeInfo(node.name, str(master_info.mastername), str(master_info.masteruri), node.uri, node.pid, node.publishedTopics, node.subscribedTopics, node.services)
        else:
          nodes_extended[n] = ExtendedNodeInfo(n, str(master_info.mastername), str(master_info.masteruri))
      self.node_tree_model.updateModelData(nodes_extended, nm.nameres().getHostname(self.masteruri))
      self.updateButtons()

  def updateButtons(self):
    '''
    Updates the enable state of the buttons depending of the selection and 
    running state of the selected node.
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    has_running = False
    has_stopped = False
    for node in selectedNodes:
      if not node.uri is None:
        has_running = True
      else:
        has_stopped = True 
    self.masterTab.startButton.setEnabled(has_stopped)
    self.masterTab.stopButton.setEnabled(has_running)
    self.masterTab.ioButton.setEnabled(has_running or has_stopped)
    self.masterTab.logButton.setEnabled(has_running or has_stopped)
    self.masterTab.logDeleteButton.setEnabled(has_running or has_stopped)
    # test for available dynamic reconfigure services
    if not self.master_info is None:
      dyncfgServices = [s[:-len('/set_parameters')] for s in self.master_info.services.keys() if (s.endswith('/set_parameters'))]
      dyncfgNodes = [s for n in selectedNodes for s in dyncfgServices if s.startswith((n.name))]
      self.masterTab.dynamicConfigButton.setEnabled(len(dyncfgNodes))
    # the configuration is only available, if only one node is selected
    cfg_enable = False
    if len(selectedNodes) == 1:
      cfg_enable = len(selectedNodes[0].cfgs) > 0
    self.masterTab.editConfigButton.setEnabled(cfg_enable)

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
    return launchfiles.has_key(path)

  @property
  def launchfiles(self):
    '''
    Returns the dictionary with loaded launch files on this host
    @rtype: C{dict(str(file) : L{LaunchConfig}, ...)}
    '''
    return self.__launchfiles

  @launchfiles.setter
  def launchfiles(self, launchfile):
    '''
    Loads the launch file. If this file is already loaded, it will be reloaded.
    After successful load the node view will be updated.
    @param launchfile: the launch file path
    @type launchfile: C{str}
    '''
#    QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
    if self.__launchfiles.has_key(launchfile):
      # close the configuration
      self.removeConfigFromModel(launchfile, self.__launchfiles[launchfile].Roscfg)
    #load launch config
    try:
      # test for requerid args
      launchConfig = LaunchConfig(launchfile, masteruri=self.masteruri)
      loaded, req_args = launchConfig.load([])
      ok = False
      if not loaded:
        arg_history = launchConfig.getArgHistory()
        slots = []
        types = []
        for arg in launchConfig.argvToDict(req_args).keys():
          slots.append(arg)
          types.append('string')
        from parameter_dialog import ParameterDialog
        inputDia = ParameterDialog(slots, types, arg_history)
        inputDia.setWindowTitle(''.join(['Enter the argv for ', launchfile]))
        if inputDia.exec_():
          params = inputDia.getKeywords()
          argv = []
          for p,v in params.items():
            launchConfig.addToArgHistory(p, v)
            argv.append(''.join([p, ':=', v]))
          loaded, req_args = launchConfig.load(argv)

      if loaded:
        self.__launchfiles[launchfile] = launchConfig
        #connect the signal, which is emitted on changes on configuration file 
        launchConfig.file_changed.connect(self.on_configfile_changed)
        for name, machine in launchConfig.Roscfg.machines.items():
          if machine.name:
            nm.nameres().add(name=machine.name, host=machine.address)
        self.appendConfigToModel(launchfile, launchConfig.Roscfg)
        self.masterTab.tabWidget.setCurrentIndex(0)
        
        #set the description of the nodes
        try:
          sensor_descr = launchConfig.getSensorDesrc()
          for robot, nodes in sensor_descr.items():
            if not robot:
              robot = nm.nameres().getHost(masteruri=self.masteruri)
            self.node_tree_model.updateNodesDescr(robot, nodes)
          robot_descr = launchConfig.getRobotDescr()
          for name, d in robot_descr.items():
            if not name:
              name = nm.nameres().getHost(masteruri=self.masteruri)
            self.node_tree_model.updateHostDescription(name, d['robot_type'], d['robot_name'], d['robot_descr'])
        except:
          import traceback
          print traceback.print_exc()
#        host = nm.nameres().getName(masteruri=self.masteruri)
#        tooltip = self.node_tree_model.updateNodesDescription(host, items)
#        self.host_description_updated.emit(host, tooltip)

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
        self.masterTab.closeNodeViewButton.setEnabled(len(self.__launchfiles) > 0)
        self.masterTab.closeNodeViewButton.setToolTip("Closes open configuration files")
    except Exception, e:
      import os
      err_msg = ''.join([os.path.basename(launchfile),' loading failed!\n\n', str(e)])
      rospy.logwarn("Loading launch file: %s", err_msg)
      QtGui.QMessageBox.warning(None, self.tr("Loading launch file"),
                                err_msg,
                                QtGui.QMessageBox.Ok)

  def appendConfigToModel(self, launchfile, rosconfig):
    '''
    Update the node view 
    @param launchfile: the launch file path
    @type launchfile: C{str}
    @param rosconfig: the configuration
    @type rosconfig: L{LaunchConfig}
    '''
    nodes_extended = dict()
    for n in rosconfig.nodes:
      masteruri = self.masteruri
      mastername = nm.nameres().getName(masteruri=self.masteruri)
      if n.machine_name:
        mastername = n.machine_name
        masteruri = nm.nameres().getUri(host=rosconfig.machines[n.machine_name].address)
      node = str(''.join([n.namespace, n.name]))
      nodes_extended[node] = ExtendedNodeInfo(node, mastername, masteruri, cfgs=[launchfile])
    self.node_tree_model.appendConfigNodes(nodes_extended, nm.nameres().getHostname(self.masteruri))
    self.updateButtons()

  def removeConfigFromModel(self, launchfile, rosconfig):
    '''
    Update the node view after removed configuration.
    @param launchfile: the launch file path
    @type launchfile: C{str}
    @param rosconfig: the configuration
    @type rosconfig: L{LaunchConfig}
    '''
    nodes_extended = dict()
    for n in rosconfig.nodes:
      mastername = self.master_info.mastername
      masteruri = self.master_info.masteruri
      if n.machine_name:
        mastername = n.machine_name
        masteruri = nm.nameres().getUri(host=rosconfig.machines[n.machine_name].address)
      node = str(''.join([n.namespace, n.name]))
      nodes_extended[node] = ExtendedNodeInfo(node, mastername, masteruri, cfgs=[launchfile])
    self.node_tree_model.removeConfigNodes(nodes_extended)
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
        default_cfgs.append((name, srv.uri))
    # remove the node contained in default configuration form the view
    removed = list(set(self.__default_configs.keys()) - set(default_cfgs))
    if removed:
      for r in removed:
        host = nm.nameres().getHostname(r[1])
        nodes_extended = dict()
        for node in self.__default_configs[r]:
          nodes_extended[node] = ExtendedNodeInfo(node, nm.nameres().getName(host=host), master_info.masteruri, default_cfgs=[r])
        self.node_tree_model.removeDefaultConfigNodes(nodes_extended)
        del self.__default_configs[r]
        tooltip = self.node_tree_model.updateNodesDescription(host, [])
        self.host_description_updated.emit(host, tooltip)
    if len(default_cfgs) == 0:
      host = nm.nameres().getHost(masteruri=master_info.masteruri)
      tooltip = self.node_tree_model.updateHostDescription(host, '', '', '')
      self.host_description_updated.emit(host, tooltip)
    # request the nodes of new default configurations
    added = list(set(default_cfgs) - set(self.__default_configs.keys()))
    for (name, uri) in added:
      self.default_cfg_handler.requestNodeList(uri, name)
      #request the description
      ns = roslib.names.namespace(name)
      descr_service = self.__master_info.getService(roslib.names.ns_join(ns, 'description'))
      if not descr_service is None:
        self.default_cfg_handler.requestDescriptionList(descr_service.uri, descr_service.name)
    self.updateButtons()
    
  def remove_all_def_configs(self):
    for r in list(set(self.__default_configs.keys())):
      nodes_extended = dict()
      for node in self.__default_configs[r]:
        nodes_extended[node] = ExtendedNodeInfo(node, nm.nameres().getName(host=nm.nameres().getHostname(r[1])), self.__master_info.masteruri, default_cfgs=[r])
      self.node_tree_model.removeDefaultConfigNodes(nodes_extended)
      del self.__default_configs[r]
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
    key = (config_name, service_uri)
    if self.__default_configs.has_key(key):
      nodes_extended = dict()
      for node in self.__default_configs[key]:
        nodes_extended[node] = ExtendedNodeInfo(node, nm.nameres().getName(host=nm.nameres().getHostname(service_uri)), masteruri, default_cfgs=[key])
      self.node_tree_model.removeDefaultConfigNodes(nodes_extended)
    # add the new config
    nodes_extended = dict()
    for node in nodes:
      nodes_extended[node] = ExtendedNodeInfo(node, nm.nameres().getName(host=nm.nameres().getHostname(service_uri)), masteruri, default_cfgs=[key])
    self.node_tree_model.appendDefaultConfigNodes(nodes_extended, nm.nameres().getHostname(self.masteruri))
    self.__default_configs[key] = nodes

  def on_default_cfg_descr_retrieved(self, service_uri, config_name, items):
    '''
    Handles the description list from default configuration service.
    Emits a Qt signal L{host_description_updated} to notify about a new host 
    description.
    @param service_uri: the URI of the service provided the default configuration
    @type service_uri: C{str}
    @param config_name: the name of default configuration service
    @type config_name: C{str}
    @param items: list with descriptions
    @type items: C{[L{default_cfg_fkie.Description}]}
    '''
    host = nm.nameres().getName(host=nm.nameres().getHostname(service_uri))
    tooltip = self.node_tree_model.updateNodesDescription(host, items)
    self.host_description_updated.emit(host, tooltip)

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
  
  def on_configfile_changed(self, changed, config):
    '''
    Signal hander to handle the changes of a loaded configuration file
    @param changed: the changed file
    @type changed: C{str}
    @param config: the main configuration file to load
    @type config: C{str}
    '''
    name = self.masteruri
    if not self.__master_info is None:
      name = self.__master_info.mastername
    if not config in self.__in_question:
#      print "append", config
      self.__in_question.append(config)
      ret = QtGui.QMessageBox.question(self, ''.join([name, ' - configuration update']),
                                       ' '.join(['The configuration file', changed, 'was changed!\n\nReload the configuration', os.path.basename(config), 'for', name,'?']),
                                       QtGui.QMessageBox.Ok | QtGui.QMessageBox.No, QtGui.QMessageBox.Ok)
      self.__in_question.remove(config)
#      print "remove", config
      if ret == QtGui.QMessageBox.Ok:
        self.launchfiles = config


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
    if self.masterTab.startButton.isEnabled():
      self.on_start_clicked()
    elif self.masterTab.ioButton.isEnabled():
      self.on_io_clicked()
    else:
      self.on_log_clicked()

  def on_topic_activated(self, index):
    '''
    @param index: The index of the activated topic
    @type index: L{PySide.QtCore.QModelIndex}
    '''
    self.on_topic_echo_clicked()

  def on_service_activated(self, index):
    '''
    @param index: The index of the activated service
    @type index: L{PySide.QtCore.QModelIndex}
    '''
    self.on_service_call_clicked()

  def on_host_inserted(self, index):
    self.masterTab.nodeTreeView.expandAll()

  def on_node_collapsed(self, index):
    if not index.parent ().isValid():
      self.masterTab.nodeTreeView.selectionModel().clear()
    
  def on_node_expanded(self, index):
    pass

  def on_node_selection_changed(self, selected, deselected):
    self.updateButtons()

  def on_topic_selection_changed(self, selected, deselected):
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    self.masterTab.echoTopicButton.setEnabled(len(selectedTopics) > 0)

  def on_service_selection_changed(self, selected, deselected):
    selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
    self.masterTab.callServiceButton.setEnabled(len(selectedServices) > 0)

  def on_parameter_selection_changed(self, selected, deselected):
    selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
    self.masterTab.deleteParameterButton.setEnabled(len(selectedParameter) > 0)

  def nodesFromIndexes(self, indexes):
    result = []
    for index in indexes:
      if index.column() == 0 and index.parent().isValid():
        item = self.node_tree_model.itemFromIndex(index)
        if not item is None:
          result.append(item.node)
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
      if not item is None and isinstance(item, ParameterItem):
        result.append((item.key, item.value))
    return result


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Handling of the button activities                %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_start_clicked(self):
    '''
    Starts the selected nodes. If for a node more then one configuration is 
    available, the selection dialog will be show.
    '''
    cursor = self.cursor()
    self.masterTab.startButton.setEnabled(False)
    self.setCursor(QtCore.Qt.WaitCursor)
    key_mod = QtGui.QApplication.keyboardModifiers()
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    self.progressDialog.setWindowTitle('Start')
    self.progressDialog.setMaximum(len(selectedNodes)+1)
    self.progressDialog.show()
    QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
    i = 0
    for node in selectedNodes:
      i += 1
      self.progressDialog.setLabelText(node.name)
      self.progressDialog.setValue(i)
      if self.progressDialog.wasCanceled():
        break
      if node.uri is None:
        config = None
        choices = {} # dict with available configurations {displayed name: LaunchConfig() or str[service name of default configuration]} 
        choices_cfg = self._getCfgChoises(node)
        choices_def_cfg = self._getDefaultCfgChoises(node)
        if not choices_cfg or (key_mod & QtCore.Qt.ControlModifier and not key_mod & QtCore.Qt.ShiftModifier):
          choices = choices_def_cfg
        elif not choices_def_cfg or (key_mod & QtCore.Qt.ShiftModifier and not key_mod & QtCore.Qt.ControlModifier):
          choices = choices_cfg
        if not choices:
          choices = dict(choices_cfg.items() + choices_def_cfg.items())
        config = self._getUserCfgChoice(choices, node.name)

        # start the node using launch configuration
        if isinstance(config, LaunchConfig):
          try:
            nm.starter().runNode(node.name, self.launchfiles[node.cfgs[0]])
          except (Exception, nm.StartException), e:
            rospy.logwarn("Error while start '%s': %s", node.name, str(e))
            QtGui.QMessageBox.warning(None, 'Error while start %s'%node.name,
                                      str(e),
                                      QtGui.QMessageBox.Ok)
        elif isinstance(config, str):
          # start with default configuration
          from default_cfg_fkie.srv import Task
          try:
            nm.starter().callService(self.master_info.getService(config).uri, config, Task, node.name)
          except (Exception, nm.StartException), e:
            rospy.logwarn("Error while call a service of node '%s': %s", node.name, str(e))
            QtGui.QMessageBox.warning(None, 'Error while call a service of node %s'%node.name,
                                      str(e),
                                      QtGui.QMessageBox.Ok)
    self.progressDialog.setValue(self.progressDialog.maximum())
    self.setCursor(cursor)

  def _getDefaultCfgChoises(self, node):
    result = {}
    for (name, uri) in node.default_cfgs:
      service = name[0:-11] # remove the last '/list_nodes'
      result[' '.join(['[default]', service])] = str(''.join([service, '/run']))
    return result

  def _getCfgChoises(self, node):
    result = {}
    for c in node.cfgs:
      launch = self.launchfiles[c]
      result[''.join([str(launch.LaunchName), ' [', str(launch.PackageName), ']'])] = self.launchfiles[c]
    return result

  def _getUserCfgChoice(self, choices, nodename):
    value = None
    # Open selection
    if len(choices) == 1:
      value = choices[choices.keys()[0]]
    elif len(choices) > 0:
      item, ok = QtGui.QInputDialog.getItem(self, "Configuration selection",
                                        "Select a configuration for '%s'"%nodename,
                                        choices.keys(), 0, False)
      if ok:
        value = choices[item]
    return value

  
  def on_stop_clicked(self):
    '''
    Stops the selected and running nodes. If the node can't be stopped using his
    RPC interface, it will be unregistered from the ROS master using the masters
    RPC interface.
    '''
    cursor = self.cursor()
    self.setCursor(QtCore.Qt.WaitCursor)
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    self.stop_nodes(selectedNodes)
    self.setCursor(cursor)
  
  def stop_nodes(self, nodes):
    '''
    Internal method to stop a list with nodes
    @param nodes: the list with nodes to stop
    @type nodes: L{[master_discovery_fkie.NodeInfo, ...]}
    '''
    self.progressDialog.setWindowTitle('Stop')
    self.progressDialog.setMaximum(len(nodes)+1)
    self.progressDialog.show()
    QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
    i = 0
    for node in nodes:
      self.progressDialog.setLabelText(node.name)
      self.progressDialog.setValue(i)
      if self.progressDialog.wasCanceled():
        break
      if not node is None and not node.uri is None and (not (node.name in self._stop_ignores) or len(nodes) == 1):
        key_mod = QtGui.QApplication.keyboardModifiers()
        if key_mod & QtCore.Qt.ControlModifier:
          pid = node.pid
          if pid is None:
            # try to get the process id of the node
            try:
              rpc_node = xmlrpclib.ServerProxy(node.uri)
              code, msg, pid = rpc_node.getPid(rospy.get_name())
            except:
              self.masterTab.stopButton.setEnabled(False)
          # kill the node
          if not pid is None:
            try:
              nm.starter().kill(self.getHostFromNode(node), pid)
            except Exception, e:
              rospy.logwarn("Error while kill the node %s: %s", str(node.name), str(e))
              QtGui.QMessageBox.warning(None, 'Error while kill the node %s'%node.name,
                                        str(e),
                                        QtGui.QMessageBox.Ok)
  
        else:
          try:
            p = xmlrpclib.ServerProxy(node.uri)
            p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', socket.gethostname()]))
          except Exception, e:
#            import traceback
#            formatted_lines = traceback.format_exc().splitlines()
            rospy.logwarn("Error while stop node '%s': %s", str(node.name), str(e))
            self.masterTab.stopButton.setEnabled(False)
        if key_mod & QtCore.Qt.ShiftModifier and not node is None:
          # unregister all entries of the node from ROS master
          master = xmlrpclib.ServerProxy(node.masteruri)
          master_multi = xmlrpclib.MultiCall(master)
          master_multi.deleteParam(node.name, node.name)
          for p in node.published:
            rospy.logdebug("unregister publisher '%s' [%s] from ROS master: %s", p, node.name, node.masteruri)
            master_multi.unregisterPublisher(node.name, p, node.uri)
          for s in node.subscribed:
            rospy.logdebug("unregister subscriber '%s' [%s] from ROS master: %s", p, node.name, node.masteruri)
            master_multi.unregisterSubscriber(node.name, s, node.uri)
          if not self.master_state is None:
            for s in node.services:
              rospy.logdebug("unregister service '%s' [%s] from ROS master: %s", p, node.name, node.masteruri)
              service = self.master_info.getService(s)
              if not (service is None):
                master_multi.unregisterService(node.name, s, service.uri)
          r = master_multi()
          for code, msg, _ in r:
            if code != 1:
              rospy.logdebug("unregistration failed: %s", msg)
      i += 1
    self.progressDialog.setValue(self.progressDialog.maximum())

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
    if len(node.cfgs) > 0:
      launch_config = self.launchfiles[node.cfgs[0]]
      item = launch_config.getNode(node.name)
      if not item is None and item.machine_name:
        return launch_config.Roscfg.machines[item.machine_name].address
    # return the host of the assigned ROS master
    return nm.nameres().getHostname(node.masteruri)
  
  def on_io_clicked(self):
    '''
    Shows IO of the selected nodes.
    '''
    cursor = self.cursor()
    self.setCursor(QtCore.Qt.WaitCursor)
    key_mod = QtGui.QApplication.keyboardModifiers()
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    for node in selectedNodes:
      try:
        if key_mod & QtCore.Qt.ControlModifier:
          # send kill to the associated screens
          nm.screen().killScreens(self.getHostFromNode(node), node.name, parent=self)
        else:
          if not nm.screen().openScreen(self.getHostFromNode(node), node.name, parent=self):
            self.masterTab.ioButton.setEnabled(False)
      except Exception, e:
        rospy.logwarn("Error while show IO for %s: %s", str(node), str(e))
        QtGui.QMessageBox.warning(None, 'Error while show IO %s'%node.name,
                                  str(e),
                                  QtGui.QMessageBox.Ok)
    self.setCursor(cursor)

  def on_log_clicked(self):
    '''
    Shows log files of the selected nodes.
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    try:
      for node in selectedNodes:
        if not nm.starter().openLog(node.name, self.getHostFromNode(node)):
          self.masterTab.logButton.setEnabled(False)
    except Exception, e:
      rospy.logwarn("Error while show log for '%s': %s", str(node), str(e))
      QtGui.QMessageBox.warning(None, 'Error while show Log of %s'%node.name,
                                str(e),
                                QtGui.QMessageBox.Ok)

  def on_log_delete_clicked(self):
    '''
    Deletes log files of the selected nodes.
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    try:
      for node in selectedNodes:
        nm.starter().deleteLog(node.name, self.getHostFromNode(node))
    except Exception, e:
      rospy.logwarn("Error while delete log for '%s': %s", str(node), str(e))
      QtGui.QMessageBox.warning(None, 'Error while delete Log of %s'%node.name,
                                str(e),
                                QtGui.QMessageBox.Ok)

  
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
            item = QtGui.QInputDialog.getItem(None, self.tr("Dynamic configuration selection"),
                                              'Select a node to configure',
                                              [i for i in nodes], 0, False)
            if item[1]:
              node = item[0]
          if not node is None:
            self.masterTab.dynamicConfigButton.setEnabled(False)
            import os, subprocess
            env = dict(os.environ)
            env["ROS_MASTER_URI"] = str(self.master_info.masteruri)
            subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'reconfigure_gui', node], env=env)
        except Exception, e:
          rospy.logwarn("Start dynamic reconfiguration for '%s' failed: %s", str(node), str(e))
          QtGui.QMessageBox.warning(None, self.tr("Start dynamic reconfiguration"),
                                    str(e),
                                    QtGui.QMessageBox.Ok)

  def on_edit_config_clicked(self):
    '''
    '''
    selectedNodes = self.nodesFromIndexes(self.masterTab.nodeTreeView.selectionModel().selectedIndexes())
    from xml_editor import XmlEditor
    for node in selectedNodes:
      config = self._getUserCfgChoice(self._getCfgChoises(node), node.name)
      # get the file, which include the node and the main configuration file
      node_cfg = config.getNode(node.name)
      files = [config.Filename]
      if not node_cfg.filename in files:
        files.append(node_cfg.filename)
      self.editor = XmlEditor(files, os.path.basename(node.name), self)
      self.editor.show()

  def on_close_clicked(self):
    '''
    Closes the open launch configurations. If more then one configuration is 
    open a selection dialog will be open.
    '''
    cfgs = {'all': None} if len(self.launchfiles) > 1 else {}
    for file, launchConfig in self.launchfiles.items():
      cfgs[''.join([launchConfig.LaunchName, ' [',str(launchConfig.PackageName),']'])] = file
    if len(cfgs) > 1:
      item = QtGui.QInputDialog.getItem(None, self.tr("Close configuration selection"),
                                        'Select a configuration to close',
                                        cfgs.keys(), 0, False)
      if item[1]:
        path = cfgs[item[0]]
        if self.__launchfiles.has_key(path):
          # close the configuration
          self.removeConfigFromModel(path, self.__launchfiles[path].Roscfg)
          del self.__launchfiles[path]
        elif item[0] == 'all':
          # close all configurations
          for path, cfg in self.__launchfiles.items():
            self.removeConfigFromModel(path, cfg.Roscfg)
          self.__launchfiles.clear()
    elif len(self.__launchfiles) == 1:
      # close all configurations
      for path, cfg in self.__launchfiles.items():
        self.removeConfigFromModel(path, cfg.Roscfg)
      self.__launchfiles.clear()

  def on_topic_echo_clicked(self):
    '''
    Shows the output of the topic in a terminal.
    '''
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    for topic in selectedTopics:
      try:
        import os, shlex, subprocess
        env = dict(os.environ)
        env["ROS_MASTER_URI"] = str(self.masteruri)
        cmd = nm.terminal_cmd(['rostopic', 'echo', topic.name], topic.name)
        subprocess.Popen(shlex.split(cmd), env=env)
      except Exception, e:
        rospy.logwarn("Echo topic '%s' failed: %s", str(topic.name), str(e))
        QtGui.QMessageBox.warning(None, 'self.tr("Error")',
                                  str(''.join(['Echo of topic ', topic.name, ' failed!\n', str(e)])),
                                  QtGui.QMessageBox.Ok)


  def on_service_call_clicked(self):
    '''
    calls a service.
    '''
    from parameter_dialog import ParameterDialog
    selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
    for service in selectedServices:
      try:
        slots = service.get_service_class(True)._request_class.__slots__
        types = service.get_service_class()._request_class._slot_types
        if not slots:
          req, resp = nm.starter().callService(service.uri, service.name, service.get_service_class())
          showDia = ParameterDialog([], [], buttons=QtGui.QDialogButtonBox.Ok, parent=self)
          showDia.setWindowTitle(''.join(['Response of ', service.name]))
          showDia.setText(str(resp))
          showDia.show()
#          QtGui.QMessageBox.information(self, ''.join(['Response of ', service.name]),
#                                    str(resp),
#                                    QtGui.QMessageBox.Ok)
        else:
          inputDia = ParameterDialog(slots, types)
          inputDia.setWindowTitle(''.join(['Enter the parameter for ', service.name]))
          if inputDia.exec_():
            params = inputDia.getKeywords()
            req, resp = nm.starter().callService(service.uri, service.name, service.get_service_class(), **params)
            showDia = ParameterDialog([], [], buttons=QtGui.QDialogButtonBox.Ok, parent=self)
            showDia.setWindowTitle(''.join(['Request / Response of ', service.name]))
            showDia.setText('\n'.join([str(req), '---', str(resp)]))
            showDia.show()
#            QtGui.QMessageBox.information(self, ''.join(['Request / Response of ', service.name]),
#                                      str('\n'.join([str(req), '---', str(resp)])),
#                                      QtGui.QMessageBox.Ok)
#          QtGui.QMessageBox.information(self, ''.join(['Call of ', service.name]),
#                                    'Call of service with parameter is not yet supported!',
#                                    QtGui.QMessageBox.Ok)
      except nm.StartException, e:
        rospy.logwarn("Error while call service '%s': %s", str(service.name), str(e))
        QtGui.QMessageBox.warning(None, 'Error while call %s'%service.name,
                                  str(e),
                                  QtGui.QMessageBox.Ok)

  def on_topic_filter_changed(self, text):
    '''
    Filter the displayed topics
    '''
    self.topic_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.FixedString))

  def on_service_filter_changed(self, text):
    '''
    Filter the displayed services
    '''
    self.service_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.FixedString))

  def on_parameter_filter_changed(self, text):
    '''
    Filter the displayed parameter
    '''
    self.parameter_proxyModel.setFilterRegExp(QtCore.QRegExp(text, QtCore.Qt.CaseInsensitive, QtCore.QRegExp.FixedString))

  def on_get_parameter_clicked(self):
    '''
    Requests parameter list from the ROS parameter server.
    '''
    result = {}
    try:
      name = rospy.get_name()
      master = xmlrpclib.ServerProxy(self.masteruri)
      code, msg, paramters = master.getParamNames(name)
      for p in paramters:
        code, msg, value = master.getParam(name, p)
        if code == 1:
          result[p] = value
      self.parameter_model.updateModelData(result)
    except:
      import traceback
      rospy.logwarn("Error on retrieve parameter: %s", str(traceback.format_exc()))

  def on_delete_parameter_clicked(self):
    '''
    Deletes the parameter from the ROS parameter server. 
    '''
    selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
    try:
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
    else:
      self.on_get_parameter_clicked()


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
    firstChild = root.child(0, 0)
    lastChild = root.child(0, 2)
    i = 0
    selection = QtGui.QItemSelection()
    while root.child(i, 0).isValid():
      index = root.child(i, 0)
      item = self.node_tree_model.itemFromIndex(index)
      if not item is None and not self._is_in_ignore_list(item.node.name):
        selection.append(QtGui.QItemSelectionRange(index, root.child(i, 2)))
      i = i + 1
#    selection = QtGui.QItemSelection(firstChild, lastChild)
    self.masterTab.nodeTreeView.selectionModel().select(selection, QtGui.QItemSelectionModel.ClearAndSelect)

  def _is_in_ignore_list(self, name):
    ignore_list = ['rosout', 'master_discovery', 'master_sync', 'node_manager', 'default_cfg']
    for i in ignore_list:
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
      except Exception, e:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_topic_clicked(self):
    result = ''
    selectedTopics = self.topicsFromIndexes(self.masterTab.topicsView.selectionModel().selectedIndexes())
    for topic in selectedTopics:
      try:
        result = ' '.join([result, topic.name, topic.type])
      except Exception, e:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_service_clicked(self):
    result = ''
    selectedServices = self.servicesFromIndexes(self.masterTab.servicesView.selectionModel().selectedIndexes())
    for service in selectedServices:
      try:
        result = ' '.join([result, service.name, service.type])
      except Exception, e:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())

  def on_copy_parameter_clicked(self):
    result = ''
    selectedParameter = self.parameterFromIndexes(self.masterTab.parameterView.selectionModel().selectedIndexes())
    for (key, value) in selectedParameter:
      try:
        result = ' '.join([result, key, str(value)])
      except Exception, e:
        pass
    QtGui.QApplication.clipboard().setText(result.strip())


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Filter handling                               %%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _filterTopicsAcceptsRow(self, sourceRow, sourceParent):
    '''
    Perform filtering on columns 0 and 3 (Name, Type)
    '''
    index0 = self.topic_proxyModel.sourceModel().index(sourceRow, 0, sourceParent)
    index3 = self.topic_proxyModel.sourceModel().index(sourceRow, 3, sourceParent)

    regex = self.topic_proxyModel.filterRegExp()
    return (regex.indexIn(self.topic_proxyModel.sourceModel().data(index0)) != -1
            or regex.indexIn(self.topic_proxyModel.sourceModel().data(index3)) != -1)

  def _filterServiceAcceptsRow(self, sourceRow, sourceParent):
    index0 = self.service_proxyModel.sourceModel().index(sourceRow, 0, sourceParent)
    index1 = self.service_proxyModel.sourceModel().index(sourceRow, 1, sourceParent)

    regex = self.service_proxyModel.filterRegExp()
    return (regex.indexIn(self.service_proxyModel.sourceModel().data(index0)) != -1
            or regex.indexIn(self.service_proxyModel.sourceModel().data(index1)) != -1)

  def _filterParameterAcceptsRow(self, sourceRow, sourceParent):
    index0 = self.parameter_proxyModel.sourceModel().index(sourceRow, 0, sourceParent)
    index1 = self.parameter_proxyModel.sourceModel().index(sourceRow, 1, sourceParent)

    regex = self.parameter_proxyModel.filterRegExp()
    return (regex.indexIn(self.parameter_proxyModel.sourceModel().data(index0)) != -1
            or regex.indexIn(self.parameter_proxyModel.sourceModel().data(index1)) != -1)


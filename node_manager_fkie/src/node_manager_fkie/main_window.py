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

import os
import time
import uuid
import xmlrpclib
import threading
import getpass

from datetime import datetime

from python_qt_binding import QtGui
from python_qt_binding import QtCore
#from python_qt_binding import QtUiTools
from python_qt_binding import loadUi

import roslib; roslib.load_manifest('node_manager_fkie')
import rospy

import gui_resources
from .discovery_listener import MasterListService, MasterStateTopic, MasterStatisticTopic, OwnMasterMonitoring
from .update_handler import UpdateHandler
from .launch_list_model import LaunchListModel 
from .master_view_proxy import MasterViewProxy
from .launch_config import LaunchConfig, LaunchConfigException
from .capability_table import CapabilityTable
from .xml_editor import XmlEditor
from .detailed_msg_box import WarningMessageBox
from .network_discovery_dialog import NetworkDiscoveryDialog
from .parameter_dialog import ParameterDialog
from .progress_queue import ProgressQueue, ProgressThread
from .screen_handler import ScreenHandler
from .sync_dialog import SyncDialog
from .common import masteruri_from_ros, package_name
from .select_dialog import SelectDialog
from .master_list_model import MasterModel, MasterSyncItem

import node_manager_fkie as nm

from multimaster_msgs_fkie.msg import LinkState, LinkStatesStamped, MasterState#, ROSMaster, SyncMasterInfo, SyncTopicInfo
from master_discovery_fkie.common import resolve_url
#from master_discovery_fkie.srv import DiscoverMasters, GetSyncInfo


class MainWindow(QtGui.QMainWindow):
  '''
  The class to create the main window of the application.
  '''
  DELAYED_NEXT_REQ_ON_ERR = 5.0

  def __init__(self, files=[], restricted_to_one_master=False, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QMainWindow.__init__(self)
    restricted_to_one_master = False
    self._finished = False
    self._history_selected_robot = ''
    self.__icons = {'default_pc' : (QtGui.QIcon(''.join([':/icons/crystal_clear_miscellaneous.png'])), ':/icons/crystal_clear_miscellaneous.png')} # (masnter name : (QIcon, path))
    self.__current_icon = None
    self.__current_master_label_name = None
    self.__current_path = os.path.expanduser('~')
    self._changed_files = dict()
    self._changed_files_param = dict()
    #self.setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
    #load the UI formular for the main window
#    loader = QtUiTools.QUiLoader()
    self.setObjectName('MainWindow')
    self.ui = mainWindow = QtGui.QMainWindow()
#    self.ui = mainWindow = loader.load(":/forms/MainWindow.ui")
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MainWindow.ui')
    #/home/tiderko/ros/src/multimaster_fkie/node_manager_fkie/src/node_manager_fkie/
    loadUi(ui_file, self.ui)
    self.ui.setObjectName('MainUI')
    self.ui.user_frame.setVisible(False)
    self._add_user_to_combo(getpass.getuser())
    self.ui.userComboBox.editTextChanged.connect(self.on_user_changed)
    self.ui.masterInfoFrame.setEnabled(False)
    self.ui.infoButton.clicked.connect(self.on_info_clicked)
    self.ui.refreshHostButton.clicked.connect(self.on_refresh_master_clicked)
    self.ui.runButton.clicked.connect(self.on_run_node_clicked)
    self.ui.rxconsoleButton.clicked.connect(self.on_show_rxconsole_clicked)
    self.ui.rxgraphButton.clicked.connect(self.on_show_rxgraph_clicked)
    self.ui.syncButton.released.connect(self.on_sync_dialog_released)
    # creates a default config menu
#    sync_menu = QtGui.QMenu(self)
#    self.syncDialogAct = QtGui.QAction("&Open sync dialog", self, shortcut=QtGui.QKeySequence(QtCore.Qt.CTRL + QtCore.Qt.Key_S), statusTip="Opens a sync dialog with additional sync options", triggered=self.on_sync_dialog_released)
#    sync_menu.addAction(self.syncDialogAct)
#    self.ui.syncButton.setMenu(sync_menu)

    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_prop_run.png")
    self.setWindowIcon(self.mIcon)
    self.setWindowTitle("Node Manager")
    self.setCentralWidget(mainWindow)

    # init the stack layout which contains the information about different ros master
    self.stackedLayout = QtGui.QStackedLayout()
    self.stackedLayout.setObjectName('stackedLayout')
    emptyWidget = QtGui.QWidget()
    emptyWidget.setObjectName('emptyWidget')
    self.stackedLayout.addWidget(emptyWidget)
    self.ui.tabWidget.currentChanged.connect(self.on_currentChanged_tab)
    self.ui.tabLayout = QtGui.QVBoxLayout(self.ui.tabPlace)
    self.ui.tabLayout.setObjectName("tabLayout")
    self.ui.tabLayout.setContentsMargins(0, 0, 0, 0)
    self.ui.tabLayout.addLayout(self.stackedLayout)

    # initialize the progress queue
    self._progress_queue = ProgressQueue(self.ui.progressFrame, self.ui.progressBar, self.ui.progressCancelButton)
    self._progress_queue_cfg = ProgressQueue(self.ui.progressFrame_cfg, self.ui.progressBar_cfg, self.ui.progressCancelButton_cfg)
    self._progress_queue_sync = ProgressQueue(self.ui.progressFrame_sync, self.ui.progressBar_sync, self.ui.progressCancelButton_sync)

    # initialize the view for the discovered ROS master
    self.master_model = MasterModel(self.getMasteruri())
    self.ui.masterTableView.setModel(self.master_model)
#    self.ui.masterTableView.setAlternatingRowColors(True)
    self.ui.masterTableView.clicked.connect(self.on_master_table_clicked)
    self.ui.masterTableView.pressed.connect(self.on_master_table_pressed)
    self.ui.masterTableView.activated.connect(self.on_master_table_activated)
    sm = self.ui.masterTableView.selectionModel()
    sm.currentRowChanged.connect(self.on_masterTableView_selection_changed)
    for i, (name, width) in enumerate(MasterModel.header):
      self.ui.masterTableView.setColumnWidth(i, width)
    self.ui.refreshAllButton.clicked.connect(self.on_all_master_refresh_clicked)
    self.ui.discoveryButton.clicked.connect(self.on_discover_network_clicked)
    self.ui.startRobotButton.clicked.connect(self.on_start_robot_clicked)

    # initialize the view for the launch files
    self.ui.xmlFileView.setModel(LaunchListModel())
    self.ui.xmlFileView.setAlternatingRowColors(True)
    self.ui.xmlFileView.activated.connect(self.on_launch_selection_activated)
    self.ui.xmlFileView.setDragDropMode(QtGui.QAbstractItemView.DragOnly)
    self.ui.xmlFileView.setDragEnabled(True)
    sm = self.ui.xmlFileView.selectionModel()
    sm.selectionChanged.connect(self.on_xmlFileView_selection_changed)
    self.ui.refreshXmlButton.clicked.connect(self.on_refresh_xml_clicked)
    self.ui.editXmlButton.clicked.connect(self.on_edit_xml_clicked)
    self.ui.newXmlButton.clicked.connect(self.on_new_xml_clicked)
    self.ui.openXmlButton.clicked.connect(self.on_open_xml_clicked)
    #self.ui.newXmlButton.setVisible(False)
    self.ui.transferButton.clicked.connect(self.on_transfer_file_clicked)
    self.ui.loadXmlButton.clicked.connect(self.on_load_xml_clicked)
    self.ui.loadXmlAsDefaultButton.clicked.connect(self.on_load_as_default)

    # stores the widget to a 
    self.masters = dict() # masteruri : MasterViewProxy
    self.currentMaster = None # MasterViewProxy

    nm.file_watcher().file_changed.connect(self.on_configfile_changed)
    nm.file_watcher_param().file_changed.connect(self.on_configparamfile_changed)
    self.__in_question = set()

    ############################################################################
    ############################################################################
    ############################################################################
    ############################################################################
    ############################################################################
    self.capabilitiesTable = CapabilityTable(self.ui.capabilities_tab)
    self.capabilitiesTable.setObjectName("capabilitiesTable")
    self.capabilitiesTable.start_nodes_signal.connect(self.on_start_nodes)
    self.capabilitiesTable.stop_nodes_signal.connect(self.on_stop_nodes)
    self.capabilitiesTable.description_requested_signal.connect(self.on_description_update_cap)
    self.ui.capabilities_tab.layout().addWidget(self.capabilitiesTable)

    self.ui.descriptionTextEdit.setOpenLinks(False)
    self.ui.descriptionTextEdit.anchorClicked.connect(self.on_description_anchorClicked)
    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+Shift+C", "copy selected description")), self.ui.descriptionTextEdit)
    self._shortcut_copy.activated.connect(self.ui.descriptionTextEdit.copy)
    self.ui.tabifyDockWidget(self.ui.launchDock, self.ui.descriptionDock)
    self.ui.tabifyDockWidget(self.ui.launchDock, self.ui.helpDock)
    self.ui.launchDock.raise_()
    self.ui.helpDock.setWindowIcon(QtGui.QIcon(':icons/crystal_clear_helpcenter.png'))

    flags = self.windowFlags()
    self.setWindowFlags(flags | QtCore.Qt.WindowContextHelpButtonHint)

    # creates a default config menu
    start_menu = QtGui.QMenu(self)
    self.loadDeafaultAtHostAct = QtGui.QAction("&Load default config on host", self, statusTip="Loads the default config at given host", triggered=self.on_load_as_default_at_host)
    start_menu.addAction(self.loadDeafaultAtHostAct)
    self.ui.loadXmlAsDefaultButton.setMenu(start_menu)

    self.default_load_launch = os.path.abspath(resolve_url(files[0])) if files else ''
    if self.default_load_launch:
      if os.path.isdir(self.default_load_launch):
        self.ui.xmlFileView.model().setPath(self.default_load_launch)
      elif os.path.isfile(self.default_load_launch):
        self.ui.xmlFileView.model().setPath(os.path.dirname(self.default_load_launch))

    self._discover_dialog = None
    self.restricted_to_one_master = restricted_to_one_master
    if restricted_to_one_master:
      self.ui.syncButton.setEnabled(False)
      self.ui.refreshAllButton.setEnabled(False)
      self.ui.discoveryButton.setEnabled(False)
      self.ui.startRobotButton.setEnabled(False)

    self._sync_dialog = SyncDialog()

    self.editor_dialogs  = dict() # [file] = XmlEditor
    '''@ivar: stores the open XmlEditor '''

    self.ui.simTimeLabel.setVisible(False)
    self.ui.launchServerLabel.setVisible(False)
    self.ui.hideDocksButton.clicked.connect(self.on_hide_docks_toggled)

    # since the is_local method is threaded for host names, call it to cache the localhost
    nm.is_local("localhost")

    # timer to update the showed update time of the ros state 
    self.master_timecheck_timer = QtCore.QTimer()
    self.master_timecheck_timer.timeout.connect(self.on_master_timecheck)
    self.master_timecheck_timer.start(1000)
    self._refresh_time = time.time()
    self._last_time_view_update = time.time()

    # set the help text
    try:
      from docutils import examples
      with file(nm.HELP_FILE) as f:
        self.ui.textBrowser.setText(examples.html_body(unicode(f.read())))
    except:
      import traceback
      msg = ''.join(["Error while generate help: ", str(traceback.format_exc())])
      rospy.logwarn(msg)
      self.ui.textBrowser.setText(msg)

    try:
      ScreenHandler.testScreen()
    except Exception as e:
      WarningMessageBox(QtGui.QMessageBox.Warning, "No SCREEN", 
                        "No SCREEN available! You can't launch nodes.",
                        str(e)).exec_()

    self.ui.imageLabel.mouseDoubleClickEvent = self.image_mouseDoubleClickEvent

    # =============================
    # Initialize the update handler
    # =============================

    # initialize the class to get the state of discovering of other ROS master
    self._update_handler = UpdateHandler()
    self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
    self._update_handler.error_signal.connect(self.on_master_info_error)

    # this monitor class is used, if no master_discovery node is running to get the state of the local ROS master
    self.own_master_monitor = OwnMasterMonitoring()
    self.own_master_monitor.init(22622)
    self.own_master_monitor.state_signal.connect(self.on_master_state_changed)
    self.own_master_monitor.err_signal.connect(self.on_master_monitor_err)

    # get the name of the service and topic of the discovery node. The name are determine by the message type  of those topics
    self.masterlist_service = masterlist_service = MasterListService()
    masterlist_service.masterlist_signal.connect(self.on_master_list_retrieved)
    masterlist_service.masterlist_err_signal.connect(self.on_master_list_err_retrieved)
    self.state_topic = MasterStateTopic()
    self.state_topic.state_signal.connect(self.on_master_state_changed)
    self.stats_topic = MasterStatisticTopic()
    self.stats_topic.stats_signal.connect(self.on_conn_stats_updated)

    self._con_tries = dict()
    self._subscribe()


  def on_hide_docks_toggled(self, checked):
    if self.ui.dockWidgetArea(self.ui.launchDock) == QtCore.Qt.LeftDockWidgetArea:
      self.ui.launchDock.setVisible(not checked)
    if self.ui.dockWidgetArea(self.ui.descriptionDock) == QtCore.Qt.LeftDockWidgetArea:
      self.ui.descriptionDock.setVisible(not checked)
    if self.ui.dockWidgetArea(self.ui.helpDock) == QtCore.Qt.LeftDockWidgetArea:
      self.ui.helpDock.setVisible(not checked)
    if self.ui.dockWidgetArea(self.ui.networkDock) == QtCore.Qt.LeftDockWidgetArea:
      self.ui.networkDock.setVisible(not checked)
    self.ui.hideDocksButton.setArrowType(QtCore.Qt.RightArrow if checked else QtCore.Qt.LeftArrow)
    historyFile = os.path.join(nm.CFG_PATH, 'view.history')
    with open(historyFile, 'w') as f:
      f.write(''.join(['selected_robot:=', self._history_selected_robot, '\n']))
      f.write(''.join(['show_left_docks:=', 'false' if checked else 'true', '\n']))
      f.write(''.join(['area_launch_dock:=', str(self.ui.dockWidgetArea(self.ui.launchDock)), '\n']))
      f.write(''.join(['area_descr_dock:=', str(self.ui.dockWidgetArea(self.ui.descriptionDock)), '\n']))

  def read_view_history(self):
    show_left_docks = True
    area_launch_dock = 1
    area_descr_dock = 1
    historyFile = os.path.join(nm.CFG_PATH, 'view.history')
    with open(historyFile, 'r') as f:
      line = f.readline()
      while line:
        if line:
          line = line.strip()
          if line:
            key, sep, value = line.partition(':=')
            if sep:
              if key == 'selected_robot':
                self._history_selected_robot = value
              if key == 'show_left_docks':
                show_left_docks = (value=='true')
              if key == 'area_launch_dock':
                area_launch_dock = int(value)
              if key == 'area_descr_dock':
                area_descr_dock = int(value)
        line = f.readline()
    if area_launch_dock != QtCore.Qt.LeftDockWidgetArea:
      self.ui.addDockWidget(area_launch_dock, self.ui.launchDock)
    if area_descr_dock != QtCore.Qt.LeftDockWidgetArea:
      self.ui.addDockWidget(area_descr_dock, self.ui.descriptionDock)
    self.ui.hideDocksButton.setChecked(not show_left_docks)
    self.on_hide_docks_toggled(not show_left_docks)

  def on_currentChanged_tab(self, index):
    pass
#    if index == self.ui.tabWidget.widget(0):
#      self.ui.networkDock.show()
#      self.ui.launchDock.show()
#    else:
#      self.ui.networkDock.hide()
#      self.ui.launchDock.hide()

  def closeEvent(self, event):
    self.finish()

  def finish(self):
    if not self._finished:
      self._finished = True
      print "Mainwindow finish..."
      self._progress_queue.stop()
      self._progress_queue_cfg.stop()
      self._progress_queue_sync.stop()
      self._update_handler.stop()
      self.state_topic.stop()
      self.stats_topic.stop()
      for key, master in self.masters.iteritems():
        master.stop()
      self.own_master_monitor.stop()
      self.master_timecheck_timer.stop()
      print "Mainwindow finished!"

  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it. The 'materuri' attribute will be set to the requested value.
    @return: ROS master URI
    @rtype: C{str} or C{None}
    '''
    if not hasattr(self, 'materuri') or self.materuri is None:
      masteruri = masteruri_from_ros()
      master = xmlrpclib.ServerProxy(masteruri)
      code, message, self.materuri = master.getUri(rospy.get_name())
      nm.is_local(nm.nameres().getHostname(self.materuri))
    return self.materuri

  def removeMaster(self, masteruri):
    '''
    Removed master with given master URI from the list.
    @param masteruri: the URI of the ROS master
    @type masteruri: C{str}
    '''
    if self.masters.has_key(masteruri):
      if not self.currentMaster is None and self.currentMaster.masteruri == masteruri:
        self.setCurrentMaster(None)
      self.masters[masteruri].stop()
      self.masters[masteruri].updateHostRequest.disconnect()
      self.masters[masteruri].host_description_updated.disconnect()
      self.masters[masteruri].capabilities_update_signal.disconnect()
      self.masters[masteruri].remove_config_signal.disconnect()
      self.masters[masteruri].description_signal.disconnect()
      self.masters[masteruri].request_xml_editor.disconnect()
      self.masters[masteruri].stop_nodes_signal.disconnect()
      self.masters[masteruri].robot_icon_updated.disconnect()
      self.stackedLayout.removeWidget(self.masters[masteruri])
      self.ui.tabPlace.layout().removeWidget(self.masters[masteruri])
      self.masters[masteruri].setParent(None)
      del self.masters[masteruri]

  def getMaster(self, masteruri, create_new=True):
    '''
    @return: the Widget which represents the master of given ROS master URI. If no
    Widget for given URI is available a new one will be created.
    @rtype: L{MasterViewProxy} 
    '''
    if not self.masters.has_key(masteruri):
      if not create_new:
        return None
      self.masters[masteruri] = MasterViewProxy(masteruri, self)
      self.masters[masteruri].updateHostRequest.connect(self.on_host_update_request)
      self.masters[masteruri].host_description_updated.connect(self.on_host_description_updated)
      self.masters[masteruri].capabilities_update_signal.connect(self.on_capabilities_update)
      self.masters[masteruri].remove_config_signal.connect(self.on_remove_config)
      self.masters[masteruri].description_signal.connect(self.on_description_update)
      self.masters[masteruri].request_xml_editor.connect(self._editor_dialog_open)
      self.masters[masteruri].stop_nodes_signal.connect(self.on_stop_nodes)
      self.masters[masteruri].robot_icon_updated.connect(self._on_robot_icon_changed)
      self.stackedLayout.addWidget(self.masters[masteruri])
      if masteruri == self.getMasteruri():
        if self.default_load_launch:
          try:
            if os.path.isfile(self.default_load_launch):
              args = list()
              args.append(''.join(['_package:=', str(package_name(os.path.dirname(self.default_load_launch))[0])]))
              args.append(''.join(['_launch_file:="', os.path.basename(self.default_load_launch), '"']))
              host = nm.nameres().address(masteruri)
              node_name = ''.join([str(nm.nameres().mastername(masteruri)), roslib.names.SEP, 
                                    os.path.basename(self.default_load_launch).replace('.launch',''), 
                                    roslib.names.SEP, 'default_cfg'])
              self._progress_queue_cfg.add2queue(str(uuid.uuid4()), 
                                             'start default config '+str(host), 
                                             nm.starter().runNodeWithoutConfig, 
                                             (host, 'default_cfg_fkie', 'default_cfg', node_name, args, masteruri, False, self.masters[masteruri].current_user))
              self._progress_queue_cfg.start()
          except Exception as e:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Load default configuration", 
                  ''.join(['Load default configuration ', self.default_load_launch, ' failed!']),
                  str(e)).exec_()
    return self.masters[masteruri]

  def on_host_update_request(self, host):
    for key, value in self.masters.items():
      if nm.nameres().getHostname(key) == host and not value.master_state is None:
        self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)

  def on_host_description_updated(self, masteruri, host, descr):
    self.master_model.updateDescription(nm.nameres().mastername(masteruri, host), descr)

  def on_capabilities_update(self, masteruri, address, config_node, descriptions):
    for d in descriptions:
      self.capabilitiesTable.updateCapabilities(masteruri, config_node, d)
    if not masteruri is None:
      master = self.getMaster(masteruri)
      self.capabilitiesTable.updateState(masteruri, master.master_info)

  def on_remove_config(self, cfg):
    self.capabilitiesTable.removeConfig(cfg)

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%            Handling of local monitoring            %%%%%%%%
  #%%%%%%%%%%%%%  (Backup, if no master_discovery node is running)  %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _subscribe(self):
    '''
    Try to subscribe to the topics of the master_discovery node. If it fails, the
    own local monitoring of the ROS master state will be enabled.
    '''
    if not self.restricted_to_one_master:
      result_1 = self.state_topic.registerByROS(self.getMasteruri(), False)
      result_2 = self.stats_topic.registerByROS(self.getMasteruri(), False)
      self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)
      if not result_1 or not result_2:
        self._setLocalMonitoring(True)
    else:
      self._setLocalMonitoring(True)

  def _setLocalMonitoring(self, on):
    '''
    Enables the local monitoring of the ROS master state and disables the view of
    the discoved ROS master.
    @param on: the enable / disable the local monitoring
    @type on: C{boolean}
    '''
    self.ui.masterTableView.setEnabled(not on)
    self.ui.refreshAllButton.setEnabled(not on)
    self.own_master_monitor.pause(not on)
    if on:
      self.ui.masterTableView.setToolTip("use 'Start' button to enable the master discovering")
    else:
      self.ui.masterTableView.setToolTip('')
    if on:
      # remove discovered ROS master and set the local master to selected
      for uri in self.masters.keys():
        master = self.masters[uri]
        if nm.is_local(nm.nameres().getHostname(uri)):
          if not self._history_selected_robot or master.mastername == self._history_selected_robot:
            self.setCurrentMaster(master)
        else:
          if not master.master_state is None:
            self.master_model.removeMaster(master.master_state.name)
          self.removeMaster(uri)
#      self.ui.masterTableView.doItemsLayout()



  def on_master_list_err_retrieved(self, masteruri, error):
    '''
    The callback method connected to the signal, which is emitted on an error 
    while call the service to determine the discovered ROS master. On the error
    the local monitoring will be enabled.
    '''
    self._setLocalMonitoring(True)

  def hasDiscoveryService(self, minfo):
    '''
    Test whether the new retrieved MasterInfo contains the master_discovery node.
    This is identified by a name of the contained 'list_masters' service.
    @param minfo: the ROS master Info
    @type minfo: L{master_discovery_fkie.MasterInfo}
    '''
    # use no discovery services, if roscore is running on a remote host
    if self.restricted_to_one_master:
      return False
    for service in minfo.services.keys():
      if service.endswith('list_masters'):
        return True
    return False


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Handling of received ROS master state messages   %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_master_list_retrieved(self, masteruri, servic_name, master_list):
    '''
    Handle the retrieved list with ROS master.
      1. update the ROS Network view
    @param master_list: a list with ROS masters
    @type master_list: C{[L{master_discovery_fkie.msg.MasterState}]}
    '''
    self._setLocalMonitoring(False)
    self._con_tries[masteruri] = 0
    for m in master_list:
      if not m.uri is None:
        host = nm.nameres().getHostname(m.uri)
        nm.nameres().addMasterEntry(m.uri, m.name, host, host)
        m.name = nm.nameres().mastername(m.uri)
        master = self.getMaster(m.uri)
        master.master_state = m
        self._assigne_icon(m.name)
        self.master_model.updateMaster(m)
        self._update_handler.requestMasterInfo(m.uri, m.monitoruri)

  def on_master_state_changed(self, msg):
    '''
    Handle the received master state message.
      1. update the ROS Network view
      2. enable local master monitoring, if all masters are removed (the local master too)
    @param msg: the ROS message with new master state
    @type msg: L{master_discovery_fkie.msg.MasterState}
    '''
    #'print "*on_master_state_changed"
    host=nm.nameres().getHostname(msg.master.uri)
    if msg.state == MasterState.STATE_CHANGED:
      nm.nameres().addMasterEntry(msg.master.uri, msg.master.name, host, host)
      msg.master.name = nm.nameres().mastername(msg.master.uri)
      self.getMaster(msg.master.uri).master_state = msg.master
      self._assigne_icon(msg.master.name)
      self.master_model.updateMaster(msg.master)
#      self.ui.masterTableView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == MasterState.STATE_NEW:
      nm.nameres().addMasterEntry(msg.master.uri, msg.master.name, host, host)
      msg.master.name = nm.nameres().mastername(msg.master.uri)
      self.getMaster(msg.master.uri).master_state = msg.master
      self._assigne_icon(msg.master.name)
      self.master_model.updateMaster(msg.master)
#      self.ui.masterTableView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == MasterState.STATE_REMOVED:
      if msg.master.uri == self.getMasteruri():
        # switch to locale monitoring, if the local master discovering was removed
        self._setLocalMonitoring(True)
      else:
        nm.nameres().removeMasterEntry(msg.master.uri)
        self.master_model.removeMaster(msg.master.name)
#        self.ui.masterTableView.doItemsLayout()
        self.removeMaster(msg.master.uri)
#      if len(self.masters) == 0:
#        self._setLocalMonitoring(True)
    #'print "**on_master_state_changed"

  def _assigne_icon(self, name, path=None):
    '''
    Sets the new icon to the given robot. If the path is `None` set search for
    .png file with robot name.
    :param name: robot name
    :type name: str
    :param path: path of the icon (Default: None)
    :type path: str
    '''
    icon_path = path if path else ''.join([nm.ROBOTS_DIR, name, '.png'])
    if not self.__icons.has_key(name) or self.__icons[name][1] != path:
      if QtCore.QFile.exists(icon_path):
        self.__icons[name] = (QtGui.QIcon(icon_path), icon_path)
      elif self.__icons.has_key(name):
        del self.__icons[name]

  def on_master_monitor_err(self, msg):
    self._con_tries[self.getMasteruri()] += 1

  def on_master_info_retrieved(self, minfo):
    '''
    Integrate the received master info.
    @param minfo: the ROS master Info
    @type minfo: L{master_discovery_fkie.MasterInfo}
    '''
    rospy.logdebug("MASTERINFO from %s (%s) received", minfo.mastername, minfo.masteruri)
    self._con_tries[minfo.masteruri] = 0
#    cputimes_m = os.times()
#    cputime_init_m = cputimes_m[0] + cputimes_m[1]
    if self.masters.has_key(minfo.masteruri):
      for uri, master in self.masters.items():
        try:
          # check for running discovery service
          new_info = master.master_info is None or master.master_info.timestamp < minfo.timestamp
#          cputimes = os.times()
#          cputime_init = cputimes[0] + cputimes[1]
          master.master_info = minfo
#          cputimes = os.times()
#          cputime = cputimes[0] + cputimes[1] - cputime_init
#          print master.master_state.name, cputime
          if not master.master_info is None:
            if self._history_selected_robot == minfo.mastername and self._history_selected_robot == master.mastername and self.currentMaster != master:
              if not self.currentMaster is None and not self.currentMaster.is_local:
                self.setCurrentMaster(master)
            elif nm.is_local(nm.nameres().getHostname(master.master_info.masteruri)) or self.restricted_to_one_master:
              if new_info:
                has_discovery_service = self.hasDiscoveryService(minfo)
                if not self.own_master_monitor.isPaused() and has_discovery_service:
                  self._subscribe()
                elif self.currentMaster is None and (not self._history_selected_robot or self._history_selected_robot == minfo.mastername):
                  self.setCurrentMaster(master)

            # update the list view, whether master is synchronized or not
            if master.master_info.masteruri == minfo.masteruri:
              self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
          self.capabilitiesTable.updateState(minfo.masteruri, minfo)
        except Exception, e:
          rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, str(e))
      # update the duplicate nodes
      self.updateDuplicateNodes()
      # update the buttons, whether master is synchronized or not
      if not self.currentMaster is None and not self.currentMaster.master_info is None and not self.restricted_to_one_master:
        self.ui.syncButton.setEnabled(True)
        self.ui.syncButton.setChecked(not self.currentMaster.master_info.getNodeEndsWith('master_sync') is None)
#    cputimes_m = os.times()
#    cputime_m = cputimes_m[0] + cputimes_m[1] - cputime_init_m
#    print "ALL:", cputime_m

  def on_master_info_error(self, masteruri, error):
    if not self._con_tries.has_key(masteruri):
      self._con_tries[masteruri] = 0
    self._con_tries[masteruri] += 1
    if masteruri == self.getMasteruri():
      rospy.logwarn("Error while connect to local master_discovery %s: %s", masteruri, error)
      # switch to local monitoring after 3 timeouts
#      self._local_tries += 1
#      if self._local_tries > 2:
#        print "CONNECTION ERROR2222222"
#        self._setLocalMonitoring(True)
#      elif not masteruri is None:
    master = self.getMaster(masteruri, False)
    if master and not master.master_state is None:
      self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri, self.DELAYED_NEXT_REQ_ON_ERR)

  def on_conn_stats_updated(self, stats):
    '''
    Handle the retrieved connection statistics.
      1. update the ROS Network view
    @param stats: a list with connection statistics
    @type stats: C{[L{master_discovery_fkie.msg.LinkState}]}
    '''
    #'print "+on_conn_stats_updated"
    for stat in stats.links:
      self.master_model.updateMasterStat(stat.destination, stat.quality)
    #'print "++on_conn_stats_updated"


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master info frame         %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_info_clicked(self):
    text = ''.join(['<dl>'])
    text = ''.join([text, '<dt><b>Maintainer</b>: ', 'Alexander Tiderko ',  '<font color=gray>alexander.tiderko@gmail.com</font>', '</dt>'])
    text = ''.join([text, '<dt><b>Author</b>: ', 'Alexander Tiderko, Timo Roehling', '</dt>'])
    text = ''.join([text, '<dt><b>License</b>: ', 'BSD, some icons are licensed under the GNU Lesser General Public License (LGPL) or Creative Commons Attribution-Noncommercial 3.0 License', '</dt>'])
    text = ''.join([text, '</dl>'])
    text = ''.join([text, '<dt><b>Version</b>: ', nm.__version__, ' (', nm.__date__,')', '</dt>'])
    QtGui.QMessageBox.about(self, 'About Node Manager', text)

  def on_refresh_master_clicked(self):
    if not self.currentMaster is None:
      rospy.loginfo("Request an update from %s", str(self.currentMaster.master_state.monitoruri))
      check_ts = self.currentMaster.master_info.check_ts
      self.currentMaster.master_info.timestamp = self.currentMaster.master_info.timestamp - 1.0
      self.currentMaster.master_info.check_ts = check_ts
      self._update_handler.requestMasterInfo(self.currentMaster.master_state.uri, self.currentMaster.master_state.monitoruri)
#      self.currentMaster.remove_all_def_configs()

  def on_run_node_clicked(self):
    '''
    Open a dialog to run a ROS node without a configuration
    '''
    from run_dialog import RunDialog
    if not self.currentMaster is None:
      dia = RunDialog(nm.nameres().getHostname(self.currentMaster.masteruri), self.currentMaster.masteruri)
      if dia.exec_():
        dia.runSelected()

  def on_show_rxconsole_clicked(self):
    if not self.currentMaster is None:
      import os, subprocess
      env = dict(os.environ)
      env["ROS_MASTER_URI"] = str(self.currentMaster.master_state.uri)
      cmd = 'rxconsole'
      try:
        import rqt_console
        cmd = 'rqt_console'
      except:
        pass
      rospy.loginfo("start rxconsole: %s", cmd)
      ps = subprocess.Popen([cmd], env=env)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=ps.wait)
      thread.setDaemon(True)
      thread.start()

  def on_show_rxgraph_clicked(self):
    if not self.currentMaster is None:
      import os, subprocess
      env = dict(os.environ)
      env["ROS_MASTER_URI"] = str(self.currentMaster.master_state.uri)
      cmd = 'rxgraph'
      try:
        import rqt_graph
        cmd = 'rqt_graph'
      except:
        pass
      rospy.loginfo("start rxgraph: %s", cmd)
      ps = subprocess.Popen([cmd], env=env)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=ps.wait)
      thread.setDaemon(True)
      thread.start()

  def on_sync_dialog_released(self, released=False, masteruri=None, external_call=False):
    self.ui.syncButton.setEnabled(False)
    master = self.currentMaster
    if not masteruri is None:
      master = self.getMaster(masteruri, False)
    if not master is None and (self.ui.syncButton.isChecked() or external_call):
      self._sync_dialog.resize(350,190)
      if self._sync_dialog.exec_():
        try:
          host = nm.nameres().getHostname(master.masteruri)
          if not self._sync_dialog.interface_filename is None:
            # copy the interface file to remote machine
            self._progress_queue_sync.add2queue(str(uuid.uuid4()),
                                           'Transfer sync interface '+str(host), 
                                           nm.starter().transfer_files, 
                                           (str(host), self._sync_dialog.interface_filename, False, master.current_user))
          self._progress_queue_sync.add2queue(str(uuid.uuid4()), 
                                         'Start sync on '+str(host), 
                                         nm.starter().runNodeWithoutConfig, 
                                         (str(host), 'master_sync_fkie', 'master_sync', 'master_sync', self._sync_dialog.sync_args, str(master.masteruri), False, master.current_user))
          self._progress_queue_sync.start()
        except:
          import traceback
          WarningMessageBox(QtGui.QMessageBox.Warning, "Start sync error", 
                            "Error while start sync node",
                            str(traceback.format_exc())).exec_()
      else:
        self.ui.syncButton.setChecked(False)
    elif not master is None and not master.master_info is None:
      node = master.master_info.getNodeEndsWith('master_sync')
      if not node is None:
        master.stop_nodes([node])
    self.ui.syncButton.setEnabled(True)

  def on_sync_released(self, external_call=False):
    '''
    Enable or disable the synchronization of the master cores
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
      if external_call:
        self.on_sync_dialog_released()
#      else:
#        self.ui.syncButton.showMenu()
      if not self.currentMaster.master_info is None:
        node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
        self.ui.syncButton.setChecked(not node is None)
    else:
      self.ui.syncButton.setEnabled(False)
      if not self.currentMaster is None:
        if self.ui.syncButton.isChecked():
          # ask the user to start the master_sync with loaded launch file
          if not self.currentMaster.master_info is None:
            node = self.currentMaster.getNode('/master_sync')
            if node:
              def_cfg_info = '\nNote: default_cfg parameter will be changed!' if node[0].has_default_cfgs(node[0].cfgs) else ''
              ret = QtGui.QMessageBox.question(self, 'Start synchronization','Start the synchronization using loaded configuration?\n\n `No` starts the master_sync with default parameter.%s'%def_cfg_info, QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
              if ret == QtGui.QMessageBox.Yes:
                self.currentMaster.start_nodes([node[0]])
                return

          # start the master sync with default settings
          sync_args = []
          sync_args.append(''.join(['_interface_url:=', "'.'"]))
          sync_args.append(''.join(['_sync_topics_on_demand:=', 'False']))
          sync_args.append(''.join(['_ignore_hosts:=', '[]']))
          sync_args.append(''.join(['_sync_hosts:=', '[]']))
          sync_args.append(''.join(['_ignore_nodes:=', '[]']))
          sync_args.append(''.join(['_sync_nodes:=', '[]']))
          sync_args.append(''.join(['_ignore_topics:=', '[]']))
          sync_args.append(''.join(['_sync_topics:=', '[]']))
          sync_args.append(''.join(['_ignore_services:=', '[]']))
          sync_args.append(''.join(['_sync_services:=', '[]']))
          sync_args.append(''.join(['_sync_remote_nodes:=', 'False']))

          try:
            host = nm.nameres().getHostname(self.currentMaster.masteruri)
            self._progress_queue_sync.add2queue(str(uuid.uuid4()), 
                                           'start sync on '+str(host), 
                                           nm.starter().runNodeWithoutConfig, 
                                           (str(host), 'master_sync_fkie', 'master_sync', 'master_sync', sync_args, str(self.currentMaster.masteruri), False, self.currentMaster.current_user))
            self._progress_queue_sync.start()
          except:
            pass
        elif not self.currentMaster.master_info is None:
          node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
          if not node is None:
            self.currentMaster.stop_nodes([node])
      self.ui.syncButton.setEnabled(True)

  def on_master_timecheck(self):
    # HACK: sometimes the local monitoring will not be activated. This is the detection.
    if len(self.masters) < 2 and self.currentMaster is None:
      self._subscribe()
      return
    # update the info panel of the robot. If the node manager is not selected the updates are rarer. 
    current_time = time.time()
    if self.isActiveWindow() or current_time - self._last_time_view_update > 5:
      self._last_time_view_update = current_time
      if not self.currentMaster is None and not self.currentMaster.master_state is None:
        master = self.getMaster(self.currentMaster.master_state.uri)
        name = master.master_state.name
        masteruri = master.master_state.uri
        if self.restricted_to_one_master:
          name = ''.join([name, ' <span style=" color:red;">(restricted)</span>'])
          if not self.ui.masternameLabel.toolTip():
            self.ui.masternameLabel.setToolTip('The multicore options are disabled, because the roscore is running on remote host!')
        if not master.master_info is None:
          self.showMasterName(masteruri, name, self.timestampStr(master.master_info.check_ts), master.master_state.online)
          pass
        elif not master.master_state is None:
          self.showMasterName(masteruri, name, 'Try to get info!!!', master.master_state.online)
      else:
        self.showMasterName('', 'No robot selected', None, False)
    if (current_time - self._refresh_time > 30.0):
      masteruri = self.getMasteruri()
      if not masteruri is None:
        master = self.getMaster(masteruri)
        if not master is None and not master.master_state is None:
          self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri)
        self._refresh_time = current_time


  def showMasterName(self, masteruri, name, timestamp, online=True):
    '''
    Update the view of the info frame.
    '''
    con_err = ''
    try:
      tries = self._con_tries[masteruri]
      if tries > 1:
        con_err = '<span style=" color:red;">connection problems (%s tries)! </span>'%str(tries)
    except:
      pass
    if self.__current_master_label_name != name:
      self.__current_master_label_name = name
      self.ui.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s</span>'%name)
    ts = 'updated: %s'%str(timestamp) if not timestamp is None else ''
    self.ui.masterInfoLabel.setText('<span style=" font-size:8pt;">%s%s</span>'%(con_err, ts))

    # load the robot image, if one exists
    if self.ui.masternameLabel.isEnabled():
      if self.__icons.has_key(name):
        if self.__icons[name][0] != self.__current_icon:
          icon = self.__icons[name][0]
          self.__current_icon = icon
          self.ui.imageLabel.setPixmap(icon.pixmap(self.ui.nameFrame.size()))
          self.ui.imageLabel.setToolTip(''.join(['<html><head></head><body><img src="', self.__icons[name][1], '" alt="', name,'"></body></html>']))
      elif self.__icons['default_pc'][0] != self.__current_icon:
        icon = self.__icons['default_pc'][0]
        self.__current_icon = icon
        self.ui.imageLabel.setPixmap(icon.pixmap(self.ui.nameFrame.size()))
        self.ui.imageLabel.setToolTip('')
    # set sim_time info
    master = self.getMaster(masteruri, False)
    sim_time_enabled = self.ui.masternameLabel.isEnabled() and not master is None and master.use_sim_time
    self.ui.simTimeLabel.setVisible(sim_time_enabled)
    launch_server_enabled = self.ui.masternameLabel.isEnabled() and (not master is None) and master.has_launch_server()
    self.ui.launchServerLabel.setVisible(launch_server_enabled)
    self.ui.masternameLabel.setEnabled(online)
    self.ui.masterInfoFrame.setEnabled((not timestamp is None))

  def timestampStr(self, timestamp):
    dt = datetime.fromtimestamp(timestamp)
    diff = time.time()-timestamp
    diff_dt = datetime.fromtimestamp(diff)
    before = '0 sec'
    if (diff < 60):
      before = diff_dt.strftime('%S sec')
    elif (diff < 3600):
      before = diff_dt.strftime('%M:%S min')
    elif (diff < 86400):
      before = diff_dt.strftime('%H:%M:%S std')
    else:
      before = diff_dt.strftime('%d Day(s) %H:%M:%S')
    return ''.join([dt.strftime('%H:%M:%S'), ' (', before, ')'])

  def updateDuplicateNodes(self):
    # update the duplicate nodes
    running_nodes = dict()
    for uri, m in self.masters.items():
      if not m.master_state is None and m.master_state.online:
        running_nodes.update(m.getRunningNodesIfLocal())
    for uri, m in self.masters.items():
      if not m.master_state is None:
        m.markNodesAsDuplicateOf(running_nodes)



  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master list view          %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_master_table_pressed(self, selected):
    pass

  def on_master_table_clicked(self, selected):
    '''
    On click on the sync item, the master_sync node will be started or stopped,
    depending on run state.
    '''
    item = self.master_model.itemFromIndex(selected)
    if isinstance(item, MasterSyncItem):
      self.ui.syncButton.setChecked(not item.synchronized)
      item.synchronized = not item.synchronized
      self.on_sync_released(True)
      item.synchronized = self.ui.syncButton.isChecked()

  def on_master_table_activated(self, selected):
    pass

  def on_master_selection_changed(self, selected):
    '''
    If a master was selected, set the corresponding Widget of the stacked layout
    to the current widget and shows the state of the selected master.
    '''
#     si = self.ui.masterTableView.selectedIndexes()
#     for index in si:
#       if index.row() == selected.row():
    item = self.master_model.itemFromIndex(selected)
    if not item is None:
      self._history_selected_robot = item.master.name
      self.setCurrentMaster(item.master.uri)
      if not self.currentMaster.master_info is None and not self.restricted_to_one_master:
        node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
        self.ui.syncButton.setEnabled(True)
        self.ui.syncButton.setChecked(not node is None)
      else:
        self.ui.syncButton.setEnabled(False)
      return
    self.ui.launchDock.raise_()

  def setCurrentMaster(self, master):
    '''
    Changes the view of the master.
    :param master: the MasterViewProxy object or masteruri
    :type master: MasterViewProxy or str
    '''
    show_user_field = False
    if isinstance(master, MasterViewProxy):
      self.currentMaster = master
      self.stackedLayout.setCurrentWidget(master)
      show_user_field = not master.is_local
      self._add_user_to_combo(self.currentMaster.current_user)
      self.ui.userComboBox.setEditText(self.currentMaster.current_user)
    elif master is None:
      self.currentMaster = None
      self.stackedLayout.setCurrentIndex(0)
    else: # it's masteruri
      self.currentMaster = self.getMaster(master)
      if not self.currentMaster is None:
        self.stackedLayout.setCurrentWidget(self.currentMaster)
        show_user_field = not self.currentMaster.is_local
        self._add_user_to_combo(self.currentMaster.current_user)
        self.ui.userComboBox.setEditText(self.currentMaster.current_user)
      else:
        self.stackedLayout.setCurrentIndex(0)
    self.ui.user_frame.setVisible(show_user_field)
    self.on_master_timecheck()

  def _add_user_to_combo(self, user):
    for i in range(self.ui.userComboBox.count()):
      if user.lower() == self.ui.userComboBox.itemText(i).lower():
        return
    self.ui.userComboBox.addItem(user)

  def on_user_changed(self, user):
    if not self.currentMaster is None:
      self.currentMaster.current_user = user

  def on_masterTableView_selection_changed(self, selected, deselected):
    '''
    On selection of a master list.
    '''
    if selected.isValid():
      self.on_master_selection_changed(selected)

  def on_all_master_refresh_clicked(self):
    '''
    Retrieves from the master_discovery node the list of all discovered ROS 
    master and get their current state.
    '''
    # set the timestamp of the current master info back
    for uri, m in self.masters.items():
      if not m.master_info is None:
        check_ts = m.master_info.check_ts
        m.master_info.timestamp = m.master_info.timestamp - 1.0
        m.master_info.check_ts = check_ts
    self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)

  def on_discover_network_clicked(self):
    try:
      self._discover_dialog.raise_()
    except:
      self._discover_dialog = NetworkDiscoveryDialog('226.0.0.0', 11511, 100, self)
      self._discover_dialog.network_join_request.connect(self._join_network)
      self._discover_dialog.show()

  def on_start_robot_clicked(self):
    '''
    Tries to start the master_discovery node on the machine requested by a dialog.
    '''
    # get the history list
    params_optional = {'Discovery type': ('string', ['master_discovery', 'zeroconf']),
                       'ROS Master Name' : ('string', 'autodetect'),
                       'ROS Master URI' : ('string', 'ROS_MASTER_URI'),
                       'Static hosts' : ('string', ''),
                       'Username' : ('string', [self.ui.userComboBox.itemText(i) for i in reversed(range(self.ui.userComboBox.count()))]),
                       'Send MCast' : ('bool', True),
                      }
    params = {'Host' : ('string', 'localhost'),
              'Network(0..99)' : ('int', '0'),
              'Optional Parameter' : ('list', params_optional) }
    dia = ParameterDialog(params, sidebar_var='Host')
    dia.setFilterVisible(False)
    dia.setWindowTitle('Start discovery')
    dia.resize(450,280)
    dia.setFocusField('Host')
    if dia.exec_():
      try:
        params = dia.getKeywords()
        hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
        port = params['Network(0..99)']
        discovery_type = params['Optional Parameter']['Discovery type']
        mastername = 'autodetect'
        masteruri = 'ROS_MASTER_URI'
        if len(hostnames) < 2:
          mastername = params['Optional Parameter']['ROS Master Name']
          masteruri = params['Optional Parameter']['ROS Master URI']
        static_hosts = params['Optional Parameter']['Static hosts']
        username = params['Optional Parameter']['Username']
        send_mcast = params['Optional Parameter']['Send MCast']
        if static_hosts:
          static_hosts = static_hosts.replace(' ', '')
          static_hosts = static_hosts.replace('[', '')
          static_hosts = static_hosts.replace(']', '')
        for hostname in hostnames:
          try:
            args = []
            if not port is None and port and int(port) < 100 and int(port) >= 0:
              args.append(''.join(['_mcast_port:=', str(11511 + int(port))]))
            else:
              args.append(''.join(['_mcast_port:=', str(11511)]))
            if not mastername == 'autodetect':
              args.append(''.join(['_name:=', str(mastername)]))
            args.append('_send_mcast:=%s'%str(send_mcast))
            args.append(''.join(['_static_hosts:=[', static_hosts, ']']))
            #TODO: remove the name parameter from the ROS parameter server
            self._progress_queue.add2queue(str(uuid.uuid4()), 
                                           'start discovering on '+str(hostname), 
                                           nm.starter().runNodeWithoutConfig, 
                                           (str(hostname), 'master_discovery_fkie', str(discovery_type), str(discovery_type), args, (None if masteruri == 'ROS_MASTER_URI' else str(masteruri)), False, username))

          except (Exception, nm.StartException), e:
            import traceback
            print traceback.format_exc()
            rospy.logwarn("Error while start master_discovery for %s: %s", str(hostname), str(e))
            WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                              'Error while start master_discovery',
                              str(e)).exec_()
          self._progress_queue.start()
      except Exception, e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                          'Error while parse parameter',
                          str(e)).exec_()

  def _join_network(self, network):
    try:
      hostname = 'localhost'
      args = []
      if network < 100 and network >= 0:
        args.append(''.join(['_mcast_port:=', str(11511 + int(network))]))
      self._progress_queue.add2queue(str(uuid.uuid4()), 
                                     'start discovering on '+str(hostname), 
                                     nm.starter().runNodeWithoutConfig, 
                                     (str(hostname), 'master_discovery_fkie', 'master_discovery', 'master_discovery', args, None, False))
      self._progress_queue.start()
    except (Exception, nm.StartException), e:
      rospy.logwarn("Error while start master_discovery for %s: %s", str(hostname), str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                        'Error while start master_discovery',
                        str(e)).exec_()

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of the launch file view      %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_launch_selection_activated(self, activated):
    '''
    Tries to load the launch file, if one was activated.
    '''
    item, path, id = activated.model().items[activated.row()]
    try:
#      self.ui.xmlFileView.setEnabled(False)
      file = activated.model().expandItem(item, path)
      if not file is None:
        self.loadLaunchFile(path, row=activated.row())
#      self.ui.xmlFileView.setEnabled(True)
#      self.ui.xmlFileView.setFocus(QtCore.Qt.ActiveWindowFocusReason)
    except Exception, e:
#      self.ui.xmlFileView.setEnabled(True)
      rospy.logwarn("Error while load launch file %s: %s", str(item), str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Load error", 
                        ''.join(['Error while load launch file:\n', item]),
                        str(e)).exec_()

  def on_xmlFileView_selection_changed(self, selected, deselected):
    '''
    On selection of a launch file, the buttons are enabled otherwise disabled.
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      isfile = self.ui.xmlFileView.model().isLaunchFile(index.row())
      self.ui.editXmlButton.setEnabled(isfile)
      self.ui.loadXmlButton.setEnabled(isfile)
      self.ui.transferButton.setEnabled(isfile)
      self.ui.loadXmlAsDefaultButton.setEnabled(isfile)

  def on_refresh_xml_clicked(self):
    '''
    Reload the current path.
    '''
    self.ui.xmlFileView.model().reloadCurrentPath()
    self.ui.editXmlButton.setEnabled(False)
    self.ui.loadXmlButton.setEnabled(False)
    self.ui.transferButton.setEnabled(False)
    self.ui.loadXmlAsDefaultButton.setEnabled(False)
    
  def on_edit_xml_clicked(self):
    '''
    Opens an XML editor to edit the launch file. 
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().expandItem(pathItem, path)
      if not path is None:
        self._editor_dialog_open([path], '')

  def on_new_xml_clicked(self):
    '''
    Creates a new launch file.
    '''
    (fileName, filter) = QtGui.QFileDialog.getSaveFileName(self,
                                                 "New launch file",
                                                 self.__current_path if self.ui.xmlFileView.model().currentPath is None else self.ui.xmlFileView.model().currentPath,
                                                 "Config files (*.launch *.yaml);;All files (*)")
    if fileName:
      try:
        (pkg, pkg_path) = package_name(os.path.dirname(fileName))
        if pkg is None:
          WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                         'The new file is not in a ROS package').exec_()
          return
        with open(fileName, 'w+') as f:
          f.write("<launch>\n"
                  "  <arg name=\"robot_ns\" default=\"my_robot\"/>\n"
                  "  <group ns=\"$(arg robot_ns)\">\n"
                  "    <param name=\"tf_prefix\" value=\"$(arg robot_ns)\"/>\n"
                  "\n"
                  "    <node pkg=\"my_pkg\" type=\"my_node\" name=\"my_name\" >\n"
                  "      <param name=\"capability_group\" value=\"MY_GROUP\"/>\n"
                  "    </node>\n"
                  "  </group>\n"
                  "</launch>\n"
                  )
        self._editor_dialog_open([fileName], '')
        self.__current_path = os.path.dirname(fileName)
        self.ui.xmlFileView.model().setPath(self.__current_path)
      except EnvironmentError as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                         'Error while create a new file',
                          str(e)).exec_()

  def on_open_xml_clicked(self):
    (fileName, filter) = QtGui.QFileDialog.getOpenFileName(self,
                                                 "Load launch file", 
                                                 self.__current_path, 
                                                 "Config files (*.launch);;All files (*)")
    if fileName:
      self.__current_path = os.path.dirname(fileName)
      self.loadLaunchFile(fileName)

  def on_transfer_file_clicked(self):
    '''
    Copies the selected file to 
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().expandItem(pathItem, path)
      if not path is None:
        host = 'localhost'
        username = nm.ssh().USER_DEFAULT
        if not self.currentMaster is None:
          host = nm.nameres().getHostname(self.currentMaster.masteruri)
          username = self.currentMaster.current_user
        params = {'Host' : ('string', host),
                  'recursive' : ('bool', 'False'),
                  'Username' : ('string', str(username)) }
        dia = ParameterDialog(params)
        dia.setFilterVisible(False)
        dia.setWindowTitle('Transfer file')
        dia.resize(350,120)
        dia.setFocusField('Host')
        if dia.exec_():
          try:
            params = dia.getKeywords()
            host = params['Host']
            rospy.loginfo("TRANSFER the launch file to host %s@%s: %s", str(username), str(host), path)
            recursive = params['recursive']
            username = params['Username']
            self._progress_queue_cfg.add2queue(str(uuid.uuid4()), 
                                           'transfer files to '+str(host), 
                                           nm.starter().transfer_files, 
                                           (str(host), path, False, username))
            if recursive:
              for f in LaunchConfig.getIncludedFiles(path):
                self._progress_queue_cfg.add2queue(str(uuid.uuid4()), 
                                               'transfer files to '+str(host), 
                                               nm.starter().transfer_files, 
                                               (str(host), f, False, username))
            self._progress_queue_cfg.start()
          except Exception, e:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Transfer error", 
                             'Error while transfer files',
                              str(e)).exec_()

  def _editor_dialog_open(self, files, search_text, trynr=1):
    if files:
      path = files[0]
      if self.editor_dialogs.has_key(path):
        last_path = files[-1]
        try:
          self.editor_dialogs[path].on_load_request(last_path, search_text)
          self.editor_dialogs[path].raise_()
          self.editor_dialogs[path].activateWindow()
        except:
          if trynr > 1:
            raise
          del self.editor_dialogs[path]
          self._editor_dialog_open(files, search_text, 2)
      else:
        editor = XmlEditor(files, search_text, self)
        self.editor_dialogs[path] = editor
        editor.finished_signal.connect(self._editor_dialog_closed)
        editor.show()

  def _editor_dialog_closed(self, files):
    if self.editor_dialogs.has_key(files[0]):
      del self.editor_dialogs[files[0]]

  def _reload_globals_at_next_start(self, file):
    if not self.currentMaster is None:
      self.currentMaster.reload_global_parameter_at_next_start(file)

  def on_load_xml_clicked(self):
    '''
    Tries to load the selected launch file. The button is only enabled and this
    method is called, if the button was enabled by on_launch_selection_clicked()
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().expandItem(pathItem, path)
      if not path is None:
        self.loadLaunchFile(path, row=index.row())

  def loadLaunchFile(self, path, force_as_default=False, host=None, row=None):
    '''
    Load the launch file. A ROS master mast be selected first.
    @param path: the path of the launch file.
    @type path: C{str}
    '''
    rospy.loginfo("LOAD the launch file: %s", path)
    master_proxy = self.stackedLayout.currentWidget()
    if isinstance(master_proxy, MasterViewProxy):
      cursor = self.cursor()
#      self.setCursor(QtCore.Qt.WaitCursor)
#      self.ui.xmlFileView.setEnabled(False)
      self.ui.xmlFileView.model().add2LoadHistory(path)
      try:
        if not row is None:
          sm = self.ui.xmlFileView.selectionModel()
          sm.select(self.ui.xmlFileView.model().createIndex(row, 0), QtGui.QItemSelectionModel.Select)
      except:
        pass
#      QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
      #todo: except errors on determination of the defaul_cfg name
      key_mod = QtGui.QApplication.keyboardModifiers()
      if (key_mod & QtCore.Qt.ShiftModifier) or force_as_default:
        args = list()
        args.append(''.join(['_package:=', str(package_name(os.path.dirname(path))[0])]))
        args.append(''.join(['_launch_file:="', os.path.basename(path), '"']))
        try:
          # test for requerid args
          launchConfig = LaunchConfig(path)
          req_args = launchConfig.getArgs()
          if req_args:
            params = dict()
            arg_dict = launchConfig.argvToDict(req_args)
            for arg in arg_dict.keys():
              params[arg] = ('string', [arg_dict[arg]])
            inputDia = ParameterDialog(params)
            inputDia.setFilterVisible(False)
            inputDia.setWindowTitle(''.join(['Enter the argv for ', path]))
            if inputDia.exec_():
              params = inputDia.getKeywords()
              args.extend(launchConfig.resolveArgs([''.join([p, ":='", v, "'"]) for p,v in params.items() if v]))
            else:
#              self.ui.xmlFileView.setEnabled(True)
#              self.setCursor(cursor)
              return
        except:
          import traceback
          print traceback.format_exc()
        hostname = nm.nameres().address(master_proxy.masteruri) if host is None else host
#        name_prefix = str(nm.nameres().mastername(master_proxy.masteruri) if host is None else host)
#        name_prefix = os.path.basename(path).replace(' ', '_')
        node_name = ''.join([str(nm.nameres().mastername(master_proxy.masteruri) if host is None else host), 
                             roslib.names.SEP, 
                             os.path.basename(path).replace('.launch','').replace(' ', '_'), 
                             roslib.names.SEP, 'default_cfg'])
        self._progress_queue_cfg.add2queue(str(uuid.uuid4()),
                                       'start default config '+str(hostname), 
                                       nm.starter().runNodeWithoutConfig, 
                                       (str(hostname), 'default_cfg_fkie', 'default_cfg', node_name, args, master_proxy.masteruri, False, master_proxy.current_user))
        self._progress_queue_cfg.start()
      else:
        try:
          master_proxy.launchfiles = path
          # update the duplicate nodes: will be updated on master_info receive
#          self.updateDuplicateNodes()
        except Exception, e:
          import traceback
          print traceback.format_exc()
          WarningMessageBox(QtGui.QMessageBox.Warning, "Loading launch file", path, str(e)).exec_()
#      self.ui.xmlFileView.setEnabled(True)
#      self.setCursor(cursor)
    else:
      QtGui.QMessageBox.information(self, "Load of launch file",
                                    "Select a master first!", )

  def on_load_as_default_at_host(self):
    '''
    Tries to load the selected launch file as default configuration. The button 
    is only enabled and this method is called, if the button was enabled by 
    on_launch_selection_clicked()
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().expandItem(pathItem, path)
      if not path is None:
        params = {'Host' : ('string', 'localhost') }
        dia = ParameterDialog(params)
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start node on...')
        dia.resize(350,120)
        dia.setFocusField('Host')
        if dia.exec_():
          try:
            params = dia.getKeywords()
            host = params['Host']
            rospy.loginfo("LOAD the launch file on host %s as default: %s", str(host), path)
            self.loadLaunchFile(path, True, host)
          except Exception, e:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Load default config error", 
                             'Error while parse parameter',
                              str(e)).exec_()


  def on_load_as_default(self):
    '''
    Tries to load the selected launch file as default configuration. The button 
    is only enabled and this method is called, if the button was enabled by 
    on_launch_selection_clicked()
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier):
      self.ui.loadXmlAsDefaultButton.showMenu()
    else:
      indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
      for index in indexes:
        pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
        path = self.ui.xmlFileView.model().expandItem(pathItem, path)
        if not path is None:
          rospy.loginfo("LOAD the launch file as default: %s", path)
          self.loadLaunchFile(path, True)

  def on_configfile_changed(self, changed, affected):
    '''
    Signal hander to handle the changes of a loaded configuration file
    @param changed: the changed file
    @type changed: C{str}
    @param affected: the list of tuples with masteruri and launchfile, which are affected by file change
    @type affected: list
    '''
    # create a list of launch files and masters, which are affected by the changed file
    # and are not currently in question
    if self.isActiveWindow():
      self._changed_files[changed] = affected
      self._check_for_changed_files()
    else:
      self._changed_files[changed] = affected

  def _check_for_changed_files(self):
    '''
    Check the dictinary with changed files and notify the masters about changes.
    '''
    new_affected = list()
    for changed, affected in self._changed_files.items():
      for (muri, lfile) in affected:
        if not (muri, lfile) in self.__in_question:
          self.__in_question.add((muri, lfile))
          new_affected.append((muri, lfile))
    # if there are no question to reload the launch file -> ask
    if new_affected:
      choices = dict()
      for (muri, lfile) in new_affected:
        master = self.getMaster(muri)
        if not master is None:
          master.launchfile = lfile
          choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
      cfgs, ok = SelectDialog.getValue('Reload configurations?',
                                   '<b>%s</b> was changed.<br>Select affected configurations to reload:'%', '.join([os.path.basename(f) for f in self._changed_files.keys()]), choices.keys(),
                                   False, True,
                                   ':/icons/crystal_clear_launch_file.png',
                                   self)
      for (muri, lfile) in new_affected:
        self.__in_question.remove((muri, lfile))
      for c in cfgs:
        choices[c][0].launchfiles = choices[c][1]
    self._changed_files.clear()

  def on_configparamfile_changed(self, changed, affected):
    '''
    Signal hander to handle the changes of a configuration file referenced by a paramter value
    @param changed: the changed file
    @type changed: C{str}
    @param affected: the list of tuples with masteruri and launchfile, which are affected by file change
    @type affected: list
    '''
    # create a list of launch files and masters, which are affected by the changed file
    # and are not currently in question
    if self.isActiveWindow():
      self._changed_files_param[changed] = affected
      self._check_for_changed_files_param()
    else:
      self._changed_files_param[changed] = affected

  def _check_for_changed_files_param(self):
    '''
    Check the dictinary with changed files and notify about the transfer of changed file.
    '''
    new_affected = list()
    for changed, affected in self._changed_files_param.items():
      for (muri, lfile) in affected:
        if not (muri, changed) in self.__in_question:
          self.__in_question.add((muri, changed))
          new_affected.append((muri, changed))
    # if there are no question to reload the launch file -> ask
    if new_affected:
      choices = dict()
      for (muri, lfile) in new_affected:
        master = self.getMaster(muri)
        if not master is None:
          master.launchfile = lfile
          choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
      cfgs, ok = SelectDialog.getValue('Transfer configurations?',
                                   'Configuration files referenced by parameter are changed.<br>Select affected configurations for copy to remote host: (don\'t forget to restart the nodes!)', 
                                   choices.keys(), False, True,
                                   ':/icons/crystal_clear_launch_file_transfer.png',
                                   self)
      for (muri, lfile) in new_affected:
        self.__in_question.remove((muri, lfile))
      for c in cfgs:
        host = nm.nameres().getHostname(choices[c][0].masteruri)
        username = choices[c][0].current_user
        self._progress_queue_cfg.add2queue(str(uuid.uuid4()), 
                                       'transfer files to '+str(host),
                                       nm.starter().transfer_files,
                                       (str(host), choices[c][1], False, username))
      self._progress_queue_cfg.start()
    self._changed_files_param.clear()

  def changeEvent(self, event):
    '''
    Check for changed files, if the main gui is activated.
    '''
    QtGui.QMainWindow.changeEvent(self, event)
    self._check_for_changed_files()
    self._check_for_changed_files_param()

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Capabilities handling      %%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_start_nodes(self, masteruri, cfg, nodes):
    if not masteruri is None:
      master = self.getMaster(masteruri)
      master.start_nodes_by_name(nodes, (cfg, ''))

  def on_stop_nodes(self, masteruri, nodes):
    if not masteruri is None:
      master = self.getMaster(masteruri)
      master.stop_nodes_by_name(nodes)

  def on_description_update(self, title, text):
    if self.sender() == self.currentMaster or not isinstance(self.sender(), MasterViewProxy):
      self.ui.descriptionDock.setWindowTitle(title)
      self.ui.descriptionTextEdit.setText(text)
      if text:
        self.ui.descriptionDock.raise_()
      else:
        self.ui.launchDock.raise_()

  def on_description_update_cap(self, title, text):
    self.ui.descriptionDock.setWindowTitle(title)
    self.ui.descriptionTextEdit.setText(text)

  def on_description_anchorClicked(self, url):
    if url.toString().startswith('open_sync_dialog://'):
      self.on_sync_dialog_released(False, str(url.encodedPath()).replace('open_sync_dialog', 'http'), True)
    elif url.toString().startswith('show_all_screens://'):
      master = self.getMaster(str(url.encodedPath()).replace('show_all_screens', 'http'), False)
      if not master is None:
        master.on_show_all_screens()
    elif url.toString().startswith('remove_all_launch_server://'):
      master = self.getMaster(str(url.encodedPath()).replace('remove_all_launch_server', 'http'), False)
      if not master is None:
        master.on_remove_all_launch_server()
    elif url.toString().startswith('topic://'):
      if not self.currentMaster is None:
        self.currentMaster.show_topic_output(url.encodedPath(), False)
    elif url.toString().startswith('topichz://'):
      if not self.currentMaster is None:
        self.currentMaster.show_topic_output(url.encodedPath(), True)
    elif url.toString().startswith('service://'):
      if not self.currentMaster is None:
        self.currentMaster.service_call(url.encodedPath())
    elif url.toString().startswith('unregister_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_unregister_nodes()
    elif url.toString().startswith('start_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_start_clicked()
    elif url.toString().startswith('restart_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_force_start_nodes()
    elif url.toString().startswith('start_node_at_host://'):
      if not self.currentMaster is None:
        self.currentMaster.on_start_nodes_at_host()
    elif url.toString().startswith('kill_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_kill_nodes()
    elif url.toString().startswith('kill_screen://'):
      if not self.currentMaster is None:
        self.currentMaster.on_kill_screens()
    elif url.toString().startswith('copy_log_path://'):
      if not self.currentMaster is None:
        self.currentMaster.on_log_path_copy()
    elif url.toString().startswith('launch://'):
      self._editor_dialog_open([str(url.encodedPath())], '')
    elif url.toString().startswith('reload_globals://'):
      self._reload_globals_at_next_start(str(url.encodedPath()).replace('reload_globals://', ''))
    else:
      QtGui.QDesktopServices.openUrl(url)

  def keyReleaseEvent(self, event):
    '''
    Deletes the selected history launch file.
    '''
    if self.ui.xmlFileView.hasFocus() and event.key() == QtCore.Qt.Key_Delete:
      indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
      for index in indexes:
        pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
        try:
          self.ui.xmlFileView.model().load_history.remove(path)
        except:
          pass
      self.ui.xmlFileView.model().reloadCurrentPath()
    QtGui.QMainWindow.keyReleaseEvent(self, event)

  def image_mouseDoubleClickEvent(self, event):
    '''
    Set the robot image
    '''
    if self.currentMaster:
      try:
        if not os.path.isdir(nm.ROBOTS_DIR):
          os.makedirs(nm.ROBOTS_DIR)
        (fileName, filter) = QtGui.QFileDialog.getOpenFileName(self,
                                                 "Set robot image",
                                                 nm.ROBOTS_DIR,
                                                 "Image files (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.xbm);;All files (*)")
        if fileName and self.__current_master_label_name:
          p = QtGui.QPixmap(fileName)
          p.save(os.path.join(nm.ROBOTS_DIR, self.__current_master_label_name+'.png'))
        if self.__icons.has_key(self.__current_master_label_name):
          del self.__icons[self.__current_master_label_name]
        self._assigne_icon(self.__current_master_label_name)
      except Exception as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Error", 
                          ''.join(['Set robot image for ', str(self.__current_master_label_name), ' failed!']),
                          str(e)).exec_()
        rospy.logwarn("Error while set robot image for %s: %s", str(self.__current_master_label_name), str(e))

  def _on_robot_icon_changed(self, masteruri, path):
    '''
    One of the robot icons was chagned. Update the icon.
    '''
    master = self.getMaster(masteruri, False)
    if master:
      self._assigne_icon(master.mastername, resolve_url(path))

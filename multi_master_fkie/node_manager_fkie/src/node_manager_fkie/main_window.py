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

import os
import time
import sys
import xmlrpclib
import threading

from datetime import datetime

from PySide import QtGui
from PySide import QtCore
from PySide import QtUiTools

import roslib; roslib.load_manifest('node_manager_fkie')
import rospy

import gui_resources
from discovery_listener import MasterListService, MasterStateTopic, MasterStatisticTopic, OwnMasterMonitoring
from update_handler import UpdateHandler
from launch_list_model import LaunchListModel 
from master_view_proxy import MasterViewProxy
from launch_config import LaunchConfig, LaunchConfigException
from capability_table import CapabilityTable
from xml_editor import XmlEditor
from detailed_msg_box import WarningMessageBox

import node_manager_fkie as nm

from master_discovery_fkie.msg import *
from master_discovery_fkie.srv import *


class MainWindow(QtGui.QMainWindow):
  '''
  The class to create the main window of the application.
  '''

  def __init__(self, args=[], parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QMainWindow.__init__(self)
    #self.setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
    #load the UI formular for the main window
    loader = QtUiTools.QUiLoader()
    self.setObjectName('MainWindow')
    self.ui = mainWindow = loader.load(":/forms/MainWindow.ui")
    self.ui.setObjectName('MainUI')
    self.ui.masterInfoFrame.setEnabled(False)
    self.ui.refreshHostButton.clicked.connect(self.on_refresh_master_clicked)
    self.ui.runButton.clicked.connect(self.on_run_node_clicked)
    self.ui.rxconsoleButton.clicked.connect(self.on_show_rxconsole_clicked)
    self.ui.rxgraphButton.clicked.connect(self.on_show_rxgraph_clicked)
    self.ui.syncButton.released.connect(self.on_sync_released)

    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_prop_run.png")
    self.setWindowIcon(self.mIcon)
    self.setWindowTitle("Node Manager")
    self.setCentralWidget(mainWindow)
    self.resize(1024,768)
    
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

    # initialize the view for the discovered ROS master
    from master_list_model import MasterModel
    self.master_model = MasterModel(self.getMasteruri())
    self.ui.masterListView.setModel(self.master_model)
    self.ui.masterListView.setAlternatingRowColors(True)
    self.ui.masterListView.clicked.connect(self.on_master_selection_changed)
    self.ui.masterListView.activated.connect(self.on_master_selection_changed)
    self.ui.masterListView.selectionModel().currentRowChanged.connect(self.on_masterListView_selection_changed)
    self.ui.refreshAllButton.clicked.connect(self.on_all_master_refresh_clicked)
    self.ui.startRobotButton.clicked.connect(self.on_start_robot_clicked)

    # initialize the view for the launch files
    self.ui.xmlFileView.setModel(LaunchListModel())
    self.ui.xmlFileView.setAlternatingRowColors(True)
    self.ui.xmlFileView.activated.connect(self.on_launch_selection_activated)
    self.ui.xmlFileView.selectionModel().selectionChanged.connect(self.on_xmlFileView_selection_changed)
    self.ui.refreshXmlButton.clicked.connect(self.on_refresh_xml_clicked)
    self.ui.editXmlButton.clicked.connect(self.on_edit_xml_clicked)
    self.ui.loadXmlButton.clicked.connect(self.on_load_xml_clicked)
    self.ui.loadXmlAsDefaultButton.clicked.connect(self.on_load_as_default)

    # stores the widget to a 
    self.masters = dict() # masteruri : MasterViewProxy
    self.currentMaster = None # MasterViewProxy
    
    # initialize the class to get the state of discovering of other ROS master
    self._update_handler = UpdateHandler()
    self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
    self._update_handler.error_signal.connect(self.on_master_info_error)
    
    # this monitor class is used, if no master_discovery node is running to get the state of the local ROS master
    self.own_master_monitor = OwnMasterMonitoring()
    self.own_master_monitor.init(22622)
    self.own_master_monitor.state_signal.connect(self.on_master_state_changed)

    # get the name of the service and topic of the discovery node. The name are determine by the message type  of those topics
    self.masterlist_service = masterlist_service = MasterListService()
    masterlist_service.masterlist_signal.connect(self.on_master_list_retrieved)
    masterlist_service.masterlist_err_signal.connect(self.on_master_list_err_retrieved)
    self.state_topic = MasterStateTopic()
    self.state_topic.state_signal.connect(self.on_master_state_changed)
    self.stats_topic = MasterStatisticTopic()
    self.stats_topic.stats_signal.connect(self.on_conn_stats_updated)
    
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
    
    self.ui.tabifyDockWidget(self.ui.launchDock, self.ui.descriptionDock)
    self.ui.launchDock.raise_()
    
    self.default_load_launch = os.path.abspath(args[1]) if len(args) >= 2 else ''
    if self.default_load_launch:
      if os.path.isdir(self.default_load_launch):
        self.ui.xmlFileView.model().setPath(self.default_load_launch)
      elif os.path.isfile(self.default_load_launch):
        self.ui.xmlFileView.model().setPath(os.path.dirname(self.default_load_launch))

    self._subscribe()

    self.editor_dialogs  = dict() # [file] = XmlEditor
    '''@ivar: stores the open XmlEditor '''

    # since the is_local method is threaded for host names, call it to cache the localhost
    nm.is_local("localhost")

    # timer to update the showed update time of the ros state 
    self.master_timecheck_timer = QtCore.QTimer()
    self.master_timecheck_timer.timeout.connect(self.on_master_timecheck)
    self.master_timecheck_timer.start(1000)
    self._refresh_time = time.time()


  def createSlider(self):
    slider = QtGui.QSlider()
    palette = QtGui.QPalette()
    brush = QtGui.QBrush(QtGui.QColor(59, 223, 18))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
    brush = QtGui.QBrush(QtGui.QColor(59, 223, 18))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
    brush = QtGui.QBrush(QtGui.QColor(59, 223, 18))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
    #red
    palette = QtGui.QPalette()
    brush = QtGui.QBrush(QtGui.QColor(212, 0, 0))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
    brush = QtGui.QBrush(QtGui.QColor(212, 0, 0))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
    brush = QtGui.QBrush(QtGui.QColor(212, 0, 0))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)

    slider.setPalette(palette)
    slider.setMinimum(1)
    slider.setMaximum(2)
    slider.setPageStep(1)
    slider.setOrientation(QtCore.Qt.Horizontal)
#    slider.setInvertedAppearance(True)
#    slider.setInvertedControls(True)
#    slider.setTickPosition(QtGui.QSlider.TicksBelow)
    slider.actionTriggered .connect(self.was)
    return slider



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
    self.state_topic.stop()
    self.stats_topic.stop()

  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it. The 'materuri' attribute will be set to the requested value.
    @return: ROS master URI
    @rtype: C{str} or C{None}
    '''
    if not hasattr(self, 'materuri') or self.materuri is None:
      masteruri = nm.masteruri_from_ros()
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
        self.currentMaster = None
        self.stackedLayout.setCurrentIndex(0)
        self.on_master_timecheck()
      self.stackedLayout.removeWidget(self.masters[masteruri])
      self.ui.tabPlace.layout().removeWidget(self.masters[masteruri])
      self.masters[masteruri].setParent(None)
      del self.masters[masteruri]

  def getMaster(self, masteruri):
    '''
    @return: the Widget which represents the master of given ROS master URI. If no
    Widget for given URI is available a new one will be created.
    @rtype: L{MasterViewProxy} 
    '''
    if not self.masters.has_key(masteruri):
      self.masters[masteruri] = MasterViewProxy(masteruri, self)
      self.masters[masteruri].updateHostRequest.connect(self.on_host_update_request)
      self.masters[masteruri].host_description_updated.connect(self.on_host_description_updated)
      self.masters[masteruri].capabilities_update_signal.connect(self.on_capabilities_update)
      self.masters[masteruri].remove_config_signal.connect(self.on_remove_config)
      self.masters[masteruri].description_signal.connect(self.on_description_update)
      self.masters[masteruri].request_xml_editor.connect(self._editor_dialog_open)
      self.stackedLayout.addWidget(self.masters[masteruri])
      if masteruri == self.getMasteruri():
        if self.default_load_launch:
          if os.path.isfile(self.default_load_launch):
            args = list()
            args.append(''.join(['_package:=', str(LaunchConfig.packageName(os.path.dirname(self.default_load_launch))[0])]))
            args.append(''.join(['_launch_file:="', os.path.basename(self.default_load_launch), '"']))
            nm.starter().runNodeWithoutConfig(nm.nameres().getHost(masteruri=masteruri), 'default_cfg_fkie', 'default_cfg', ''.join([str(nm.nameres().getHost(masteruri=masteruri)), '/default_cfg']), args)

    return self.masters[masteruri]

  def on_host_update_request(self, host):
    for key, value in self.masters.items():
      if nm.nameres().getHostname(key) == host and not value.master_state is None:
        self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)
        
  def on_host_description_updated(self, host, descr):
    self.master_model.updateDescription(nm.nameres().getName(host=host), descr)

  def on_capabilities_update(self, host, config_node, descriptions):
    for d in descriptions:
      self.capabilitiesTable.updateCapabilities(host, config_node, d)
    master = self.getMaster(nm.nameres().getUri(host=host))
    self.capabilitiesTable.updateState(host, master.master_info)

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
    result_1 = self.state_topic.registerByROS(self.getMasteruri(), False)
    result_2 = self.stats_topic.registerByROS(self.getMasteruri(), False)
    self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)
    if not result_1 or not result_2:
      self._setLocalMonitoring(True)

  def _setLocalMonitoring(self, on):
    '''
    Enables the local monitoring of the ROS master state and disables the view of
    the discoved ROS master.
    @param on: the enable / disable the local monitoring
    @type on: C{boolean}
    '''
    self.ui.masterListView.setEnabled(not on)
    self.ui.refreshAllButton.setEnabled(not on)
    self.own_master_monitor.pause(not on)
    if on:
      self.ui.masterListView.setToolTip("use 'Start' button to enable the master discovering")
    else:
      self.ui.masterListView.setToolTip('')
    if on:
      # remove discovered ROS master and set the local master to selected
      to_remove = []
      for uri, master in self.masters.items():
        if nm.is_local(nm.nameres().getHostname(uri)):
          self.currentMaster = master
          self.stackedLayout.setCurrentWidget(master)
          self.on_master_timecheck()
        else:
          self.master_model.removeMaster(master.master_state.name)
          to_remove.append(uri)
      for r in to_remove:
        self.removeMaster(r)
      self.ui.masterListView.doItemsLayout()



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
    for m in master_list:
      nm.nameres().add(name=m.name, masteruri=m.uri, host=nm.nameres().getHostname(m.uri))
      master = self.getMaster(m.uri)
      master.master_state = m
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
    if msg.state == master_discovery_fkie.msg.MasterState.STATE_CHANGED:
      nm.nameres().add(name=msg.master.name, masteruri=msg.master.uri, host=nm.nameres().getHostname(msg.master.uri))
      self.getMaster(msg.master.uri).master_state = msg.master
      self.master_model.updateMaster(msg.master)
      self.ui.masterListView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == master_discovery_fkie.msg.MasterState.STATE_NEW:
      nm.nameres().add(name=msg.master.name, masteruri=msg.master.uri, host=nm.nameres().getHostname(msg.master.uri))
      self.getMaster(msg.master.uri).master_state = msg.master
      self.master_model.updateMaster(msg.master)
      self.ui.masterListView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == master_discovery_fkie.msg.MasterState.STATE_REMOVED:
      nm.nameres().remove(name=msg.master.name, masteruri=msg.master.uri, host=nm.nameres().getHostname(msg.master.uri))
      self.master_model.removeMaster(msg.master.name)
      self.ui.masterListView.doItemsLayout()
      self.removeMaster(msg.master.uri)
      if len(self.masters) == 0:
        self._setLocalMonitoring(True)

  def on_master_info_retrieved(self, minfo):
    '''
    Integrate the received master info.
    @param minfo: the ROS master Info
    @type minfo: L{master_discovery_fkie.MasterInfo}
    '''
    rospy.loginfo("MASTERINFO from %s received", minfo.mastername)
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
            if nm.is_local(nm.nameres().getHostname(master.master_info.masteruri)) and new_info:
              has_discovery_service = self.hasDiscoveryService(minfo)
              if not self.own_master_monitor.isPaused() and has_discovery_service:
                self._subscribe()
              elif self.currentMaster is None:
                self.currentMaster = master
                self.stackedLayout.setCurrentWidget(master)
                self.on_master_timecheck()
  
            # update the list view, whether master is synchronized or not
            if master.master_info.masteruri == minfo.masteruri:
              self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
          self.capabilitiesTable.updateState(nm.nameres().getHost(masteruri=minfo.masteruri), minfo)
          self.updateDuplicateNodes()
        except Exception, e:
          rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, str(e))
      # update the buttons, whether master is synchronized or not
      if not self.currentMaster is None and not self.currentMaster.master_info is None:
        self.ui.syncButton.setEnabled(True)
        self.ui.syncButton.setChecked(not self.currentMaster.master_info.getNodeEndsWith('master_sync') is None)
#    cputimes_m = os.times()
#    cputime_m = cputimes_m[0] + cputimes_m[1] - cputime_init_m
#    print "ALL:", cputime_m

  def on_master_info_error(self, masteruri, error):
    if nm.is_local(nm.nameres().getHostname(masteruri)):
      self._setLocalMonitoring(True)

  def on_conn_stats_updated(self, stats):
    '''
    Handle the retrieved connection statistics.
      1. update the ROS Network view
    @param stats: a list with connection statistics
    @type stats: C{[L{master_discovery_fkie.msg.LinkState}]}
    '''
    for stat in stats:
      self.master_model.updateMasterStat(stat.destination, stat.quality)
    self.ui.masterListView.doItemsLayout()


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master info frame         %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_refresh_master_clicked(self):
    if not self.currentMaster is None:
      self._update_handler.requestMasterInfo(self.currentMaster.master_state.uri, self.currentMaster.master_state.monitoruri)
#      self.currentMaster.remove_all_def_configs()

  def on_run_node_clicked(self):
    '''
    Open a dialog to run a ROS node without a configuration
    '''
    from run_dialog import RunDialog
    if not self.currentMaster is None:
      dia = RunDialog(nm.nameres().getHostname(self.currentMaster.masteruri))
      if dia.exec_():
        dia.runSelected()
  
  def on_show_rxconsole_clicked(self):
    if not self.currentMaster is None:
      import os, subprocess
      env = dict(os.environ)
      env["ROS_MASTER_URI"] = str(self.currentMaster.master_state.uri)
      ps = subprocess.Popen(['rxconsole'], env=env)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=ps.wait)
      thread.setDaemon(True)
      thread.start()

  def on_show_rxgraph_clicked(self):
    if not self.currentMaster is None:
      import os, subprocess
      env = dict(os.environ)
      env["ROS_MASTER_URI"] = str(self.currentMaster.master_state.uri)
      ps = subprocess.Popen(['rxgraph'], env=env)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=ps.wait)
      thread.setDaemon(True)
      thread.start()

  def on_sync_released(self):
    '''
    Enable or disable the synchronization of the master cores
    '''
    self.ui.syncButton.setEnabled(False)
    if not self.currentMaster is None:
      if self.ui.syncButton.isChecked():
        try:
          nm.starter().runNodeWithoutConfig(nm.nameres().getHostname(self.currentMaster.masteruri), 'master_sync_fkie', 'master_sync', 'master_sync')
        except (Exception, nm.StartException), e:
          rospy.logwarn("Error while start master_sync for %s: %s", str(self.currentMaster.masteruri), str(e))
          WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                            'Error while start master_sync',
                            str(e)).exec_()
      elif not self.currentMaster.master_info is None:
        node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
        self.currentMaster.stop_nodes([node])

  def on_master_timecheck(self):
    if not self.currentMaster is None:
      master = self.getMaster(self.currentMaster.master_state.uri)
      if not master.master_info is None:
        self.showMasterName(master.master_state.name, self.timestampStr(master.master_info.check_ts), master.master_state.online)
      elif not master.master_state is None:
        self.showMasterName(master.master_state.name, 'Try to get info!!! Currently not received!!!', master.master_state.online)
    else:
      self.showMasterName('No robot selected', None, False)
    if (time.time() - self._refresh_time > 15.0):
      master = self.getMaster(self.getMasteruri())
      if not master is None:
        self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri)
      self._refresh_time = time.time()


  def showMasterName(self, name, timestamp, online=True):
    '''
    Update the view of the info frame.
    '''
    label = self.ui.masternameLabel
    label.setText(''.join(['<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta name="qrichtext" content="1" /><style type="text/css">\np, li { white-space: pre-wrap; }\n</style></head><body style=" font-family:"Ubuntu"; font-size:11pt; font-weight:400; font-style:normal;">\n<p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:14pt; font-weight:600;">',
                           name, 
                           '</span></p>\n<p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:8pt;">', 
                          'updated: ' if not timestamp is None else '', 
                          str(timestamp) if not timestamp is None else ' ', 
                           '</span></p></body></html>'])
                  )
    # load the robot image, if one exists
    if label.isEnabled():
      if QtCore.QFile.exists(''.join([nm.ROBOTS_DIR, name, '.png'])):
        icon = QtGui.QIcon(''.join([nm.ROBOTS_DIR, name, '.png']))
        self.ui.imageLabel.setPixmap(icon.pixmap(label.size()))
        self.ui.imageLabel.setToolTip(''.join(['<html><head></head><body><img src="', nm.ROBOTS_DIR, name, '.png', '" alt="', name,'"></body></html>']))
      else:
        icon = QtGui.QIcon(''.join([':/icons/crystal_clear_miscellaneous.png']))
        self.ui.imageLabel.setPixmap(icon.pixmap(label.size()))
        self.ui.imageLabel.setToolTip('')
    else:
      icon = QtGui.QIcon()
      self.ui.imageLabel.setPixmap(icon.pixmap(label.size()))
      self.ui.imageLabel.setToolTip('')
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
    running_nodes = []
    for uri, m in self.masters.items():
      if m.master_state.online:
#        running_nodes[len(running_nodes):] = m.getRunningNodesIfSync()
        running_nodes[len(running_nodes):] = m.getRunningNodesIfLocal()
    for uri, m in self.masters.items():
      m.markNodesAsDuplicateOf(running_nodes)



  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master list view          %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_master_selection_changed(self, selected):
    '''
    If a master was selected, set the corresponding Widget of the stacked layout
    to the current widget and shows the state of the selected master.
    '''
    si = self.ui.masterListView.selectedIndexes()
    for index in si:
      if index.row() == selected.row():
        item = self.master_model.itemFromIndex(selected)
        if not item is None:
          self.currentMaster = self.getMaster(item.master.uri)
          self.stackedLayout.setCurrentWidget(self.currentMaster)
          self.on_master_timecheck()
          if not self.currentMaster.master_info is None:
            node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
            self.ui.syncButton.setEnabled(True)
            self.ui.syncButton.setChecked(not node is None)
          else:
            self.ui.syncButton.setEnabled(False)
          return
  
  def on_masterListView_selection_changed(self, selected, deselected):
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
    self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)

  def on_start_robot_clicked(self):
    '''
    Tries to start the master_discovery node on the machine requested by a dialog.
    '''
    # get the history list
    history = self._getHostHistory()
    host, result = QtGui.QInputDialog.getItem(self, self.tr("Launch master_discovery on"),
                                      self.tr("Hostname / address:"), list(['localhost'] + history), 0,
                                      True)
    if result and host:
      try:
        progressDialog = QtGui.QProgressDialog('Start discovering on %s'%host, 'wait', 0, 2)
        progressDialog.setWindowModality(QtCore.Qt.WindowModal)
        progressDialog.show()
        nm.starter().runNodeWithoutConfig(host, 'master_discovery_fkie', 'master_discovery', 'master_discovery')
#        progressDialog.hide()
        if host != 'localhost':
          # insert the host into first place
          try:
            history.remove(host)
          except:
            pass
          history.insert(0, host)
          self._storeHostHistory(history)
      except (Exception, nm.StartException), e:
        rospy.logwarn("Error while start master_discovery for %s: %s", str(host), str(e))
        WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                          'Error while start master_discovery',
                          str(e)).exec_()

  def _getHostHistory(self):
    '''
    Read the history of the entered robots from the file stored in ROS_HOME path.
    @return: the list with robot names
    @rtype: C{[str]}
    '''
    result = list()
    historyFile = ''.join([nm.CFG_PATH, 'host.history'])
    if os.path.isfile(historyFile):
      with open(historyFile, 'r') as f:
        line = f.readline()
        while line:
          if line:
            result.append(line.strip())
          line = f.readline()
      f.closed
    return result

  def _storeHostHistory(self, hosts):
    '''
    Saves the list of hosts to history. The existing history will be replaced!
    @param hosts: the list with robot names
    @type hosts: C{[str]}
    '''
    if not os.path.isdir(nm.CFG_PATH):
      os.makedirs(nm.CFG_PATH)
    with open(''.join([nm.CFG_PATH, 'host.history']), 'w') as f:
      for host in hosts:
        f.write(''.join([host, '\n']))
    f.closed


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of the launch file view      %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_launch_selection_activated(self, activated):
    '''
    Tries to load the launch file, if one was activated.
    '''
    item, path, id = activated.model().items[activated.row()]
    file = activated.model().getFilePath(item)
    if not file is None:
      self.loadLaunchFile(path)

  def on_xmlFileView_selection_changed(self, selected, deselected):
    '''
    On selection of a launch file, the buttons are enabled otherwise disabled.
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      isfile = self.ui.xmlFileView.model().isLaunchFile(index.row())
      self.ui.editXmlButton.setEnabled(isfile)
      self.ui.loadXmlButton.setEnabled(isfile)
      self.ui.loadXmlAsDefaultButton.setEnabled(isfile)

  def on_refresh_xml_clicked(self):
    '''
    Reload the current path.
    '''
    self.ui.xmlFileView.model().reloadCurrentPath()
    self.ui.editXmlButton.setEnabled(False)
    self.ui.loadXmlButton.setEnabled(False)
    self.ui.loadXmlAsDefaultButton.setEnabled(False)
    
  def on_edit_xml_clicked(self):
    '''
    Opens an XML editor to edit the launch file. 
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().getFilePath(pathItem)
      if not path is None:
        self._editor_dialog_open([path], '')

  def _editor_dialog_open(self, files, search_text):
    if files:
      path = files[0]
      if self.editor_dialogs.has_key(path):
        last_path = files[-1]
        self.editor_dialogs[path].on_load_request(last_path, search_text)
        self.editor_dialogs[path].raise_()
        self.editor_dialogs[path].activateWindow()
      else:
        editor = XmlEditor(files, search_text, self)
        self.editor_dialogs[path] = editor
        editor.finished_signal.connect(self._editor_dialog_closed)
        editor.show()

  def _editor_dialog_closed(self, files):
    if self.editor_dialogs.has_key(files[0]):
      del self.editor_dialogs[files[0]]

  def on_load_xml_clicked(self):
    '''
    Tries to load the selected launch file. The button is only enabled and this
    method is called, if the button was enabled by on_launch_selection_clicked()
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().getFilePath(pathItem)
      if not path is None:
        self.loadLaunchFile(path)

  def loadLaunchFile(self, path, force_as_default=False):
    '''
    Load the launch file. A ROS master mast be selected first.
    @param path: the path of the launch file.
    @type path: C{str}
    '''
    rospy.loginfo("LOAD the launch file: %s", path)
    master_proxy = self.stackedLayout.currentWidget()
    if isinstance(master_proxy, MasterViewProxy):
      cursor = self.cursor()
      self.setCursor(QtCore.Qt.WaitCursor)
      self.ui.xmlFileView.setEnabled(False)
      self.ui.xmlFileView.model().add2LoadHistory(path)
      QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
      #todo: except errors on determination of the defaul_cfg name
      key_mod = QtGui.QApplication.keyboardModifiers()
      if (key_mod & QtCore.Qt.ControlModifier) or force_as_default:
        args = list()
        args.append(''.join(['_package:=', str(LaunchConfig.packageName(os.path.dirname(path))[0])]))
        args.append(''.join(['_launch_file:="', os.path.basename(path), '"']))
        nm.starter().runNodeWithoutConfig(nm.nameres().getHost(masteruri=master_proxy.masteruri), 'default_cfg_fkie', 'default_cfg', ''.join([str(nm.nameres().getHost(masteruri=master_proxy.masteruri)), '/default_cfg']), args)
      else:
        master_proxy.launchfiles = path
        # update the duplicate nodes
        self.updateDuplicateNodes()
      self.ui.xmlFileView.setEnabled(True)
      self.setCursor(cursor)
    else:
      QtGui.QMessageBox.information(self, "Load of launch file",
                                    "Select a master first!", )

  def on_load_as_default(self):
    '''
    Tries to load the selected launch file as default configuration. The button 
    is only enabled and this method is called, if the button was enabled by 
    on_launch_selection_clicked()
    '''
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().getFilePath(pathItem)
      if not path is None:
        rospy.loginfo("LOAD the launch file as default: %s", path)
        self.loadLaunchFile(path, True)


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Capabilities handling      %%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_start_nodes(self, host, cfg, nodes):
    master = self.getMaster(nm.nameres().getUri(host=host))
    master.start_nodes_by_name(nodes, (cfg, ''))
  
  def on_stop_nodes(self, host, nodes):
    master = self.getMaster(nm.nameres().getUri(host=host))
    master.stop_nodes_by_name(nodes)
    
  def on_description_update(self, title, text):
    self.ui.descriptionDock.setWindowTitle(title)
    self.ui.descriptionTextEdit.setText(text)
    if text:
      self.ui.descriptionDock.raise_()

  def on_description_update_cap(self, title, text):
    self.ui.descriptionDock.setWindowTitle(title)
    self.ui.descriptionTextEdit.setText(text)

  

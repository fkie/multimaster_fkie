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

import time
import sys
import xmlrpclib

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
import node_manager_fkie as nm

from master_discovery_fkie.msg import *
from master_discovery_fkie.srv import *


class MainWindow(QtGui.QMainWindow):
  '''
  The class to create the main window of the application.
  '''

  def __init__(self, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QMainWindow.__init__(self)
    self.setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
    #load the UI formular for the main window
    loader = QtUiTools.QUiLoader()
    self.ui = mainWindow = loader.load(":/forms/MainWindow.ui")
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
    self.stackedLayout.addWidget(QtGui.QWidget())
    self.ui.tabLayout = QtGui.QVBoxLayout(self.ui.tabPlace)
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

    # stores the widget to a 
    self.masters = dict() # masteruri : MasterViewProxy
    self.currentMaster = None # MasterViewProxy
    
    # initialize the class to get the state of discovering of other ROS master
    self._update_handler = UpdateHandler()
    self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
    
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
    
    if not masterlist_service.retrieveMasterList(self.getMasteruri(), False):
      self._setLocalMonitoring(True)
    else:
      self._subscribe()

    self.show_ros_names = True
    self.shortcut_toggle_names = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+N", "toggle the view of the names")), self)
    self.shortcut_toggle_names.activated.connect(self.on_toggle_name_view)


    # timer to update the showed update time of the ros state 
    self.master_timecheck_timer = QtCore.QTimer()
    self.master_timecheck_timer.timeout.connect(self.on_master_timecheck)
    self.master_timecheck_timer.start(1000)

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
    return self.materuri

  def removeMaster(self, masteruri):
    '''
    Removed master with given master URI from the list.
    @param masteruri: the URI of the ROS master
    @type masteruri: C{str}
    '''
    if self.masters.has_key(masteruri):
      self.currentMaster = None
      self.stackedLayout.setCurrentIndex(0)
      self.ui.masterInfoFrame.setEnabled(False)
      self.on_master_timecheck()
      self.stackedLayout.removeWidget(self.masters[masteruri])
      del self.masters[masteruri]
    #todo update the tab View?

  def getMaster(self, masteruri):
    '''
    @return: the Widget which represents the master of given ROS master URI. If no
    Widget for given URI is available a new one will be created.
    @rtype: L{MasterViewProxy} 
    '''
    if not self.masters.has_key(masteruri):
      self.masters[masteruri] = MasterViewProxy(masteruri)
      self.masters[masteruri].updateHostRequest.connect(self.on_host_update_request)
      self.masters[masteruri].host_description_updated.connect(self.on_host_description_updated)
      self.stackedLayout.addWidget(self.masters[masteruri])
    return self.masters[masteruri]

  def on_host_update_request(self, host):
    for key, value in self.masters.items():
      if nm.nameres().getHostname(key) == host and not value.master_state is None:
        self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)
        
  def on_host_description_updated(self, host, descr):
    self.master_model.updateDescription(nm.nameres().getName(host=host), descr)


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%            Handling of local monitoring            %%%%%%%%
  #%%%%%%%%%%%%%  (Backup, if no master_discovery node is running)  %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _subscribe(self):
    '''
    Try to subscribe to the topics of the master_discovery node. If it fails, the
    own local monitoring of the ROS master state will be enabled.
    '''
#    self.masterlist_service.retrieveMasterList(self.getMasteruri())
    result = self.state_topic.registerByROS(self.getMasteruri(), False)
    result = self.stats_topic.registerByROS(self.getMasteruri(), False)
    result = self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)
    if not result:
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

  def on_master_list_err_retrieved(self, str):
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

  def on_master_list_retrieved(self, master_list):
    '''
    Handle the retrieved list with ROS master.
      1. update the ROS Network view
    @param master_list: a list with ROS masters
    @type master_list: C{[L{master_discovery_fkie.msg.MasterState}]}
    '''
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
      self.ui.masternameLabel.setEnabled(True)
      self.getMaster(msg.master.uri).master_state = msg.master
      self.master_model.updateMaster(msg.master)
      self.ui.masterListView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == master_discovery_fkie.msg.MasterState.STATE_NEW:
      nm.nameres().add(name=msg.master.name, masteruri=msg.master.uri, host=nm.nameres().getHostname(msg.master.uri))
      self.ui.masternameLabel.setEnabled(True)
      self.getMaster(msg.master.uri).master_state = msg.master
      self.master_model.updateMaster(msg.master)
      self.ui.masterListView.doItemsLayout()
      self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
    if msg.state == master_discovery_fkie.msg.MasterState.STATE_REMOVED:
      nm.nameres().remove(name=msg.master.name, masteruri=msg.master.uri, host=nm.nameres().getHostname(msg.master.uri))
      self.ui.masternameLabel.setEnabled(False)
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
    if self.masters.has_key(minfo.masteruri):
      for uri, master in self.masters.items():
        try:
          # check for running discovery service
          if nm.is_local(nm.nameres().getHostname(minfo.masteruri)) and (master.master_info is None or master.master_info.timestamp < minfo.timestamp):
            has_discovery_service = self.hasDiscoveryService(minfo)
            if not self.own_master_monitor.isPaused() and has_discovery_service:
              self._setLocalMonitoring(False)
              self._subscribe()
            elif not has_discovery_service:
              self._setLocalMonitoring(True)
              self.currentMaster = master
              self.stackedLayout.setCurrentWidget(master)
              self.ui.masterInfoFrame.setEnabled(True)
              self.on_master_timecheck()
          master.master_info = minfo
          if not master.master_info is None and master.master_info.masteruri == minfo.masteruri:
            self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
        except Exception, e:
          rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, str(e))
          
      if not self.currentMaster is None and not self.currentMaster.master_info is None:
        self.ui.syncButton.setEnabled(True)
        self.ui.syncButton.setChecked(not self.currentMaster.master_info.getNodeEndsWith('master_sync') is None)

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
      self.currentMaster.remove_all_def_configs()

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
      subprocess.Popen(['rxconsole'], env=env)

  def on_show_rxgraph_clicked(self):
    if not self.currentMaster is None:
      import os, subprocess
      env = dict(os.environ)
      env["ROS_MASTER_URI"] = str(self.currentMaster.master_state.uri)
      subprocess.Popen(['rxgraph'], env=env)

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
          QtGui.QMessageBox.warning(None, 'Error while start master_sync',
                                    str(e),
                                    QtGui.QMessageBox.Ok)
      elif not self.currentMaster.master_info is None:
        node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
        self.currentMaster.stop_node(node)

  def on_master_timecheck(self):
    if not self.currentMaster is None:
      master = self.getMaster(self.currentMaster.master_state.uri)
      if not master.master_info is None:
        self.showMasterName(self.currentMaster.master_state.name, self.timestampStr(master.master_info.check_ts), self.currentMaster.master_state.online)
    else:
      self.showMasterName('No robot selected', None, False)


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
          self.ui.masterInfoFrame.setEnabled(True)
          self.ui.masternameLabel.setEnabled(True)
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
    self.masterlist_service.retrieveMasterList(self.getMasteruri())

  def on_start_robot_clicked(self):
    '''
    Tries to start the master_discovery node on the machine requested by a dialog.
    '''
    host, result = QtGui.QInputDialog.getText(self, self.tr("Launch master_discovery on"),
                                      self.tr("Hostname / address:"), QtGui.QLineEdit.Normal,
                                      'localhost')
    if result and host:
      try:
        nm.starter().runNodeWithoutConfig(host, 'master_discovery_fkie', 'master_discovery', 'master_discovery')
      except (Exception, nm.StartException), e:
        rospy.logwarn("Error while start master_discovery for %s: %s", str(host), str(e))
        QtGui.QMessageBox.warning(None, 'Error while start master_discovery',
                                  str(e),
                                  QtGui.QMessageBox.Ok)

#  def on_clear_master_selection(self):
#    if not self.ui.masterListView.selectionModel() is None:
#      self.ui.masterListView.selectionModel().clear()
#    self.ui.masterInfoFrame.setEnabled(False)
##    self.ui.closeNodeViewButton.setEnabled(False)
##    self.ui.tabWidget.setEnabled(False)
#    self.currentMaster = None
#    self.ui.tabPlace.layout().clear()
#    self.ui.nodesView.setModel(self.empty_node_tree_model)
#    self.ui.nodesView.model().setSourceModel(self.empty_node_tree_model)


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
      rospy.loginfo("LOAD the launch file: %s", path)
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

  def on_refresh_xml_clicked(self):
    '''
    Reload the current path.
    '''
    self.ui.xmlFileView.model().reloadCurrentPath()
    self.ui.editXmlButton.setEnabled(False)
    self.ui.loadXmlButton.setEnabled(False)
    
  def on_edit_xml_clicked(self):
    '''
    Opens an XML editor to edit the launch file. 
    '''
    from xml_editor import XmlEditor
    indexes = self.ui.xmlFileView.selectionModel().selectedIndexes()
    for index in indexes:
      pathItem, path, pathId = self.ui.xmlFileView.model().items[index.row()]
      path = self.ui.xmlFileView.model().getFilePath(pathItem)
      if not path is None:
        self.editor = XmlEditor([path], '', self)
        self.editor.show()
    
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
        rospy.loginfo("LOAD the launch file: %s", path)
        self.loadLaunchFile(path)

  def loadLaunchFile(self, path):
    '''
    Load the launch file. A ROS master mast be selected first.
    @param path: the path of the launch file.
    @type path: C{str}
    '''
    master_proxy = self.stackedLayout.currentWidget()
    if isinstance(master_proxy, MasterViewProxy):
      cursor = self.cursor()
      self.setCursor(QtCore.Qt.WaitCursor)
      self.ui.xmlFileView.setEnabled(False)
      QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
      master_proxy.launchfiles = path
      self.ui.xmlFileView.setEnabled(True)
      self.setCursor(cursor)
    else:
      QtGui.QMessageBox.information(self, "Load of launch file",
                                    "Select a master first!", )

  def on_toggle_name_view(self):
    self.show_ros_names = not self.show_ros_names
    for uri, m in self.masters.items():
      m.show_ros_names(self.show_ros_names)

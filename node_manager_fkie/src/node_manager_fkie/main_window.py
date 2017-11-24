# encoding: utf-8
#
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

from datetime import datetime
from multimaster_msgs_fkie.msg import MasterState
from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import QFile, QPoint, QSize, Qt, QTimer, Signal
from python_qt_binding.QtGui import QDesktopServices, QIcon, QKeySequence, QPixmap
from python_qt_binding.QtGui import QPalette, QColor
import getpass
import os
import roslib
import rospy
import socket
import time
import uuid
import xmlrpclib

from master_discovery_fkie.common import get_hostname, resolve_url, subdomain

import node_manager_fkie as nm

from .capability_table import CapabilityTable
from .common import masteruri_from_ros, package_name, utf8
from .detailed_msg_box import MessageBox
from .discovery_listener import MasterListService, MasterStateTopic, MasterStatisticTopic, OwnMasterMonitoring
from .editor import Editor
from .launch_config import LaunchConfig  # , LaunchConfigException
from .launch_files_widget import LaunchFilesWidget
from .log_widget import LogWidget
from .master_list_model import MasterModel
from .master_view_proxy import MasterViewProxy
from .menu_rqt import MenuRqt
from .network_discovery_dialog import NetworkDiscoveryDialog
from .parameter_dialog import ParameterDialog
from .profile_widget import ProfileWidget
from .progress_queue import ProgressQueue  # , ProgressThread
from .screen_handler import ScreenHandler
from .select_dialog import SelectDialog
from .settings_widget import SettingsWidget
from .sync_dialog import SyncDialog
from .update_handler import UpdateHandler


try:
    from python_qt_binding.QtGui import QApplication, QFileDialog, QMainWindow, QStackedLayout, QWidget
    from python_qt_binding.QtGui import QShortcut, QVBoxLayout, QColorDialog, QDialog, QRadioButton
except:
    from python_qt_binding.QtWidgets import QApplication, QFileDialog, QMainWindow, QStackedLayout, QWidget
    from python_qt_binding.QtWidgets import QShortcut, QVBoxLayout, QColorDialog, QDialog, QRadioButton


try:
    import gui_resources
except:
    print "no gui resources :-/"

# from python_qt_binding import QtUiTools
try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    DIAGNOSTICS_AVAILABLE = True
except:
    import sys
    print >> sys.stderr, "Cannot import 'diagnostic_msgs', feature disabled."
    DIAGNOSTICS_AVAILABLE = False


class MainWindow(QMainWindow):
    '''
    The class to create the main window of the application.
    '''
    DELAYED_NEXT_REQ_ON_ERR = 5.0

    if DIAGNOSTICS_AVAILABLE:
        diagnostics_signal = Signal(DiagnosticStatus)
    '''@ivar: the signal is emitted if a message on topic nm_notifier was
  reiceved (DiagnosticStatus)'''

    def __init__(self, files=[], restricted_to_one_master=False, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QMainWindow.__init__(self)
        self.default_load_launch = os.path.abspath(resolve_url(files[0])) if files else ''
        self.default_profile_load = os.path.isfile(self.default_load_launch) and self.default_load_launch.endswith('.nmprofile')
        restricted_to_one_master = False
        self._finished = False
        self._history_selected_robot = ''
        self.__icons = {'empty': (QIcon(), ''),
                        'default_pc': (QIcon(':/icons/crystal_clear_miscellaneous.png'), ':/icons/crystal_clear_miscellaneous.png'),
                        'log_warning': (QIcon(':/icons/crystal_clear_warning.png'), ':/icons/crystal_clear_warning.png')
                        }  # (masnter name : (QIcon, path))
        self.__current_icon = None
        self.__current_master_label_name = None
        self._changed_files = dict()
        self._changed_binaries = dict()
        self._changed_files_param = dict()
        self._syncs_to_start = []  # hostnames
        self._accept_next_update = False
        # self.setAttribute(Qt.WA_AlwaysShowToolTips, True)
        # setup main window frame
        self.setObjectName('MainWindow')
#    self = mainWindow = QMainWindow()
#    self = mainWindow = loader.load(":/forms/MainWindow.ui")
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MainWindow.ui')
        loadUi(ui_file, self)
        self.setObjectName('MainUI')
        self.user_frame.setVisible(False)
        self._add_user_to_combo(getpass.getuser())
        self.userComboBox.editTextChanged.connect(self.on_user_changed)
        self.masterInfoFrame.setEnabled(False)
        self.infoButton.clicked.connect(self.on_info_clicked)
        self.setTimeButton.clicked.connect(self.on_set_time_clicked)
        self.refreshHostButton.clicked.connect(self.on_refresh_master_clicked)
        self.masterLogButton.clicked.connect(self.on_master_log_clicked)
        self.runButton.clicked.connect(self.on_run_node_clicked)
        self.syncButton.released.connect(self.on_sync_dialog_released)

        menu_rqt = MenuRqt(self.rqtButton)
        menu_rqt.start_rqt_plugin_signal.connect(self.on_rqt_plugin_start)

        pal = self.expert_tab.palette()
        self._default_color = pal.color(QPalette.Window)
        self._set_custom_colors()

        # setup settings widget
        self.profiler = ProfileWidget(self, self)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.profiler)
        self.settings_dock = SettingsWidget(self)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.settings_dock)
        # setup logger widget
        self.log_dock = LogWidget(self)
        self.log_dock.added_signal.connect(self._on_log_added)
        self.log_dock.cleared_signal.connect(self._on_log_cleared)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.logButton.clicked.connect(self._on_log_button_clicked)
        # setup the launch files view
        self.launch_dock = LaunchFilesWidget(self)
        self.launch_dock.load_signal.connect(self.on_load_launch_file)
        self.launch_dock.load_profile_signal.connect(self.profiler.on_load_profile_file)
        self.launch_dock.load_as_default_signal.connect(self.on_load_launch_as_default)
        self.launch_dock.edit_signal.connect(self.on_launch_edit)
        self.launch_dock.transfer_signal.connect(self.on_launch_transfer)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.launch_dock)

        self.mIcon = QIcon(":/icons/crystal_clear_prop_run.png")
        self.setWindowIcon(self.mIcon)
        self.setWindowTitle("Node Manager")
#    self.setCentralWidget(mainWindow)

        # init the stack layout which contains the information about different ros master
        self.stackedLayout = QStackedLayout()
        self.stackedLayout.setObjectName('stackedLayout')
        emptyWidget = QWidget()
        emptyWidget.setObjectName('emptyWidget')
        self.stackedLayout.addWidget(emptyWidget)
        self.tabWidget.currentChanged.connect(self.on_currentChanged_tab)
        self.tabLayout = QVBoxLayout(self.tabPlace)
        self.tabLayout.setObjectName("tabLayout")
        self.tabLayout.setContentsMargins(0, 0, 0, 0)
        self.tabLayout.addLayout(self.stackedLayout)

        # initialize the progress queue
        self._progress_queue = ProgressQueue(self.progressFrame, self.progressBar, self.progressCancelButton, 'Network')
        self._progress_queue_sync = ProgressQueue(self.progressFrame_sync, self.progressBar_sync, self.progressCancelButton_sync, 'Sync')

        # initialize the view for the discovered ROS master
        self.master_model = MasterModel(self.getMasteruri())
        self.master_model.sync_start.connect(self.on_sync_start)
        self.master_model.sync_stop.connect(self.on_sync_stop)
        self.masterTableView.setModel(self.master_model)
        self.master_model.parent_view = self.masterTableView
#    self.masterTableView.setAlternatingRowColors(True)
#    self.masterTableView.clicked.connect(self.on_master_table_clicked)
#    self.masterTableView.pressed.connect(self.on_master_table_pressed)
        self.masterTableView.activated.connect(self.on_master_table_activated)
        sm = self.masterTableView.selectionModel()
        sm.currentRowChanged.connect(self.on_masterTableView_selection_changed)
        for i, (_, width) in enumerate(MasterModel.header):  # _:=name
            self.masterTableView.setColumnWidth(i, width)
        self.refreshAllButton.clicked.connect(self.on_all_master_refresh_clicked)
        self.discoveryButton.clicked.connect(self.on_discover_network_clicked)
        self.startRobotButton.clicked.connect(self.on_start_robot_clicked)

        # stores the widget to a
        self.masters = dict()  # masteruri : MasterViewProxy
        self.currentMaster = None  # MasterViewProxy
        self._close_on_exit = True

        nm.filewatcher().config_changed.connect(self.on_configfile_changed)
        nm.filewatcher().binary_changed.connect(self.on_binaryfile_changed)
        nm.file_watcher_param().config_changed.connect(self.on_configparamfile_changed)
        self.__in_question = set()

        ############################################################################
        self.capabilitiesTable = CapabilityTable(self.capabilities_tab)
        self.capabilitiesTable.setObjectName("capabilitiesTable")
        self.capabilitiesTable.start_nodes_signal.connect(self.on_start_nodes)
        self.capabilitiesTable.stop_nodes_signal.connect(self.on_stop_nodes)
        self.capabilitiesTable.description_requested_signal.connect(self.on_description_update_cap)
        self.capabilities_tab.layout().addWidget(self.capabilitiesTable)

        self.descriptionTextEdit.setOpenLinks(False)
        self.descriptionTextEdit.anchorClicked.connect(self.on_description_anchorClicked)
        self._shortcut_copy = QShortcut(QKeySequence(self.tr("Ctrl+Shift+C", "copy selected description")), self.descriptionTextEdit)
        self._shortcut_copy.activated.connect(self.descriptionTextEdit.copy)

        self.tabifyDockWidget(self.launch_dock, self.descriptionDock)
        self.tabifyDockWidget(self.launch_dock, self.settings_dock)
        self.tabifyDockWidget(self.launch_dock, self.helpDock)
        self.launch_dock.raise_()
        self.helpDock.setWindowIcon(QIcon(':icons/crystal_clear_helpcenter.png'))

        flags = self.windowFlags()
        self.setWindowFlags(flags | Qt.WindowContextHelpButtonHint)

        if self.default_load_launch:
            if os.path.isdir(self.default_load_launch):
                self.launch_dock.launchlist_model.setPath(self.default_load_launch)
            elif os.path.isfile(self.default_load_launch):
                self.launch_dock.launchlist_model.setPath(os.path.dirname(self.default_load_launch))

        self._discover_dialog = None
        self.restricted_to_one_master = restricted_to_one_master
        if restricted_to_one_master:
            self.syncButton.setEnabled(False)
            self.refreshAllButton.setEnabled(False)
            self.discoveryButton.setEnabled(False)
            self.startRobotButton.setEnabled(False)

        self._sync_dialog = SyncDialog()
        self._shortcut_focus = QShortcut(QKeySequence(self.tr("Ctrl+Shift+F", "switch to next focus area")), self)
        self._shortcut_focus.activated.connect(self._show_section_menu)

        self.editor_dialogs = dict()  # [file] = Editor
        '''@ivar: stores the open Editor '''

        self.simTimeLabel.setVisible(False)
        self.launchServerLabel.setVisible(False)

        # since the is_local method is threaded for host names, call it to cache the localhost
        nm.is_local("localhost")

        # set the help text
        try:
            from docutils import examples
            with file(nm.settings().HELP_FILE) as f:
                self.textBrowser.setText(examples.html_body(utf8(f.read())))
        except:
            import traceback
            msg = "Error while generate help: %s" % traceback.format_exc(2)
            rospy.logwarn(msg)
            self.textBrowser.setText(msg)

        try:
            ScreenHandler.testScreen()
        except Exception as e:
            rospy.logerr("No SCREEN available! You can't launch nodes.")
#      MessageBox.warning(self, "No SCREEN",
#                        "No SCREEN available! You can't launch nodes.",
#                        '%s'%utf8(e))

        self.imageLabel.mouseDoubleClickEvent = self.image_mouseDoubleClickEvent
        self.masternameLabel.mouseDoubleClickEvent = self.mastername_mouseDoubleClickEvent

        try:
            self.readSettings()
            self.launch_dock.raise_()
        except Exception as e:
            rospy.logwarn("Error while read settings: %s" % e)
        # setup the hide button, which hides the docks on left side
        docks = self._dock_widget_in(Qt.LeftDockWidgetArea, only_visible=True)
        if not docks:
            self.hideDocksButton.toggle()
            self.on_hide_docks_toggled(True)
        self.hideDocksButton.clicked.connect(self.on_hide_docks_toggled)

        # =============================
        # Initialize the update handler
        # =============================

        # initialize the class to get the state of discovering of other ROS master
        self._update_handler = UpdateHandler()
        self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
        self._update_handler.master_errors_signal.connect(self.on_master_errors_retrieved)
        self._update_handler.timediff_signal.connect(self.on_master_timediff_retrieved)
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

        # timer to update the showed update time of the ros state
        self.master_timecheck_timer = QTimer()
        self.master_timecheck_timer.timeout.connect(self.on_master_timecheck)
        self.master_timecheck_timer.start(1000)
        self._refresh_time = time.time()
        self._last_time_view_update = time.time()

        self._con_tries = dict()
        self._subscribe()
        if DIAGNOSTICS_AVAILABLE:
            self._sub_extended_log = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._callback_diagnostics)
        self.launch_dock.launchlist_model.reloadPackages()
        self._select_index = 0
        self._shortcut_restart_nodes = QShortcut(QKeySequence(self.tr("Ctrl+R", "restart selected nodes")), self)
        self._shortcut_restart_nodes.activated.connect(self._restart_nodes)

    def _dock_widget_in(self, area=Qt.LeftDockWidgetArea, only_visible=False):
        result = []
        docks = [self.launch_dock, self.descriptionDock, self.helpDock, self.networkDock]
        for dock in docks:
            if self.dockWidgetArea(dock) == area:
                if not only_visible or (only_visible and dock.isVisibleTo(self)):
                    result.append(dock)
        return result

    def _on_log_button_clicked(self):
        self.log_dock.setVisible(not self.log_dock.isVisible())

    def _on_log_added(self, info, warn, err, fatal):
        self.logButton.setEnabled(True)

    def _on_log_cleared(self):
        self.logButton.setIcon(self.__icons['log_warning'][0])
        self.logButton.setText('')
        self.logButton.setEnabled(False)

    def on_hide_docks_toggled(self, checked):
        if self.dockWidgetArea(self.launch_dock) == Qt.LeftDockWidgetArea:
            self.launch_dock.setVisible(not checked)
        if self.dockWidgetArea(self.descriptionDock) == Qt.LeftDockWidgetArea:
            self.descriptionDock.setVisible(not checked)
        if self.dockWidgetArea(self.helpDock) == Qt.LeftDockWidgetArea:
            self.helpDock.setVisible(not checked)
        if self.dockWidgetArea(self.networkDock) == Qt.LeftDockWidgetArea:
            self.networkDock.setVisible(not checked)
        if self.dockWidgetArea(self.settings_dock) == Qt.LeftDockWidgetArea:
            self.settings_dock.setVisible(not checked)
        self.hideDocksButton.setArrowType(Qt.RightArrow if checked else Qt.LeftArrow)

    def on_currentChanged_tab(self, index):
        pass
#    if index == self.tabWidget.widget(0):
#      self.networkDock.show()
#      self.launch_dock.show()
#    else:
#      self.networkDock.hide()
#      self.launch_dock.hide()

    def readSettings(self):
        if nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            self._history_selected_robot = settings.value("selected_robot", '')
            settings.beginGroup("mainwindow")
            maximized = settings.value("maximized", 'false') == 'true'
            if maximized:
                self.showMaximized()
            else:
                self.resize(settings.value("size", QSize(1024, 720)))
                self.move(settings.value("pos", QPoint(0, 0)))
            try:
                self.restoreState(settings.value("window_state"))
            except:
                pass
            settings.endGroup()

    def storeSetting(self):
        if nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup("mainwindow")
            settings.setValue("size", self.size())
            settings.setValue("pos", self.pos())
            settings.setValue("maximized", self.isMaximized())
            settings.setValue("window_state", self.saveState())
            settings.endGroup()

    def closeEvent(self, event):
        # ask to close nodes on exit
        if self._close_on_exit and nm.settings().confirm_exit_when_closing:
            masters = [uri for uri, m in self.masters.items() if m.online]
            res = SelectDialog.getValue('Stop nodes?', "Select masters where to stop:",
                                        masters, False, False, '', self,
                                        select_if_single=False,
                                        checkitem1="don't show this dialog again")
            masters2stop, self._close_on_exit = res[0], res[1]
            nm.settings().confirm_exit_when_closing = not res[2]
            if self._close_on_exit:
                self._on_finish = True
                self._stop_local_master = None
                for uri in masters2stop:
                    try:
                        m = self.masters[uri]
                        if m is not None:
                            if m.is_local:
                                self._stop_updating()
                                self._stop_local_master = m
                            m.stop_nodes_by_name(m.getRunningNodesIfLocal(), True, [rospy.get_name(), '/rosout'])
                            if not m.is_local:
                                m.killall_roscore()
                    except Exception as e:
                        rospy.logwarn("Error while stop nodes on %s: %s" % (uri, utf8(e)))
                QTimer.singleShot(200, self._test_for_finish)
            else:
                self._close_on_exit = True
            event.ignore()
        elif self._are_master_in_process():
            QTimer.singleShot(200, self._test_for_finish)
            self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s ...closing...</span>' % self.masternameLabel.text())
            rospy.loginfo("Wait for running processes are finished...")
            event.ignore()
        else:
            try:
                self.storeSetting()
            except Exception as e:
                rospy.logwarn("Error while store settings: %s" % e)
            self.finish()
            QMainWindow.closeEvent(self, event)

    def _are_master_in_process(self):
        for _uri, m in self.masters.items():
            if m.in_process():
                return True
        return False

    def _test_for_finish(self):
        # this method test on exit for running process queues with stopping jobs
        if self._are_master_in_process():
            QTimer.singleShot(200, self._test_for_finish)
            return
        if hasattr(self, '_stop_local_master') and self._stop_local_master is not None:
            self.finish()
            self._stop_local_master.killall_roscore()
            del self._stop_local_master
        self._close_on_exit = False
        self.close()

    def _stop_updating(self):
        if hasattr(self, "_discover_dialog") and self._discover_dialog is not None:
            self._discover_dialog.stop()
        self.masterlist_service.stop()
        self._progress_queue.stop()
        self._progress_queue_sync.stop()
        self._update_handler.stop()
        self.state_topic.stop()
        self.stats_topic.stop()
        self.own_master_monitor.stop()
        self.master_timecheck_timer.stop()
        self.launch_dock.stop()
        self.log_dock.stop()

    def finish(self):
        if not self._finished:
            self._finished = True
            print "Mainwindow finish..."
            self._stop_updating()
            for _, editor in self.editor_dialogs.items():
                editor.close()
            for _, master in self.masters.iteritems():
                master.stop()
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
            _, _, self.materuri = master.getUri(rospy.get_name())  # _:=code, message
            nm.is_local(get_hostname(self.materuri))
        return self.materuri

    def setMasterOnline(self, masteruri, online=True):
        if masteruri in self.masters:
            self.masters[masteruri].online = online

    def removeMaster(self, masteruri):
        '''
        Removed master with given master URI from the list.
        @param masteruri: the URI of the ROS master
        @type masteruri: C{str}
        '''
        if masteruri in self.masters:
            if self.currentMaster is not None and self.currentMaster.masteruri == masteruri:
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
            self.masters[masteruri].save_profile_signal.disconnect()
            if DIAGNOSTICS_AVAILABLE:
                self.diagnostics_signal.disconnect(self.masters[masteruri].append_diagnostic)
            self.stackedLayout.removeWidget(self.masters[masteruri])
            self.tabPlace.layout().removeWidget(self.masters[masteruri])
            for cfg in self.masters[masteruri].default_cfgs:
                self.capabilitiesTable.removeConfig(cfg)
            self.masters[masteruri].setParent(None)
            del self.masters[masteruri]

    def getMaster(self, masteruri, create_new=True):
        '''
        @return: the Widget which represents the master of given ROS master URI. If no
        Widget for given URI is available a new one will be created.
        @rtype: L{MasterViewProxy}
        '''
        if masteruri not in self.masters:
            if not create_new:
                return None
            self.masters[masteruri] = MasterViewProxy(masteruri, self)
            self.masters[masteruri].updateHostRequest.connect(self.on_host_update_request)
            self.masters[masteruri].host_description_updated.connect(self.on_host_description_updated)
            self.masters[masteruri].capabilities_update_signal.connect(self.on_capabilities_update)
            self.masters[masteruri].remove_config_signal.connect(self.on_remove_config)
            self.masters[masteruri].description_signal.connect(self.on_description_update)
            self.masters[masteruri].request_xml_editor.connect(self.on_launch_edit)
            self.masters[masteruri].stop_nodes_signal.connect(self.on_stop_nodes)
            self.masters[masteruri].robot_icon_updated.connect(self._on_robot_icon_changed)
            self.masters[masteruri].save_profile_signal.connect(self.profiler.on_save_profile)
            if DIAGNOSTICS_AVAILABLE:
                self.diagnostics_signal.connect(self.masters[masteruri].append_diagnostic)
            self.stackedLayout.addWidget(self.masters[masteruri])
            if masteruri == self.getMasteruri():
                if self.default_load_launch:
                    try:
                        if os.path.isfile(self.default_load_launch):
                            if self.default_load_launch.endswith('.launch'):
                                args = list()
                                args.append('_package:=%s' % (package_name(os.path.dirname(self.default_load_launch))[0]))
                                args.append('_launch_file:="%s"' % os.path.basename(self.default_load_launch))
                                host = '%s' % nm.nameres().address(masteruri)
                                node_name = roslib.names.SEP.join(['%s' % (nm.nameres().masteruri2name(masteruri)),
                                                                   os.path.basename(self.default_load_launch).replace('.launch', ''),
                                                                   'default_cfg'])
                                self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                                          'start default config @%s' % host,
                                                                          nm.starter().runNodeWithoutConfig,
                                                                          ('%s' % (nm.nameres().mastername(masteruri)), 'default_cfg_fkie',
                                                                           'default_cfg', node_name,
                                                                           args, masteruri, False,
                                                                           self.masters[masteruri].current_user))
                                self.launch_dock.progress_queue.start()
                    except Exception as e:
                        MessageBox.warning(self, "Load default configuration",
                                           'Load default configuration %s failed!' % self.default_load_launch,
                                           '%s' % utf8(e))
        return self.masters[masteruri]

    def on_host_update_request(self, host):
        for key, value in self.masters.items():
            if get_hostname(key) == host and value.master_state is not None:
                self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)

    def on_host_description_updated(self, masteruri, host, descr):
        # self.master_model.updateDescription(nm.nameres().mastername(masteruri, host), descr)
        pass

    def on_capabilities_update(self, masteruri, address, config_node, descriptions):
        for d in descriptions:
            self.capabilitiesTable.updateCapabilities(masteruri, config_node, d)
        if masteruri is not None:
            master = self.getMaster(masteruri)
            self.capabilitiesTable.updateState(masteruri, master.master_info)

    def on_remove_config(self, cfg):
        self.capabilitiesTable.removeConfig(cfg)

# ======================================================================================================================
# Handling of local monitoring
# (Backup, if no master_discovery node is running)
# ======================================================================================================================

    def _subscribe(self):
        '''
        Try to subscribe to the topics of the master_discovery node. If it fails, the
        own local monitoring of the ROS master state will be enabled.
        '''
        if not self.restricted_to_one_master:
            try:
                self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)
            except:
                pass
        else:
            self._setLocalMonitoring(True)

    def _setLocalMonitoring(self, on, discoverer=''):
        '''
        Enables the local monitoring of the ROS master state and disables the view of
        the discoved ROS master.
        @param on: the enable / disable the local monitoring
        @type on: C{boolean}
        '''
        if self.own_master_monitor.is_running() != on:
            self.masterTableView.setEnabled(not on)
            self.refreshAllButton.setEnabled(not on)
            self.own_master_monitor.pause(not on)
            if on:
                self.masterTableView.setToolTip("use 'Start' button to enable the master discovering")
                self.networkDock.setWindowTitle("ROS Network [disabled]")
            else:
                self.masterTableView.setToolTip('')
            if on:
                # remove discovered ROS master and set the local master to selected
                for uri in self.masters.keys():
                    master = self.masters[uri]
                    if nm.is_local(get_hostname(uri)) or uri == self.getMasteruri():
                        if not self._history_selected_robot or master.mastername == self._history_selected_robot:
                            self.setCurrentMaster(master)
                    else:
                        if master.master_state is not None:
                            self.master_model.removeMaster(master.master_state.name)
                        #self.removeMaster(uri)
            else:
                try:
                    # determine the ROS network ID
                    mcast_group = rospy.get_param(rospy.names.ns_join(discoverer, 'mcast_port'))
                    self.networkDock.setWindowTitle("ROS Network [id: %d]" % (mcast_group - 11511))
                    self._subscribe()
                except:
                    # try to get the multicast port of master discovery from log
                    port = 0
                    network_id = -1
                    import re
                    with open(ScreenHandler.getROSLogFile(node=discoverer.rstrip('/')), 'r') as mdfile:
                        for line in mdfile:
                            if line.find("Listen for multicast at") > -1:
                                port = map(int, re.findall(r'\d+', line))[-1]
                            elif line.find("Network ID") > -1:
                                network_id = map(int, re.findall(r'\d+', line))[-1]
                                port = 11511 + network_id
                    if port > 0:
                        self.networkDock.setWindowTitle("ROS Network [id: %d]" % (port - 11511))
                    else:
                        self.networkDock.setWindowTitle("ROS Network")

    def on_master_list_err_retrieved(self, masteruri, error):
        '''
        The callback method connected to the signal, which is emitted on an error
        while call the service to determine the discovered ROS master. On the error
        the local monitoring will be enabled.
        '''
        if 'no service' not in error:
            rospy.logwarn(error)
        self._setLocalMonitoring(True)

    def hasDiscoveryService(self, minfo):
        '''
        Test whether the new retrieved MasterInfo contains the master_discovery node.
        This is identified by a name of the contained 'list_masters' service.
        @param minfo: the ROS master Info
        @type minfo: U{master_discovery_fkie.MasterInfo<http://docs.ros.org/api/master_discovery_fkie/html/modules.html#module-master_discovery_fkie.master_info>}
        '''
        # use no discovery services, if roscore is running on a remote host
        if self.restricted_to_one_master:
            return False
        for service in minfo.services.keys():
            if service.endswith('list_masters'):
                return True
        return False

# ======================================================================================================================
# Handling of received ROS master state messages
# ======================================================================================================================

    def on_master_list_retrieved(self, masteruri, servic_name, master_list):
        '''
        Handle the retrieved list with ROS master.
          1. update the ROS Network view
        @param master_list: a list with ROS masters
        @type master_list: C{[U{master_discovery_fkie.msg.MasterState<http://docs.ros.org/api/multimaster_msgs_fkie/html/msg/MasterState.html>}]}
        '''
        result_1 = self.state_topic.registerByROS(self.getMasteruri(), False)
        result_2 = self.stats_topic.registerByROS(self.getMasteruri(), False)
        local_mon = not result_1 or not result_2
        self._setLocalMonitoring(local_mon, rospy.names.namespace(result_1))
        self._con_tries[masteruri] = 0
        # remove ROS master which are not in the new list
        new_uris = [m.uri for m in master_list if m.uri is not None]
        for uri in self.masters.keys():
            if uri not in new_uris:
                master = self.masters[uri]
                if not (nm.is_local(get_hostname(uri)) or uri == self.getMasteruri()):
                    if master.master_state is not None:
                        self.master_model.removeMaster(master.master_state.name)
                    self.setMasterOnline(uri, False)
                    # self.removeMaster(uri)
        # add or update master
        for m in master_list:
            if m.uri is not None:
                host = get_hostname(m.uri)
                nm.nameres().add_master_entry(m.uri, m.name, host)
                m.name = nm.nameres().mastername(m.uri)
                master = self.getMaster(m.uri)
                master.master_state = m
                master.online = True
                master.force_next_update()
                self._assigne_icon(m.name)
                self.master_model.updateMaster(m)
                self._update_handler.requestMasterInfo(m.uri, m.monitoruri)

    def on_master_state_changed(self, msg):
        '''
        Handle the received master state message.
          1. update the ROS Network view
          2. enable local master monitoring, if all masters are removed (the local master too)
        @param msg: the ROS message with new master state
        @type msg: U{master_discovery_fkie.msg.MasterState<http://docs.ros.org/api/multimaster_msgs_fkie/html/msg/MasterState.html>}
        '''
        # do not update while closing
        if hasattr(self, "_on_finish"):
            rospy.logdebug("ignore changes on %s, because currently on closing...", msg.master.uri)
            return
        host = get_hostname(msg.master.uri)
        if msg.state == MasterState.STATE_CHANGED:
            nm.nameres().add_master_entry(msg.master.uri, msg.master.name, host)
            msg.master.name = nm.nameres().mastername(msg.master.uri)
            self.getMaster(msg.master.uri).master_state = msg.master
            self._assigne_icon(msg.master.name)
            self.master_model.updateMaster(msg.master)
            if nm.settings().autoupdate:
                self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
            else:
                rospy.loginfo("Autoupdate disabled, the data will not be updated for %s" % msg.master.uri)
            if not msg.master.online:
                host = get_hostname(msg.master.uri)
                rospy.loginfo("remove SSH connection for '%s' because the master is now offline" % host)
                nm.ssh().remove(host)
        if msg.state == MasterState.STATE_NEW:
            # if new master with uri of the local master is received update the master list
            if msg.master.uri == self.getMasteruri():
                self.masterlist_service.retrieveMasterList(msg.master.uri, False)
            nm.nameres().add_master_entry(msg.master.uri, msg.master.name, host)
            msg.master.name = nm.nameres().mastername(msg.master.uri)
            self.getMaster(msg.master.uri).master_state = msg.master
            self._assigne_icon(msg.master.name)
            self.master_model.updateMaster(msg.master)
            if nm.settings().autoupdate:
                self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
            else:
                rospy.loginfo("Autoupdate disabled, the data will not be updated for %s" % msg.master.uri)
        if msg.state == MasterState.STATE_REMOVED:
            if msg.master.uri == self.getMasteruri():
                # switch to locale monitoring, if the local master discovering was removed
                self._setLocalMonitoring(True)
            else:
                nm.nameres().remove_master_entry(msg.master.uri)
                self.master_model.removeMaster(msg.master.name)
                self.setMasterOnline(msg.master.uri, False)
#                self.removeMaster(msg.master.uri)
        # start master_sync, if it was selected in the start dialog to start with master_dsicovery
        if self._syncs_to_start:
            if msg.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
                # we don't know which name for host was used to start master discovery
                if host in self._syncs_to_start:
                    self.on_sync_start(msg.master.uri)
                    self._syncs_to_start.remove(host)
                elif msg.master.name in self._syncs_to_start:
                    self.on_sync_start(msg.master.uri)
                    self._syncs_to_start.remove(msg.master.name)
                else:
                    addresses = nm.nameres().addresses(msg.master.uri)
                    for address in addresses:
                        if address in self._syncs_to_start:
                            self.on_sync_start(msg.master.uri)
                            self._syncs_to_start.remove(address)
#      if len(self.masters) == 0:
#        self._setLocalMonitoring(True)

    def _assigne_icon(self, name, path=None):
        '''
        Sets the new icon to the given robot. If the path is `None` set search for
        .png file with robot name.
        :param name: robot name
        :type name: str
        :param path: path of the icon (Default: None)
        :type path: str
        '''
        icon_path = path if path else nm.settings().robot_image_file(name)
        if name not in self.__icons or self.__icons[name][1] != path:
            if QFile.exists(icon_path):
                self.__icons[name] = (QIcon(icon_path), icon_path)
            elif name in self.__icons:
                del self.__icons[name]

    def on_master_monitor_err(self, msg):
        self._con_tries[self.getMasteruri()] += 1

    def on_master_info_retrieved(self, minfo):
        '''
        Integrate the received master info.
        @param minfo: the ROS master Info
        @type minfo: U{master_discovery_fkie.MasterInfo<http://docs.ros.org/api/master_discovery_fkie/html/modules.html#module-master_discovery_fkie.master_info>}
        '''
        if hasattr(self, "_on_finish"):
            rospy.logdebug("ignore changes on %s, because currently on closing...", minfo.masteruri)
            return
        rospy.logdebug("MASTERINFO from %s (%s) received", minfo.mastername, minfo.masteruri)
        self._con_tries[minfo.masteruri] = 0
#    cputimes_m = os.times()
#    cputime_init_m = cputimes_m[0] + cputimes_m[1]
        if minfo.masteruri in self.masters:
            for _, master in self.masters.items():  # _:=uri
                try:
                    if not master.online and master.masteruri != minfo.masteruri:
                        continue
                    # check for running discovery service
                    new_info = master.master_info is None or master.master_info.timestamp < minfo.timestamp
#          cputimes = os.times()
#          cputime_init = cputimes[0] + cputimes[1]
                    master.master_info = minfo
#          cputimes = os.times()
#          cputime = cputimes[0] + cputimes[1] - cputime_init
#          print master.master_state.name, cputime
                    if master.master_info is not None:
                        if self._history_selected_robot == minfo.mastername and self._history_selected_robot == master.mastername and self.currentMaster != master:
                            if self.currentMaster is not None and not self.currentMaster.is_local:
                                self.setCurrentMaster(master)
                        elif nm.is_local(get_hostname(master.master_info.masteruri)) or self.restricted_to_one_master:
                            if new_info:
                                has_discovery_service = self.hasDiscoveryService(minfo)
                                if (not self.own_master_monitor.isPaused() or not self.masterTableView.isEnabled()) and has_discovery_service:
                                    self._subscribe()
                            if self.currentMaster is None and (not self._history_selected_robot or self._history_selected_robot == minfo.mastername):
                                self.setCurrentMaster(master)

                        # update the list view, whether master is synchronized or not
                        if master.master_info.masteruri == minfo.masteruri:
                            self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
                            if self.default_profile_load:
                                self.default_profile_load = False
                                QTimer.singleShot(2000, self._load_default_profile_slot)
                    self.capabilitiesTable.updateState(minfo.masteruri, minfo)
                except Exception, e:
                    rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, utf8(e))
            # update the duplicate nodes
            self.updateDuplicateNodes()
            # update the buttons, whether master is synchronized or not
            if self.currentMaster is not None and self.currentMaster.master_info is not None and not self.restricted_to_one_master:
                self.syncButton.setEnabled(True)
                self.syncButton.setChecked(not self.currentMaster.master_info.getNodeEndsWith('master_sync') is None)
        else:
            self.masterlist_service.retrieveMasterList(minfo.masteruri, False)
        self.profiler.update_progress()
#    cputimes_m = os.times()
#    cputime_m = cputimes_m[0] + cputimes_m[1] - cputime_init_m
#    print "ALL:", cputime_m

    def _load_default_profile_slot(self):
        if not hasattr(self, "_on_finish"):
            self.profiler.on_load_profile_file(self.default_load_launch)

    def on_master_errors_retrieved(self, masteruri, error_list):
        self.master_model.updateMasterErrors(nm.nameres().mastername(masteruri), error_list)

    def on_master_timediff_retrieved(self, masteruri, timediff):
        self.master_model.updateTimeDiff(nm.nameres().mastername(masteruri), timediff)

    def on_master_info_error(self, masteruri, error):
        if masteruri not in self._con_tries:
            self._con_tries[masteruri] = 0
        self._con_tries[masteruri] += 1
        if masteruri == self.getMasteruri():
            rospy.logwarn("Error while connect to local master_discovery %s: %s", masteruri, error)
            # switch to local monitoring after 3 timeouts
            if self._con_tries[masteruri] > 2:
                self._setLocalMonitoring(True)
        master = self.getMaster(masteruri, False)
        if master and master.master_state is not None:
            self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri, self.DELAYED_NEXT_REQ_ON_ERR)

    def on_conn_stats_updated(self, stats):
        '''
        Handle the retrieved connection statistics.
          1. update the ROS Network view
        @param stats: a list with connection statistics
        @type stats: C{[U{master_discovery_fkie.msg.LinkState<http://docs.ros.org/api/multimaster_msgs_fkie/html/msg/LinkState.html>}]}
        '''
        for stat in stats.links:
            self.master_model.updateMasterStat(stat.destination, stat.quality)

# ======================================================================================================================
# Handling of master info frame
# ======================================================================================================================

    def on_info_clicked(self):
        text = ''.join(['<dl>'])
        text = ''.join([text, '<dt><b>Maintainer</b>: ', 'Alexander Tiderko ', '<font color=gray>alexander.tiderko@gmail.com</font>', '</dt>'])
        text = ''.join([text, '<dt><b>Author</b>: ', 'Alexander Tiderko, Timo Roehling', '</dt>'])
        text = ''.join([text, '<dt><b>License</b>: ', 'BSD, some icons are licensed under the GNU Lesser General Public License (LGPL) or Creative Commons Attribution-Noncommercial 3.0 License', '</dt>'])
        text = ''.join([text, '</dl>'])
        text = ''.join([text, '<dt><b>Version</b>: ', nm.__version__, ' (', nm.__date__, ')', '</dt>'])
        MessageBox.about(self, 'About Node Manager', text)

    def on_master_log_clicked(self):
        '''
        Tries to get the log of master_discovery node on the machine requested by a dialog.
        '''
        # get the history list
        user_list = [self.userComboBox.itemText(i) for i in reversed(range(self.userComboBox.count()))]
        user_list.insert(0, 'last used')
        params = {'Host': ('string', 'localhost'),
                  'Show master discovery log': ('bool', True),
                  'Show master sync log': ('bool', False),
                  'Username': ('string', user_list),
                  'Only screen log': ('bool', True),
                  # 'Optional Parameter': ('list', params_optional)
                  }
        dia = ParameterDialog(params, sidebar_var='Host')
        dia.setFilterVisible(False)
        dia.setWindowTitle('Show log')
        dia.resize(450, 150)
        dia.setFocusField('Host')
        if dia.exec_():
            try:
                params = dia.getKeywords()
                hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
                log_master_discovery = params['Show master discovery log']
                log_master_sync = params['Show master sync log']
                username = params['Username']
                screen_only = params['Only screen log']
                for hostname in hostnames:
                    try:
                        usr = username
                        if username == 'last used':
                            usr = nm.settings().host_user(hostname)
                        if log_master_discovery:
                            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                           '%s: show log of master discovery' % hostname,
                                                           nm.starter().openLog,
                                                           ('/master_discovery', hostname, usr, screen_only))
                        if log_master_sync:
                            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                           '%s: show log of master sync' % hostname,
                                                           nm.starter().openLog,
                                                           ('/master_sync', hostname, usr, screen_only))
                    except (Exception, nm.StartException) as err:
                        import traceback
                        print traceback.format_exc(1)
                        rospy.logwarn("Error while show LOG for master_discovery %s: %s" % (utf8(hostname), utf8(err)))
                        MessageBox.warning(self, "Show log error",
                                           'Error while show log of master_discovery',
                                           '%s' % utf8(err))
                    self._progress_queue.start()
            except Exception as err:
                MessageBox.warning(self, "Show log error",
                                   'Error while parse parameter',
                                   '%s' % utf8(err))

    def on_set_time_clicked(self):
        if self.currentMaster is not None:  # and not self.currentMaster.is_local:
            time_dialog = QDialog()
            ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TimeInput.ui')
            loadUi(ui_file, time_dialog)
            host = get_hostname(self.currentMaster.master_state.uri)
            time_dialog.setWindowTitle('Set time on %s' % host)
            time_dialog.hostsComboBox.addItems(nm.history().cachedParamValues('/ntp'))
            if self.currentMaster.is_local:
                time_dialog.dateFrame.setVisible(False)
            if time_dialog.exec_():
                running_nodes = self.currentMaster.getRunningNodesIfLocal(remove_system_nodes=True)
                if running_nodes:
                    ret = MessageBox.question(self, 'Set Time', 'There are running nodes. Stop them?', buttons=MessageBox.Yes | MessageBox.No)
                    if ret == MessageBox.Yes:
                        self.currentMaster.stop_nodes_by_name(running_nodes)
                if time_dialog.dateRadioButton.isChecked():
                    try:
                        rospy.loginfo("Set remote host time to local time: %s" % self.currentMaster.master_state.uri)
                        socket.setdefaulttimeout(10)
                        p = xmlrpclib.ServerProxy(self.currentMaster.master_state.monitoruri)
                        uri, success, newtime, errormsg = p.setTime(time.time())
                        if not success:
                            if errormsg.find('password') > -1:
                                errormsg += "\nPlease modify /etc/sudoers with sudoedit and add user privilege, e.g:"
                                errormsg += "\n%s  ALL=NOPASSWD: /bin/date" % self.currentMaster.current_user
                                errormsg += "\n!!!needed to be at the very end of file, don't forget a new line at the end!!!"
                                errormsg += "\n\nBe aware, it does not replace the time synchronization!"
                                errormsg += "\nIt sets approximate time without undue delays on communication layer."
                            MessageBox.warning(self, "Time set error",
                                               'Error while set time on %s' % uri, '%s' % utf8(errormsg))
                        else:
                            timediff = time.time() - newtime
                            rospy.loginfo("  New time difference to %s is approx.: %.3fs" % (self.currentMaster.master_state.uri, timediff))
                            self.on_master_timediff_retrieved(self.currentMaster.master_state.uri, timediff)
                    except Exception as e:
                        errormsg = '%s' % e
                        if errormsg.find('setTime') > -1:
                            errormsg += "\nUpdate remote multimaster_fkie!"
                        rospy.logwarn("Error while set time on %s: %s" % (self.currentMaster.master_state.uri, utf8(errormsg)))
                        MessageBox.warning(self, "Time sync error",
                                           'Error while set time on %s' % self.currentMaster.master_state.uri,
                                           '%s' % utf8(errormsg))
                    finally:
                        socket.setdefaulttimeout(None)
                elif time_dialog.ntpdateRadioButton.isChecked():
                    ntp_host = time_dialog.hostsComboBox.currentText()
                    nm.history().addParamCache('/ntp', ntp_host)
                    cmd = "%s %s" % ('sudo ntpdate -v -u -t 1', ntp_host)
                    nm.starter().ntpdate(host, cmd)

    def on_refresh_master_clicked(self):
        if self.currentMaster is not None:
            rospy.loginfo("Request an update from %s", utf8(self.currentMaster.master_state.monitoruri))
            if self.currentMaster.master_info is not None:
                check_ts = self.currentMaster.master_info.check_ts
                self.currentMaster.master_info.timestamp = self.currentMaster.master_info.timestamp - 1.0
                self.currentMaster.master_info.check_ts = check_ts
            if self.currentMaster.master_state is not None:
                self._update_handler.requestMasterInfo(self.currentMaster.master_state.uri, self.currentMaster.master_state.monitoruri)
            self.currentMaster.force_next_update()
#      self.currentMaster.remove_all_def_configs()

    def on_run_node_clicked(self):
        '''
        Open a dialog to run a ROS node without a configuration
        '''
        from run_dialog import RunDialog
        if self.currentMaster is not None:
            dia = RunDialog(get_hostname(self.currentMaster.masteruri), self.currentMaster.masteruri)
            if dia.exec_():
                params = dia.run_params()
                if params:
                    params = params + (False, self.currentMaster.current_user,)  # autorequest must be false
                    try:
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'run `%s` on %s' % (params[2], params[0]),
                                                       nm.starter().runNodeWithoutConfig,
                                                       params)
                        self._progress_queue.start()
                    except (Exception, nm.StartException), e:
                        rospy.logwarn("Error while run `%s` on %s: %s", params[2], params[0], utf8(e))
                        MessageBox.warning(self, "Run error",
                                           'Error while run node %s [%s]' % (params[2], params[1]),
                                           utf8(e))
                else:
                    MessageBox.critical(self, "Run error",
                                        'No binary specified')

    def on_rqt_plugin_start(self, name, plugin):
        if self.currentMaster is not None:
            try:
                args = []
                package = 'rqt_gui'
                binary = 'rqt_gui'
                prefix = 'rqt_'
                suffix = ''
                if name == 'RViz':
                    prefix = 'rviz_'
                    package = 'rviz'
                    binary = 'rviz'
                if plugin:
                    args = ['-s', plugin]
                if name == 'rosbag record':
                    package = 'rosbag'
                    binary = 'record'
                    prefix = ''
                    topic_names = []
                    current_tab = self.currentMaster.masterTab.tabWidget.tabText(self.currentMaster.masterTab.tabWidget.currentIndex())
                    if (current_tab == 'Nodes'):
                        nodes = self.currentMaster.nodesFromIndexes(self.currentMaster.masterTab.nodeTreeView.selectionModel().selectedIndexes())
                        if nodes:
                            for n in nodes:
                                topic_names.extend(n.published)
                    else:
                        topics = self.currentMaster.topicsFromIndexes(self.currentMaster.masterTab.topicsView.selectionModel().selectedIndexes())
                        if topics:
                            topic_names.extend([t.name for t in topics])
                    count_topics = 'ALL'
                    if topic_names:
                        args = [' '.join(topic_names)]
                        count_topics = '%d selected' % len(topic_names)
                    else:
                        args = ['-a']
                    ret = MessageBox.question(self, 'Start rosbag', 'Start rosbag record with %s topics to %s/record_TIMESTAMP?' % (count_topics, nm.settings().LOG_PATH), buttons=MessageBox.Yes | MessageBox.No)
                    if ret == MessageBox.No:
                        return
                    args.append("-o %s/record" % nm.settings().LOG_PATH)
                    suffix = "_%d" % int(time.time())
                node_name = '%s%s_%s%s' % (prefix, name.lower().replace(' ', '_'),
                                           self.currentMaster.master_state.name, suffix)
                self.currentMaster._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                             'start %s' % name,
                                                             nm.starter().runNodeWithoutConfig,
                                                             ('localhost', package, binary,
                                                              nm.nameres().normalize_name(node_name), args,
                                                              '%s' % self.currentMaster.master_state.uri,
                                                              False))
            except (Exception, nm.StartException), e:
                import traceback
                print utf8(traceback.format_exc(1))
                rospy.logwarn("Error while start %s: %s" % (name, utf8(e)))
                MessageBox.warning(self, "Start error",
                                   'Error while start %s' % name,
                                   '%s' % utf8(e))
            self.currentMaster._progress_queue.start()

    def on_sync_dialog_released(self, released=False, masteruri=None, external_call=False):
        self.syncButton.setEnabled(False)
        master = self.currentMaster
        sync_node = None
        if masteruri is not None:
            master = self.getMaster(masteruri, False)
        if master is not None and master.master_info is not None:
            sync_node = master.master_info.getNodeEndsWith('master_sync')
        if master is not None and (sync_node is None or external_call):
            self._sync_dialog.resize(350, 190)
            if self._sync_dialog.exec_():
                try:
                    host = get_hostname(master.masteruri)
                    if self._sync_dialog.interface_filename is not None:
                        # copy the interface file to remote machine
                        self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                            'Transfer sync interface %s' % host,
                                                            nm.starter().transfer_files,
                                                            ("%s" % host, self._sync_dialog.interface_filename, False, master.current_user))
                    self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                        'Start sync on %s' % host,
                                                        nm.starter().runNodeWithoutConfig,
                                                        ("%s" % host, 'master_sync_fkie', 'master_sync', 'master_sync', self._sync_dialog.sync_args, "%s" % master.masteruri, False, master.current_user))
                    self._progress_queue_sync.start()
                except:
                    import traceback
                    MessageBox.warning(self, "Start sync error",
                                       "Error while start sync node",
                                       utf8(traceback.format_exc(1)))
            else:
                self.syncButton.setChecked(False)
        elif sync_node is not None:
            master.stop_nodes([sync_node])
        self.syncButton.setEnabled(True)

    def on_sync_start(self, masteruri=None):
        '''
        Enable or disable the synchronization of the master cores
        '''
        key_mod = QApplication.keyboardModifiers()
        if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
            self.on_sync_dialog_released(masteruri=masteruri, external_call=True)
#       if not master.master_info is None:
#         node = master.master_info.getNodeEndsWith('master_sync')
#         self.syncButton.setChecked(not node is None)
        else:
            self.syncButton.setEnabled(False)
            master = self.currentMaster
            if masteruri is not None:
                master = self.getMaster(masteruri, False)
            if master is not None:
                # ask the user to start the master_sync with loaded launch file
                if master.master_info is not None:
                    node = master.getNode('/master_sync')
                    if node and node[0].has_configs():
                        def_cfg_info = '\nNote: default_cfg parameter will be changed!' if node[0].has_default_cfgs(node[0].cfgs) else ''
                        ret = MessageBox.question(self, 'Start synchronization', 'Start the synchronization using loaded configuration?\n\n `No` starts the master_sync with default parameter.%s' % def_cfg_info, buttons=MessageBox.Yes | MessageBox.No)
                        if ret == MessageBox.Yes:
                            master.start_nodes([node[0]])
                            return

                # start the master sync with default settings
                default_sync_args = ["_interface_url:='.'",
                                     '_sync_topics_on_demand:=False',
                                     '_ignore_hosts:=[]', '_sync_hosts:=[]',
                                     '_ignore_nodes:=[]', '_sync_nodes:=[]',
                                     '_ignore_topics:=[]', '_sync_topics:=[]',
                                     '_ignore_services:=[]', '_sync_services:=[]',
                                     '_sync_remote_nodes:=False']
                try:
                    host = get_hostname(master.masteruri)
                    self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                        'start sync on ' + utf8(host),
                                                        nm.starter().runNodeWithoutConfig,
                                                        (utf8(host), 'master_sync_fkie', 'master_sync', 'master_sync', default_sync_args, utf8(master.masteruri), False, master.current_user))
                    self._progress_queue_sync.start()
                except:
                    pass
            self.syncButton.setEnabled(True)

    def on_sync_stop(self, masteruri=None):
        master = self.currentMaster
        if masteruri is not None:
            master = self.getMaster(masteruri, False)
        if master is not None and master.master_info is not None:
            node = master.master_info.getNodeEndsWith('master_sync')
            if node is not None:
                master.stop_nodes([node])

    def on_master_timecheck(self):
        # HACK: sometimes the local monitoring will not be activated. This is the detection.
        if len(self.masters) < 2 and self.currentMaster is None:
            self._subscribe()
            return
        # update the info panel of the robot. If the node manager is not selected the updates are rarer.
        current_time = time.time()
        if self.isActiveWindow() or current_time - self._last_time_view_update > 15:
            self._last_time_view_update = current_time
            if self.currentMaster is not None and self.currentMaster.master_state is not None:
                master = self.getMaster(self.currentMaster.master_state.uri)
                name = master.master_state.name
                masteruri = master.master_state.uri
                if self.restricted_to_one_master:
                    name = ''.join([name, ' <span style=" color:red;">(restricted)</span>'])
                    if not self.masternameLabel.toolTip():
                        self.masternameLabel.setToolTip('The multicore options are disabled, because the roscore is running on remote host!')
                if master.master_info is not None:
                    self.showMasterName(masteruri, name, self.timestampStr(master.master_info.check_ts), master.master_state.online)
                    pass
                elif master.master_state is not None:
                    text = 'Try to get info!!!'
                    if not nm.settings().autoupdate:
                        text = 'Press F5 or click on reload to get info'
                    self.showMasterName(masteruri, name, text, master.master_state.online)
            else:
                self.showMasterName('', 'No robot selected', None, False)
        if (current_time - self._refresh_time > 30.0):
            masteruri = self.getMasteruri()
            if masteruri is not None:
                master = self.getMaster(masteruri)
                if master is not None and master.master_state is not None and nm.settings().autoupdate:
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
                con_err = '<span style=" color:red;">connection problems (%s tries)! </span>' % utf8(tries)
        except:
            pass
        if self.__current_master_label_name != name:
            self.__current_master_label_name = name
            show_name = name if nm.settings().show_domain_suffix else subdomain(name)
            self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s</span>' % show_name)
            color = QColor.fromRgb(nm.settings().host_color(self.__current_master_label_name, self._default_color.rgb()))
            self._new_color(color)
        ts = 'updated: %s' % utf8(timestamp) if timestamp is not None else ''
        if not nm.settings().autoupdate:
            ts = '%s<span style=" color:orange;"> AU off</span>' % ts
        self.masterInfoLabel.setText('<span style=" font-size:8pt;">%s%s</span>' % (con_err, ts))

        # load the robot image, if one exists
        if self.masternameLabel.isEnabled():
            if name in self.__icons:
                if self.__icons[name][0] != self.__current_icon:
                    icon = self.__icons[name][0]
                    self.__current_icon = icon
                    self.imageLabel.setPixmap(icon.pixmap(self.nameFrame.size()))
                    self.imageLabel.setToolTip(''.join(['<html><head></head><body><img src="', self.__icons[name][1], '" alt="', name, '"></body></html>']))
            elif self.__icons['default_pc'][0] != self.__current_icon:
                icon = self.__icons['default_pc'][0]
                self.__current_icon = icon
                self.imageLabel.setPixmap(icon.pixmap(self.nameFrame.size()))
                self.imageLabel.setToolTip('')
        # set sim_time info
        master = self.getMaster(masteruri, False)
        sim_time_enabled = self.masternameLabel.isEnabled() and master is not None and master.use_sim_time
        self.simTimeLabel.setVisible(bool(sim_time_enabled))
        launch_server_enabled = self.masternameLabel.isEnabled() and (master is not None) and master.has_launch_server()
        self.launchServerLabel.setVisible(launch_server_enabled)
        self.masternameLabel.setEnabled(online)
        self.masterInfoFrame.setEnabled((timestamp is not None))
        # update warning symbol / text
        if self.logButton.isEnabled():
            if self.logButton.text():
                self.logButton.setIcon(self.__icons['log_warning'][0])
                self.logButton.setText('')
            else:
                self.logButton.setText('%d' % self.log_dock.count())
                self.logButton.setIcon(self.__icons['empty'][0])

    def timestampStr(self, timestamp):
        dt = datetime.fromtimestamp(timestamp)
        diff = time.time() - timestamp
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
        return '%s (%s)' % (dt.strftime('%H:%M:%S'), before)

    def updateDuplicateNodes(self):
        # update the duplicate nodes
        running_nodes = dict()
        for _, m in self.masters.items():
            if m.master_state is not None and m.master_state.online:
                running_nodes.update(m.getRunningNodesIfLocal())
        for _, m in self.masters.items():
            if m.master_state is not None:
                m.markNodesAsDuplicateOf(running_nodes)


# ======================================================================================================================
# Handling of master list view
# ======================================================================================================================

    def on_master_table_pressed(self, selected):
        pass

    def on_master_table_clicked(self, selected):
        '''
        On click on the sync item, the master_sync node will be started or stopped,
        depending on run state.
        '''
        pass
#     item = self.master_model.itemFromIndex(selected)
#     if isinstance(item, MasterSyncItem):
#       pass

    def on_master_table_activated(self, selected):
        item = self.master_model.itemFromIndex(selected)
        MessageBox.information(self, item.name, item.toolTip())

    def on_master_selection_changed(self, selected):
        '''
        If a master was selected, set the corresponding Widget of the stacked layout
        to the current widget and shows the state of the selected master.
        '''
#     si = self.masterTableView.selectedIndexes()
#     for index in si:
#       if index.row() == selected.row():
        item = self.master_model.itemFromIndex(selected)
        if item is not None:
            self._history_selected_robot = item.master.name
            self.setCurrentMaster(item.master.uri)
            if self.currentMaster.master_info is not None and not self.restricted_to_one_master:
                node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
                self.syncButton.setEnabled(True)
                self.syncButton.setChecked(node is not None)
            else:
                self.syncButton.setEnabled(False)
            return
        self.launch_dock.raise_()

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
            self.userComboBox.setEditText(self.currentMaster.current_user)
        elif master is None:
            self.currentMaster = None
            self.stackedLayout.setCurrentIndex(0)
        else:  # it's masteruri
            self.currentMaster = self.getMaster(master)
            if self.currentMaster is not None:
                self.stackedLayout.setCurrentWidget(self.currentMaster)
                show_user_field = not self.currentMaster.is_local
                self._add_user_to_combo(self.currentMaster.current_user)
                self.userComboBox.setEditText(self.currentMaster.current_user)
            else:
                self.stackedLayout.setCurrentIndex(0)
        self.user_frame.setVisible(show_user_field)
        self.on_master_timecheck()

    def _add_user_to_combo(self, user):
        for i in range(self.userComboBox.count()):
            if user.lower() == self.userComboBox.itemText(i).lower():
                return
        self.userComboBox.addItem(user)

    def on_user_changed(self, user):
        if self.currentMaster is not None:
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
        for _, m in self.masters.items():
            if m.master_info is not None:
                check_ts = m.master_info.check_ts
                m.master_info.timestamp = m.master_info.timestamp - 1.0
                m.master_info.check_ts = check_ts
        self.masterlist_service.refresh(self.getMasteruri(), False)

    def on_discover_network_clicked(self):
        try:
            self._discover_dialog.raise_()
        except:
            mcast_group = rospy.get_param('/master_discovery/mcast_group', '226.0.0.0')
            self._discover_dialog = NetworkDiscoveryDialog(mcast_group, 11511, 100, self)
            self._discover_dialog.network_join_request.connect(self._join_network)
            self._discover_dialog.show()

    def on_start_robot_clicked(self):
        '''
        Tries to start the master_discovery node on the machine requested by a dialog.
        '''
        # get the history list
        user_list = [self.userComboBox.itemText(i) for i in reversed(range(self.userComboBox.count()))]
        user_list.insert(0, 'last used')
        params_optional = {'Discovery type': ('string', ['master_discovery', 'zeroconf']),
                           'ROS Master Name': ('string', 'autodetect'),
                           'ROS Master URI': ('string', 'ROS_MASTER_URI'),
                           'Robot hosts': ('string', ''),
                           'Username': ('string', user_list),
                           'MCast Group': ('string', '226.0.0.0'),
                           'Heartbeat [Hz]': ('float', 0.5)
                           }
        params = {'Host': ('string', 'localhost'),
                  'Network(0..99)': ('int', '0'),
                  'Start sync': ('bool', nm.settings().start_sync_with_discovery),
                  'Optional Parameter': ('list', params_optional)
                  }
        dia = ParameterDialog(params, sidebar_var='Host')
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start discovery')
        dia.resize(450, 330)
        dia.setFocusField('Host')
        if dia.exec_():
            try:
                params = dia.getKeywords()
                hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
                port = params['Network(0..99)']
                start_sync = params['Start sync']
                discovery_type = params['Optional Parameter']['Discovery type']
                mastername = 'autodetect'
                masteruri = 'ROS_MASTER_URI'
                if len(hostnames) < 2:
                    mastername = params['Optional Parameter']['ROS Master Name']
                    masteruri = params['Optional Parameter']['ROS Master URI']
                robot_hosts = params['Optional Parameter']['Robot hosts']
                username = params['Optional Parameter']['Username']
                mcast_group = params['Optional Parameter']['MCast Group']
                heartbeat_hz = params['Optional Parameter']['Heartbeat [Hz]']
                if robot_hosts:
                    robot_hosts = robot_hosts.replace(' ', ',')
                    robot_hosts = robot_hosts.replace(',,', ',')
                    robot_hosts = robot_hosts.replace('[', '')
                    robot_hosts = robot_hosts.replace(']', '')
                for hostname in hostnames:
                    try:
                        args = []
                        if port is not None and port and int(port) < 100 and int(port) >= 0:
                            args.append('_mcast_port:=%s' % (11511 + int(port)))
                        else:
                            args.append('_mcast_port:=%s' % (11511))
                        if not mastername == 'autodetect':
                            args.append('_name:=%s' % (mastername))
                        args.append('_mcast_group:=%s' % mcast_group)
                        args.append('_robot_hosts:=[%s]' % robot_hosts)
                        args.append('_heartbeat_hz:=%s' % heartbeat_hz)
                        # TODO: remove the name parameter from the ROS parameter server
                        usr = username
                        if username == 'last used':
                            usr = nm.settings().host_user(hostname)
                        muri = None if masteruri == 'ROS_MASTER_URI' else utf8(masteruri)
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'start discovering on %s' % hostname,
                                                       nm.starter().runNodeWithoutConfig,
                                                       (utf8(hostname), 'master_discovery_fkie', utf8(discovery_type), utf8(discovery_type), args, muri, False, usr))

                        # start the master sync with default settings
                        if start_sync:
                            if nm.is_local(hostname):
                                default_sync_args = ["_interface_url:='.'",
                                                     '_sync_topics_on_demand:=False',
                                                     '_ignore_hosts:=[]', '_sync_hosts:=[]',
                                                     '_ignore_nodes:=[]', '_sync_nodes:=[]',
                                                     '_ignore_topics:=[]', '_sync_topics:=[]',
                                                     '_ignore_services:=[]', '_sync_services:=[]',
                                                     '_sync_remote_nodes:=False']
                                self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                                    'start sync on %s' % hostname,
                                                                    nm.starter().runNodeWithoutConfig,
                                                                    (utf8(hostname), 'master_sync_fkie', 'master_sync', 'master_sync', default_sync_args, muri, False, usr))
                                self._progress_queue_sync.start()
                            else:
                                if hostname not in self._syncs_to_start:
                                    self._syncs_to_start.append(hostname)
                    except (Exception, nm.StartException) as e:
                        import traceback
                        print traceback.format_exc(1)
                        rospy.logwarn("Error while start master_discovery for %s: %s" % (utf8(hostname), utf8(e)))
                        MessageBox.warning(self, "Start error",
                                           'Error while start master_discovery',
                                           utf8(e))
                    self._progress_queue.start()
            except Exception as e:
                MessageBox.warning(self, "Start error",
                                   'Error while parse parameter',
                                   utf8(e))

    def _join_network(self, network):
        try:
            hostname = 'localhost'
            args = []
            if network < 100 and network >= 0:
                args.append(''.join(['_mcast_port:=', utf8(11511 + int(network))]))
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'start discovering on ' + utf8(hostname),
                                           nm.starter().runNodeWithoutConfig,
                                           (utf8(hostname), 'master_discovery_fkie', 'master_discovery', 'master_discovery', args, None, False))
            self._progress_queue.start()
        except (Exception, nm.StartException), e:
            rospy.logwarn("Error while start master_discovery for %s: %s", utf8(hostname), utf8(e))
            MessageBox.warning(self, "Start error",
                               'Error while start master_discovery',
                               utf8(e))

    def poweroff_host(self, host):
        try:
            if nm.is_local(utf8(host)):
                ret = MessageBox.warning(self, "ROS Node Manager",
                                         "Do you really want to shutdown localhost?",
                                         buttons=MessageBox.Ok | MessageBox.Cancel)
                if ret == MessageBox.Cancel:
                    return
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'poweroff `%s`' % host,
                                           nm.starter().poweroff,
                                           ('%s' % host,))
            masteruris = nm.nameres().masterurisbyaddr(host)
            for masteruri in masteruris:
                master = self.getMaster(masteruri)
                master.stop_nodes_by_name(['/master_discovery'])
            self._progress_queue.start()
            self.on_description_update('Description', '')
            self.launch_dock.raise_()
        except (Exception, nm.StartException), e:
            rospy.logwarn("Error while poweroff %s: %s", host, utf8(e))
            MessageBox.warning(self, "Run error",
                               'Error while poweroff %s' % host,
                               '%s' % utf8(e))

    def rosclean(self, host):
        try:
            ret = MessageBox.warning(self, "ROS Node Manager",
                                     "Do you really want delete all logs on `%s`?" % host,
                                     buttons=MessageBox.Ok | MessageBox.Cancel)
            if ret == MessageBox.Cancel:
                return
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'rosclean `%s`' % host,
                                           nm.starter().rosclean,
                                           ('%s' % host,))
            self._progress_queue.start()
            self.launch_dock.raise_()
        except (Exception, nm.StartException), e:
            rospy.logwarn("Error while rosclean %s: %s", host, utf8(e))
            MessageBox.warning(self, "Run error",
                               'Error while rosclean %s' % host,
                               '%s' % utf8(e))

# ======================================================================================================================
# Handling of the launch view signals
# ======================================================================================================================

    def on_load_launch_file(self, path, argv=[], masteruri=None):
        '''
        Load the launch file. A ROS master must be selected first.
        :param path: the path of the launch file.
        :type path: str
        '''
        if argv:
            rospy.loginfo("LOAD launch: %s with args: %s" % (path, argv))
        else:
            rospy.loginfo("LOAD launch: %s" % path)
        master_proxy = None
        if masteruri is not None:
            master_proxy = self.getMaster(masteruri, False)
        if master_proxy is None:
            master_proxy = self.stackedLayout.currentWidget()
        if isinstance(master_proxy, MasterViewProxy):
            try:
                master_proxy.launchfiles = (path, argv)
            except Exception, e:
                import traceback
                print utf8(traceback.format_exc(1))
                MessageBox.warning(self, "Loading launch file", path, '%s' % utf8(e))
#      self.setCursor(cursor)
        else:
            MessageBox.information(self, "Load of launch file", "Select a master first!",)

    def on_load_launch_as_default_bypkg(self, pkg, launch_file, master_proxy, args=[], host=None):
        argv = list(args)
        argv.append('_package:=%s' % pkg)
        argv.append('_launch_file:="%s"' % launch_file)
        hostname = host if host else nm.nameres().address(master_proxy.masteruri)
        name_file_prefix = launch_file.replace('.launch', '').replace(' ', '_')
        node_name = roslib.names.SEP.join(['%s' % nm.nameres().masteruri2name(master_proxy.masteruri),
                                           name_file_prefix,
                                           'default_cfg'])
        self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                  'start default config %s' % hostname,
                                                  nm.starter().runNodeWithoutConfig,
                                                  ('%s' % hostname, 'default_cfg_fkie', 'default_cfg',
                                                   node_name, argv, master_proxy.masteruri, False,
                                                   master_proxy.current_user))
        self.launch_dock.progress_queue.start()

    def on_load_launch_as_default(self, path, host=None):
        '''
        Load the launch file as default configuration. A ROS master must be selected first.
        :param path: the path of the launch file.
        :type path: str
        :param host: The host name, where the configuration start.
        :type host: str (Default: None)
        '''
        rospy.loginfo("LOAD launch as default: %s" % path)
        master_proxy = self.stackedLayout.currentWidget()
        if isinstance(master_proxy, MasterViewProxy):
            args = list()
            args.append('_package:=%s' % (package_name(os.path.dirname(path))[0]))
            args.append('_launch_file:="%s"' % os.path.basename(path))
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
                    inputDia.setWindowTitle('Enter the argv for %s' % path)
                    if inputDia.exec_():
                        params = inputDia.getKeywords()
                        args.extend(launchConfig.resolveArgs([''.join([p, ":='", v, "'"]) for p, v in params.items() if v]))
                    else:
                        return
            except:
                import traceback
                rospy.logwarn('Error while load %s as default: %s' % (path, traceback.format_exc(1)))
            hostname = host if host else nm.nameres().address(master_proxy.masteruri)
            name_file_prefix = os.path.basename(path).replace('.launch', '').replace(' ', '_')
            node_name = roslib.names.SEP.join(['%s' % nm.nameres().masteruri2name(master_proxy.masteruri),
                                               name_file_prefix,
                                               'default_cfg'])
            self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                      'start default config %s' % hostname,
                                                      nm.starter().runNodeWithoutConfig,
                                                      ('%s' % hostname, 'default_cfg_fkie', 'default_cfg',
                                                       node_name, args, master_proxy.masteruri, False,
                                                       master_proxy.current_user))
            self.launch_dock.progress_queue.start()
        else:
            MessageBox.information(self, "Load of launch file", "Select a master first!",)

    def on_launch_edit(self, files, search_text='', trynr=1):
        '''
        Opens the given files in an editor. If the first file is already open, select
        the editor. If search text is given, search for the text in files an goto the
        line.
        :param file: A list with paths
        :type file: list of strings
        :param search_text: A string to search in file
        :type search_text: str
        '''
        if files:
            path = files[0]
            if path in self.editor_dialogs:
                last_path = files[-1]
                try:
                    self.editor_dialogs[path].on_load_request(last_path, search_text)
                    self.editor_dialogs[path].raise_()
                    self.editor_dialogs[path].activateWindow()
                except:
                    if trynr > 1:
                        raise
                    del self.editor_dialogs[path]
                    self.on_launch_edit(files, search_text, 2)
            else:
                editor = Editor(files, search_text)
                self.editor_dialogs[path] = editor
                editor.finished_signal.connect(self._editor_dialog_closed)
                editor.show()

    def _editor_dialog_closed(self, files):
        '''
        If a editor dialog is closed, remove it from the list...
        '''
        if files[0] in self.editor_dialogs:
            del self.editor_dialogs[files[0]]

    def on_launch_transfer(self, files):
        '''
        Copies the selected file to a remote host
        :param file: A list with paths
        :type file: list of strings
        '''
        if files:
            host = 'localhost'
            username = nm.settings().default_user
            if self.currentMaster is not None:
                host = get_hostname(self.currentMaster.masteruri)
                username = self.currentMaster.current_user
            params = {'Host': ('string', host),
                      'recursive': ('bool', 'False'),
                      'Username': ('string', '%s' % username)
                      }
            dia = ParameterDialog(params)
            dia.setFilterVisible(False)
            dia.setWindowTitle('Transfer file')
            dia.resize(350, 120)
            dia.setFocusField('Host')
            if dia.exec_():
                try:
                    params = dia.getKeywords()
                    host = params['Host']
                    recursive = params['recursive']
                    username = params['Username']
                    for path in files:
                        rospy.loginfo("TRANSFER to %s@%s: %s" % (username, host, path))
                        self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                                  'transfer files to %s' % host,
                                                                  nm.starter().transfer_files,
                                                                  ('%s' % host, path, False, username))
                        if recursive:
                            for f in LaunchConfig.included_files(path):
                                self.launch_dock.progress_queue.add2queue(utf8(uuid.uuid4()),
                                                                          'transfer files to %s' % host,
                                                                          nm.starter().transfer_files,
                                                                          ('%s' % host, f, False, username))
                    self.launch_dock.progress_queue.start()
                except Exception, e:
                    MessageBox.warning(self, "Transfer error",
                                       'Error while transfer files', '%s' % utf8(e))

    def _reload_globals_at_next_start(self, launch_file):
        if self.currentMaster is not None:
            self.currentMaster.reload_global_parameter_at_next_start(launch_file)

# ======================================================================================================================
# Change file detection
# ======================================================================================================================

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

    def on_binaryfile_changed(self, changed, affected):
        '''
        Signal hander to handle the changes started binaries.
        @param changed: the changed file
        @type changed: str
        @param affected: list of tuples(node name, masteruri, launchfile), which are
                         affected by file change
        @type affected: list
        '''
        if self.isActiveWindow():
            self._changed_binaries[changed] = affected
            self._check_for_changed_files()
        else:
            self._changed_binaries[changed] = affected

    def _check_for_changed_files(self):
        '''
        Check the dictinary with changed files and notify the masters about changes.
        '''
        new_affected = list()
        for _, affected in self._changed_files.items():  # :=changed
            for (muri, lfile) in affected:
                if not (muri, lfile) in self.__in_question:
                    self.__in_question.add((muri, lfile))
                    new_affected.append((muri, lfile))
        # if there are no question to reload the launch file -> ask
        if new_affected:
            choices = dict()
            for (muri, lfile) in new_affected:
                master = self.getMaster(muri)
                if master is not None and master.online:
                    master.launchfile = lfile
                    choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
            if choices:
                cfgs, _ = SelectDialog.getValue('Reload configurations?',
                                                '<b>%s</b> was changed.<br>Select affected configurations to reload:' % ', '.join([os.path.basename(f) for f in self._changed_files.keys()]), choices.keys(),
                                                False, True,
                                                ':/icons/crystal_clear_launch_file.png',
                                                self)
                for c in cfgs:
                    choices[c][0].launchfiles = choices[c][1]
            for (muri, lfile) in new_affected:
                self.__in_question.remove((muri, lfile))
        self._changed_files.clear()

    def _check_for_changed_binaries(self):
        '''
        Check the dictinary with changed binaries and notify the masters about changes.
        '''
        new_affected = list()
        for _, affected in self._changed_binaries.items():  # :=changed
            for (nname, muri, lfile) in affected:
                if not (nname, muri, lfile) in self.__in_question:
                    self.__in_question.add((nname, muri, lfile))
                    new_affected.append((nname, muri, lfile))
        # if there are no question to restart the nodes -> ask
        if new_affected:
            choices = dict()
            for (nname, muri, lfile) in new_affected:
                master = self.getMaster(muri)
                if master is not None:
                    master_nodes = master.getNode(nname)
                    if master_nodes and master_nodes[0].is_running():
                        choices[nname] = (master, lfile)
                    else:
                        nm.filewatcher().rem_binary(nname)
            if choices:
                nodes, _ = SelectDialog.getValue('Restart nodes?',
                                                 '<b>%s</b> was changed.<br>Select affected nodes to restart:' % ', '.join([os.path.basename(f) for f in self._changed_binaries.keys()]), choices.keys(),
                                                 False, True,
                                                 '',
                                                 self)
                for (nname, muri, lfile) in new_affected:
                    self.__in_question.remove((nname, muri, lfile))
                for nname in nodes:
                    choices[nname][0].stop_nodes_by_name([nname])
                for nname in nodes:
                    choices[nname][0].start_nodes_by_name([nname], choices[nname][1], True)
        self._changed_binaries.clear()

    def on_configparamfile_changed(self, changed, affected):
        '''
        Signal handler to handle the changes of a configuration file referenced by a parameter value
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
                if master is not None:
                    master.launchfile = lfile
                    choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
            cfgs, _ = SelectDialog.getValue('Transfer configurations?',
                                            'Configuration files referenced by parameter are changed.<br>Select affected configurations for copy to remote host: (don\'t forget to restart the nodes!)',
                                            choices.keys(), False, True,
                                            ':/icons/crystal_clear_launch_file_transfer.png',
                                            self)
            for (muri, lfile) in new_affected:
                self.__in_question.remove((muri, lfile))
            for c in cfgs:
                host = '%s' % get_hostname(choices[c][0].masteruri)
                username = choices[c][0].current_user
                self.launch_dock.progress_queue.add2queue(utf8(uuid.uuid4()),
                                                          'transfer files to %s' % host,
                                                          nm.starter().transfer_files,
                                                          (host, choices[c][1], False, username))
            self.launch_dock.progress_queue.start()
        self._changed_files_param.clear()

    def changeEvent(self, event):
        '''
        Check for changed files, if the main gui is activated.
        '''
        QMainWindow.changeEvent(self, event)
        self._check_for_changed_files()
        self._check_for_changed_binaries()
        self._check_for_changed_files_param()

# ======================================================================================================================
# Capabilities handling
# ======================================================================================================================

    def on_start_nodes(self, masteruri, cfg, nodes):
        if masteruri is not None:
            master = self.getMaster(masteruri)
            master.start_nodes_by_name(nodes, roslib.names.ns_join(cfg, 'run'))

    def on_stop_nodes(self, masteruri, nodes):
        if masteruri is not None:
            master = self.getMaster(masteruri)
            master.stop_nodes_by_name(nodes)

    def on_description_update(self, title, text, force=False):
        same_title = self.descriptionDock.windowTitle() == title
        valid_sender = self.sender() == self.currentMaster or not isinstance(self.sender(), MasterViewProxy)
        no_focus = not self.descriptionTextEdit.hasFocus()
        if (valid_sender) and (same_title or no_focus or self._accept_next_update):
            self._accept_next_update = False
            self.descriptionDock.setWindowTitle(title)
            self.descriptionTextEdit.setText(text)
            if text and force:  # and not (self.launch_dock.hasFocus() or self.launch_dock.xmlFileView.hasFocus()):
                self.descriptionDock.raise_()
#      else:
#        self.launch_dock.raise_()

    def on_description_update_cap(self, title, text):
        self.descriptionDock.setWindowTitle(title)
        self.descriptionTextEdit.setText(text)

    def on_description_anchorClicked(self, url):
        self._accept_next_update = True
        if url.toString().startswith('open-sync-dialog://'):
            self.on_sync_dialog_released(False, url.toString().replace('open-sync-dialog', 'http'), True)
        elif url.toString().startswith('show-all-screens://'):
            master = self.getMaster(url.toString().replace('show-all-screens', 'http'), False)
            if master is not None:
                master.on_show_all_screens()
        elif url.toString().startswith('remove-all-launch-server://'):
            master = self.getMaster(url.toString().replace('remove-all-launch-server', 'http'), False)
            if master is not None:
                master.on_remove_all_launch_server()
        elif url.toString().startswith('node://'):
            if self.currentMaster is not None:
                self.currentMaster.on_node_selection_changed(None, None, True, self._url_path(url))
        elif url.toString().startswith('topic://'):
            if self.currentMaster is not None:
                self.currentMaster.on_topic_selection_changed(None, None, True, self._url_path(url))
        elif url.toString().startswith('topicecho://'):
            if self.currentMaster is not None:
                self.currentMaster.show_topic_output(self._url_path(url), False)
        elif url.toString().startswith('topichz://'):
            if self.currentMaster is not None:
                self.currentMaster.show_topic_output(self._url_path(url), True)
        elif url.toString().startswith('topichzssh://'):
            if self.currentMaster is not None:
                self.currentMaster.show_topic_output(self._url_path(url), True, use_ssh=True)
        elif url.toString().startswith('topicpub://'):
            if self.currentMaster is not None:
                self.currentMaster.start_publisher(self._url_path(url))
        elif url.toString().startswith('topicrepub://'):
            if self.currentMaster is not None:
                self.currentMaster.start_publisher(self._url_path(url), True)
        elif url.toString().startswith('topicstop://'):
            if self.currentMaster is not None:
                self.currentMaster.on_topic_pub_stop_clicked(self._url_path(url))
        elif url.toString().startswith('service://'):
            if self.currentMaster is not None:
                self.currentMaster.on_service_selection_changed(None, None, True, self._url_path(url))
        elif url.toString().startswith('servicecall://'):
            if self.currentMaster is not None:
                self.currentMaster.service_call(self._url_path(url))
        elif url.toString().startswith('unregister-node://'):
            if self.currentMaster is not None:
                self.currentMaster.on_unregister_nodes()
        elif url.toString().startswith('start-node://'):
            if self.currentMaster is not None:
                self.currentMaster.on_start_clicked()
        elif url.toString().startswith('restart-node://'):
            if self.currentMaster is not None:
                self.currentMaster.on_force_start_nodes()
        elif url.toString().startswith('restart-node-g://'):
            if self.currentMaster is not None:
                self.currentMaster.on_force_start_nodes(True)
        elif url.toString().startswith('start-node-at-host://'):
            if self.currentMaster is not None:
                self.currentMaster.on_start_nodes_at_host()
        elif url.toString().startswith('start-node-adv://'):
            if self.currentMaster is not None:
                self.currentMaster.on_start_alt_clicked()
        elif url.toString().startswith('kill-node://'):
            if self.currentMaster is not None:
                self.currentMaster.on_kill_nodes()
        elif url.toString().startswith('kill-screen://'):
            if self.currentMaster is not None:
                self.currentMaster.on_kill_screens()
        elif url.toString().startswith('copy-log-path://'):
            if self.currentMaster is not None:
                self.currentMaster.on_log_path_copy()
        elif url.toString().startswith('launch://'):
            self.on_launch_edit([self._url_path(url)], '')
        elif url.toString().startswith('reload-globals://'):
            self._reload_globals_at_next_start(self._url_path(url).replace('reload-globals://', ''))
        elif url.toString().startswith('poweroff://'):
            self.poweroff_host(self._url_host(url))
        elif url.toString().startswith('rosclean://'):
            self.rosclean(self._url_host(url))
        else:
            QDesktopServices.openUrl(url)
            self._accept_next_update = False

    def _url_path(self, url):
        '''Helper class for Qt5 compatibility'''
        if hasattr(url, 'encodedPath'):
            return utf8(url.encodedPath())
        else:
            return utf8(url.path())

    def _url_host(self, url):
        '''Helper class for Qt5 compatibility'''
        if hasattr(url, 'encodedHost'):
            return utf8(url.encodedHost())
        else:
            return utf8(url.host())

    def _restart_nodes(self):
        if self.currentMaster is not None:
            self.currentMaster.on_force_start_nodes()

    def keyPressEvent(self, event):
        '''
        '''
        QMainWindow.keyPressEvent(self, event)

    def _show_section_menu(self, event=None):
        # self._timer_alt = None
        if self._select_index == 0:
            if self.currentMaster is not None:
                if self.currentMaster._is_current_tab_name('tabNodes'):
                    self.currentMaster.masterTab.nodeTreeView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabTopics'):
                    self.currentMaster.masterTab.topicsView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabServices'):
                    self.currentMaster.masterTab.servicesView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabParameter'):
                    self.currentMaster.masterTab.parameterView.setFocus(Qt.TabFocusReason)
        elif self._select_index == 1:
            self.launch_dock.raise_()
            self.launch_dock.xmlFileView.setFocus(Qt.TabFocusReason)
        elif self._select_index == 2:
            self.descriptionDock.raise_()
            self.descriptionTextEdit.setFocus(Qt.TabFocusReason)
        elif self._select_index == 3:
            self.settings_dock.raise_()
            self.settings_dock.settingsTreeView.setFocus(Qt.TabFocusReason)
        elif self._select_index == 4:
            self.helpDock.raise_()
            self.textBrowser.setFocus(Qt.TabFocusReason)
        elif self._select_index == 5:
            self.startRobotButton.setFocus(Qt.TabFocusReason)
        elif self._select_index == 6:
            self.hideDocksButton.setFocus(Qt.TabFocusReason)
        else:
            self._select_index = -1
        self._select_index += 1

    def keyReleaseEvent(self, event):
        '''
        Defines some of shortcuts for navigation/management in launch
        list view or topics view.
        '''
        key_mod = QApplication.keyboardModifiers()
        if self.currentMaster is not None and self.currentMaster.masterTab.nodeTreeView.hasFocus():
            if event.key() == Qt.Key_F4 and not key_mod:
                if self.currentMaster.masterTab.editConfigButton.isEnabled():
                    self.currentMaster.on_edit_config_clicked()
                elif self.currentMaster.masterTab.editRosParamButton.isEnabled():
                    self.currentMaster.on_edit_rosparam_clicked()
            elif event.key() == Qt.Key_F3 and not key_mod and self.currentMaster.masterTab.ioButton.isEnabled():
                self.currentMaster.on_io_clicked()
        QMainWindow.keyReleaseEvent(self, event)

    def image_mouseDoubleClickEvent(self, event):
        '''
        Set the robot image
        '''
        if self.currentMaster:
            try:
                if not os.path.isdir(nm.settings().ROBOTS_DIR):
                    os.makedirs(nm.settings().ROBOTS_DIR)
                (fileName, _) = QFileDialog.getOpenFileName(self,
                                                            "Set robot image",
                                                            nm.settings().ROBOTS_DIR,
                                                            "Image files (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.xbm);;All files (*)")
                if fileName and self.__current_master_label_name:
                    p = QPixmap(fileName)
                    p.save(nm.settings().robot_image_file(self.__current_master_label_name))
                if self.__current_master_label_name in self.__icons:
                    del self.__icons[self.__current_master_label_name]
                self._assigne_icon(self.__current_master_label_name)
            except Exception as e:
                MessageBox.warning(self, "Error",
                                   'Set robot image for %s failed!' % utf8(self.__current_master_label_name),
                                   '%s' % utf8(e))
                rospy.logwarn("Error while set robot image for %s: %s", utf8(self.__current_master_label_name), utf8(e))

    def _set_custom_colors(self):
        colors = [self._default_color, QColor(87, 93, 94), QColor(60, 116, 96)]
        # QT4 compatibility hack (expected type by QT4 is QRgb, Qt5 is QColor)
        if QT_BINDING_VERSION.startswith("4"):
            colors = [c.rgb() for c in colors]
        QColorDialog.setStandardColor(0, colors[0])
        QColorDialog.setStandardColor(1, colors[1])
        QColorDialog.setStandardColor(2, colors[2])

    def _new_color(self, color):
        bg_style = "QWidget#expert_tab { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 %s, stop: 0.7 %s);}" % (color.name(), self._default_color.name())
        self.expert_tab.setStyleSheet("%s" % (bg_style))

    def mastername_mouseDoubleClickEvent(self, event):
        '''
        Set the robot color
        '''
        if self.currentMaster:
            try:
                prev_color = QColor.fromRgb(nm.settings().host_color(self.__current_master_label_name, self._default_color.rgb()))
                cdiag = QColorDialog(prev_color)
                cdiag.currentColorChanged.connect(self._new_color)
                if cdiag.exec_():
                    nm.settings().set_host_color(self.__current_master_label_name, cdiag.selectedColor().rgb())
                else:
                    self._new_color(prev_color)
            except Exception as e:
                MessageBox.warning(self, "Error",
                                   'Set robot color for %s failed!' % utf8(self.__current_master_label_name),
                                   '%s' % utf8(e))
                rospy.logwarn("Error while set robot color for %s: %s", utf8(self.__current_master_label_name), utf8(e))

    def _on_robot_icon_changed(self, masteruri, path):
        '''
        One of the robot icons was chagned. Update the icon.
        '''
        master = self.getMaster(masteruri, False)
        if master:
            self._assigne_icon(master.mastername, resolve_url(path))

    def _callback_diagnostics(self, data):
        try:
            for diagnostic in data.status:
                if DIAGNOSTICS_AVAILABLE:
                    self.diagnostics_signal.emit(diagnostic)
        except Exception as err:
            rospy.logwarn('Error while process diagnostic messages: %s' % utf8(err))

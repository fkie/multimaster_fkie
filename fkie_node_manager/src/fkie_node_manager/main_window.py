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
from docutils import examples
from fkie_multimaster_msgs.msg import MasterState
from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import QFile, QPoint, QSize, Qt, QTimer, QUrl, Signal
from python_qt_binding.QtGui import QDesktopServices, QIcon, QKeySequence, QPixmap
from python_qt_binding.QtGui import QPalette, QColor

import getpass
import grpc
import os
import rospy
import socket
import time
import uuid
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient
import ruamel.yaml

from fkie_master_discovery.common import resolve_url, subdomain, masteruri_from_master, masteruri_from_ros
from fkie_node_manager_daemon.common import utf8, get_pkg_path
from fkie_node_manager_daemon.host import get_hostname
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon import url as nmdurl
import fkie_node_manager as nm

from .capability_table import CapabilityTable
from .detailed_msg_box import MessageBox
from .discovery_listener import MasterListService, MasterStateTopic, MasterStatisticTopic, OwnMasterMonitoring
from .editor.editor import Editor
from .launch_enhanced_line_edit import EnhancedLineEdit
from .launch_files_widget import LaunchFilesWidget
from .log_widget import LogWidget
from .master_list_model import MasterModel, MasterButtonDelegate, MasterIconsDelegate
from .master_view_proxy import MasterViewProxy
from .menu_rqt import MenuRqt
from .network_discovery_dialog import NetworkDiscoveryDialog
from .parameter_dialog import ParameterDialog
from .profile_widget import ProfileWidget
from .progress_queue import ProgressQueue
from .logscreen.screen_dock import ScreenDock
from .select_dialog import SelectDialog
from .sync_dialog import SyncDialog
from .update_handler import UpdateHandler


try:
    from python_qt_binding.QtGui import QApplication, QFileDialog, QMainWindow, QStackedLayout, QWidget, QStyle
    from python_qt_binding.QtGui import QShortcut, QVBoxLayout, QColorDialog, QDialog, QRadioButton, QDockWidget
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QFileDialog, QMainWindow, QStackedLayout, QWidget, QStyle
    from python_qt_binding.QtWidgets import QShortcut, QVBoxLayout, QColorDialog, QDialog, QRadioButton, QDockWidget

try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    DIAGNOSTICS_AVAILABLE = True
except Exception:
    import sys
    sys.stderr.write("Cannot import 'diagnostic_msgs', feature disabled.")
    DIAGNOSTICS_AVAILABLE = False


class MainWindow(QMainWindow):
    '''
    The class to create the main window of the application.
    '''
    close_signal = Signal()

    DELAYED_NEXT_REQ_ON_ERR = 5.0

    if DIAGNOSTICS_AVAILABLE:
        diagnostics_signal = Signal(DiagnosticStatus)
    '''@ivar: the signal is emitted if a message on topic nm_notifier was
  reiceved (DiagnosticStatus)'''

    def __init__(self, files=[], restricted_to_one_master=False, monitor_port=22622, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QMainWindow.__init__(self)
        self.close_event_count = 0
        self.default_load_launch = os.path.abspath(resolve_url(files[0])) if files else ''
        self.default_profile_load = os.path.isfile(self.default_load_launch) and self.default_load_launch.endswith('.nmprofile')
        restricted_to_one_master = False
        self._finished = False
        self._history_selected_robot = ''
        self.__icons = {'empty': (QIcon(), ''),
                        'default_pc': (nm.settings().icon('crystal_clear_miscellaneous.png'), nm.settings().icon_path('crystal_clear_miscellaneous.png')),
                        'log_warning': (nm.settings().icon('crystal_clear_no_io.png'), nm.settings().icon_path('crystal_clear_no_io.png')),
                        'show_io': (nm.settings().icon('crystal_clear_show_io.png'), nm.settings().icon_path('crystal_clear_show_io.png'))
                        }
        self.__current_icon = None
        self.__current_master_label_name = None
        self.mcast_port = 11511
        self._syncs_to_start = []  # hostnames
        self._daemons_to_start = []  # hostnames
        self._accept_next_update = False
        self._last_window_state = False
        self._description_history = []
        self._description_accept = ''
        self._nmd_last_errors = {}  # msg: timestamp
        self._ts_nmd_error_last_check = 0
        # self.setAttribute(Qt.WA_AlwaysShowToolTips, True)
        # setup main window frame
        self.setObjectName('MainWindow')
#    self = mainWindow = QMainWindow()
#    self = mainWindow = loader.load(":/forms/MainWindow.ui")
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'MainWindow.ui')
        loadUi(ui_file, self)
        self.setObjectName('MainUI')
        self.setDockOptions(QMainWindow.AllowNestedDocks | QMainWindow.AllowTabbedDocks | QMainWindow.AnimatedDocks | QMainWindow.VerticalTabs)
         # set icons
        self.logButton.setIcon(nm.settings().icon('crystal_clear_show_io.png'))
        self.settingsButton.setIcon(nm.settings().icon('crystal_clear_settings.png'))
        self.infoButton.setIcon(nm.settings().icon('crystal_clear_info.png'))
        self.simTimeLabel.setPixmap(nm.settings().pixmap('crystal_clear_xclock.png'))
        self.launchServerLabel.setPixmap(nm.settings().pixmap('crystal_clear_launch_server.png'))
        self.user_label.setPixmap(nm.settings().pixmap('crystal_clear_user.png'))
        self.setTimeButton.setIcon(nm.settings().icon('crystal_clear_set_clock.png'))
        self.refreshHostButton.setIcon(nm.settings().icon('oxygen_view_refresh.png'))
        self.runButton.setIcon(nm.settings().icon('crystal_clear_clicknrun.png'))
        self.syncButton.setIcon(nm.settings().icon('irondevil_sync.png'))
        self.progressCancelButton_sync.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.progressCancelButton.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.refreshAllButton.setIcon(nm.settings().icon('oxygen_view_refresh.png'))
        self.discoveryButton.setIcon(nm.settings().icon('crystal_clear_discovery.png'))
        self.masterLogButton.setIcon(nm.settings().icon('crystal_clear_show_log.png'))
        self.startRobotButton.setIcon(nm.settings().icon('crystal_clear_run_zeroconf.png'))
        self.close_signal.connect(self.close)
        self.close_without_ask = False
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
        # setup logger widget
        self.log_dock = LogWidget()
        self.log_dock.added_signal.connect(self._on_log_added)
        self.log_dock.cleared_signal.connect(self._on_log_cleared)
        self.log_dock.setVisible(False)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.logButton.clicked.connect(self._on_log_button_clicked)
        self.settingsButton.clicked.connect(self._on_settings_button_clicked)
        # setup screen dock
        self.screen_dock = ScreenDock(self)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.screen_dock)
        self.screen_dock.hide()
        # setup the launch files view
        self.launch_dock = LaunchFilesWidget()
        self.launch_dock.load_signal.connect(self.on_load_launch_file)
        self.launch_dock.load_profile_signal.connect(self.profiler.on_load_profile_file)
        self.launch_dock.edit_signal.connect(self.on_launch_edit)
        self.launch_dock.transfer_signal.connect(self.on_launch_transfer)
        self.launch_dock.save_profile_signal.connect(self.profiler.on_save_profile)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.launch_dock)

        self.mIcon = nm.settings().icon('crystal_clear_prop_run.png')
        # self.style().standardIcon(QStyle.SP_FileIcon)
        self.setWindowTitle("Node Manager")
        self.setWindowIcon(self.mIcon)
#    self.setCentralWidget(mainWindow)
        # init the stack layout which contains the information about different ros master
        self.stackedLayout = QStackedLayout()
        self.stackedLayout.setObjectName('stackedLayout')
        self.tabWidget.currentChanged.connect(self.on_currentChanged_tab)
        self.tabLayout = QVBoxLayout(self.tabPlace)
        self.tabLayout.setObjectName("tabLayout")
        self.tabLayout.setContentsMargins(0, 0, 0, 0)
        self.tabLayout.addLayout(self.stackedLayout)

        # initialize the progress queue
        self._progress_queue = ProgressQueue(self.progressFrame, self.progressBar, self.progressCancelButton, 'Network')
        self._progress_queue_sync = ProgressQueue(self.progressFrame_sync, self.progressBar_sync, self.progressCancelButton_sync, 'Sync')

        rospy.loginfo('Detected ROS Master URI: %s' % self.getMasteruri())

        # initialize the view for the discovered ROS master
        self.master_model = MasterModel(self.getMasteruri())
        self.master_model.sync_start.connect(self.on_sync_start)
        self.master_model.sync_stop.connect(self.on_sync_stop)
        self.master_delegate_button = MasterButtonDelegate()
        self.masterTableView.setItemDelegateForColumn(0, self.master_delegate_button)
        self.master_delegate = MasterIconsDelegate()
        self.masterTableView.setItemDelegateForColumn(1, self.master_delegate)
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
        self.launch_dock.raise_()

        flags = self.windowFlags()
        self.setWindowFlags(flags | Qt.WindowContextHelpButtonHint)

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

        # add help page
        self.ui_help_web_view.page().setLinkDelegationPolicy(self.ui_help_web_view.page().DelegateAllLinks)
        self.ui_help_web_view.linkClicked.connect(self._on_help_link_clicked)
        self._help_history = []
        self._help_history_idx = -1
        self._help_root_url = QUrl('file://%s' % nm.settings().HELP_FILE)
        self._on_help_go_home()
        self.ui_help_home.clicked.connect(self._on_help_go_home)
        self.ui_help_back.clicked.connect(self._on_help_go_back)
        self.ui_help_forward.clicked.connect(self._on_help_go_forward)
        if self.ui_help_home.icon().isNull():
            self.ui_help_home.setText("Home")
        if self.ui_help_back.icon().isNull():
            self.ui_help_back.setText("Back")
        if self.ui_help_forward.icon().isNull():
            self.ui_help_forward.setText("Forward")

        try:
            screen.test_screen()
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

        if not nm.settings().movable_dock_widgets:
            self.networkDock.setFeatures(self.networkDock.NoDockWidgetFeatures)
            self.launch_dock.setFeatures(self.launch_dock.NoDockWidgetFeatures)
            self.descriptionDock.setFeatures(self.descriptionDock.NoDockWidgetFeatures)
            self.log_dock.setFeatures(self.log_dock.NoDockWidgetFeatures)

        # =============================
        # Initialize the update handler
        # =============================

        # initialize the class to get the state of discovering of other ROS master
        self._update_handler = UpdateHandler()
        self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
        self._update_handler.master_errors_signal.connect(self.on_master_errors_retrieved)
        self._update_handler.timediff_signal.connect(self.on_master_timediff_retrieved)
        self._update_handler.username_signal.connect(self.on_master_username_retrieved)
        self._update_handler.error_signal.connect(self.on_master_info_error)

        # this monitor class is used, if no master_discovery node is running to get the state of the local ROS master
        self.own_master_monitor = OwnMasterMonitoring()
        self.own_master_monitor.init(monitor_port)
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
        nm.nmd().monitor.system_diagnostics_signal.connect(self._callback_system_diagnostics)
        nm.nmd().monitor.remote_diagnostics_signal.connect(self._callback_diagnostics)
        nm.nmd().monitor.username_signal.connect(self._callback_username)

        self._select_index = 0
        self._shortcut_restart_nodes = QShortcut(QKeySequence(self.tr("Ctrl+Shift+R", "restart selected nodes")), self)
        self._shortcut_restart_nodes.activated.connect(self._restart_nodes)
        self._shortcut_restart_nodes_g = QShortcut(QKeySequence(self.tr("Ctrl+Shift+Alt+R", "restart selected nodes and reload global parameter")), self)
        self._shortcut_restart_nodes_g.activated.connect(self._restart_nodes_g)

        nm.nmd().error.connect(self.on_nmd_err)
        nm.nmd().settings.yaml_config_signal.connect(self._nmd_yaml_cfg)

    def _dock_widget_in(self, area=Qt.LeftDockWidgetArea, only_visible=False):
        result = []
        docks = [self.launch_dock, self.descriptionDock, self.networkDock]
        for dock in docks:
            if self.dockWidgetArea(dock) == area:
                if not only_visible or (only_visible and dock.isVisibleTo(self)):
                    result.append(dock)
        return result

    def _on_log_button_clicked(self):
        self.log_dock.setVisible(not self.log_dock.isVisible())

    def _on_settings_button_clicked(self):
        params = nm.settings().yaml()
        dia = ParameterDialog(params, store_geometry="settings_dialog")
        dia.setWindowTitle('Node Manager Configuration')
        if dia.exec_():
            try:
                params = dia.getKeywords(only_changed=True, with_tags=True)
                nm.settings().set_yaml(params)
            except Exception as err:
                import traceback
                print(traceback.format_exc())
                MessageBox.warning(self, "Configuration error",
                                   'Error while set parameter',
                                   '%s' % utf8(err))

    def _on_log_added(self, info, warn, err, fatal):
        self.logButton.setEnabled(True)

    def _on_log_cleared(self):
        self.logButton.setIcon(self.__icons['show_io'][0])
        self.logButton.setText('')
        # self.logButton.setEnabled(False)

    def on_hide_docks_toggled(self, checked):
        if self.dockWidgetArea(self.launch_dock) == Qt.LeftDockWidgetArea:
            self.launch_dock.setVisible(not checked)
        if self.dockWidgetArea(self.descriptionDock) == Qt.LeftDockWidgetArea:
            self.descriptionDock.setVisible(not checked)
        if self.dockWidgetArea(self.networkDock) == Qt.LeftDockWidgetArea:
            self.networkDock.setVisible(not checked)
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
            except Exception:
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
        if self.close_event_count > 0:
            # we handle force first
            self.finish()
            QMainWindow.closeEvent(self, event)
            return
        self.close_event_count += 1
        # ask to close nodes on exit
        # self.close_without_ask is changes in on_shutdown method in __init__.py
        if self._close_on_exit and nm.settings().confirm_exit_when_closing and not self.close_without_ask:
            masters = [uri for uri, m in self.masters.items() if m.online]
            res = SelectDialog.getValue('Stop nodes?', "Select masters where to stop:",
                                        masters, False, False, '', parent=self,
                                        select_if_single=False,
                                        checkitem1="don't show this dialog again",
                                        closein=nm.settings().timeout_close_dialog,
                                        store_geometry='stop_nodes')
            masters2stop, self._close_on_exit = res[0], res[1]
            nm.settings().confirm_exit_when_closing = not res[2]
            if self._close_on_exit or rospy.is_shutdown():
                self.on_finish = True
                self._stop_local_master = None
                for uri in masters2stop:
                    try:
                        m = self.masters[uri]
                        if m is not None:
                            if m.is_local:
                                self._stop_updating()
                                self._stop_local_master = m
                            m.stop_nodes_by_name(m.get_nodes_runningIfLocal(remove_system_nodes=True), True, [rospy.get_name(), '/rosout'])
                            if not m.is_local:
                                m.killall_roscore()
                    except Exception as e:
                        rospy.logwarn("Error while stop nodes on %s: %s" % (uri, utf8(e)))
                QTimer.singleShot(200, self._test_for_finish)
                if masters2stop:
                    event.ignore()
                else:
                    event.accept()
            else:
                self._close_on_exit = True
                self.close_event_count = 0
                event.ignore()
        elif self._are_master_in_process():
            QTimer.singleShot(200, self._test_for_finish)
            self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s ...closing...</span>' % self.masternameLabel.text())
            rospy.loginfo("Wait for running processes are finished...")
            event.ignore()
        try:
            self.storeSetting()
        except Exception as e:
            rospy.logwarn("Error while store settings: %s" % e)
        if event.isAccepted():
            self.on_finish = True
            self.master_timecheck_timer.stop()
            self.finish()
            QMainWindow.closeEvent(self, event)

    def _are_master_in_process(self):
        for _uri, m in self.masters.items():
            m.stop_echo_dialogs()
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
        self.launch_dock.stop()
        self.log_dock.stop()

    def finish(self):
        if not self._finished:
            self._finished = True
            print("Mainwindow finish...")
            self.screen_dock.finish()
            self._stop_updating()
            try:
                editors = [e for e in self.editor_dialogs.values()]
                for editor in editors:
                    editor.close()
            except Exception as _err:
                import traceback
                print(traceback.format_exc())
            for _, master in self.masters.items():
                try:
                    master.close()
                except Exception as _err:
                    import traceback
                    print(traceback.format_exc())
            print("Mainwindow finished!")

    def getMasteruri(self):
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it. The 'materuri' attribute will be set to the requested value.
        @return: ROS master URI
        @rtype: C{str} or C{None}
        '''
        if not hasattr(self, 'materuri') or self.materuri is None:
            masteruri = masteruri_from_ros()
            master = xmlrpcclient.ServerProxy(masteruri)
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
            if DIAGNOSTICS_AVAILABLE:
                self.diagnostics_signal.connect(self.masters[masteruri].append_diagnostic)
            self.stackedLayout.addWidget(self.masters[masteruri])
            if masteruri == self.getMasteruri():
                self.masters[masteruri].default_load_launch = self.default_load_launch
        return self.masters[masteruri]

    def on_host_update_request(self, host):
        for key, value in self.masters.items():
            if get_hostname(key) == host and value.master_state is not None:
                self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)

    def on_host_description_updated(self, masteruri, host, descr):
        # self.master_model.update_description(nm.nameres().mastername(masteruri, host), descr)
        pass

    def on_capabilities_update(self, masteruri, address, config_node, descriptions):
        for d in descriptions:
            self.capabilitiesTable.updateCapabilities(masteruri, config_node, d)
        if masteruri is not None:
            master = self.getMaster(masteruri)
            self.capabilitiesTable.updateState(masteruri, master.master_info)

    def on_remove_config(self, cfg):
        self.capabilitiesTable.removeConfig(cfg)

    def open_screen_dock(self, masteruri, screen_name, nodename, user=''):
        self.screen_dock.connect(masteruri, screen_name, nodename, user)

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
            except Exception:
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
            self.master_delegate.set_enabled(not on)
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
            else:
                try:
                    # determine the ROS network ID
                    self.mcast_port = rospy.get_param(rospy.names.ns_join(discoverer, 'mcast_port'))
                    self.networkDock.setWindowTitle("ROS Network [id: %d]" % (self.mcast_port - 11511))
                    self._subscribe()
                except Exception:
                    # try to get the multicast port of master discovery from log
                    port = 0
                    network_id = -1
                    import re
                    with open(screen.get_ros_logfile(node=discoverer.rstrip('/')), 'r') as mdfile:
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
        @type minfo: U{fkie_master_discovery.MasterInfo<http://docs.ros.org/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>}
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
        @type master_list: C{[U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}]}
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
        @type msg: U{fkie_master_discovery.msg.MasterState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/MasterState.html>}
        '''
        # do not update while closing
        if hasattr(self, "on_finish"):
            rospy.logdebug("ignore changes on %s, because currently on closing...", msg.master.uri)
            return
        host = get_hostname(msg.master.uri)
        if msg.state == MasterState.STATE_CHANGED:
            nm.nameres().add_master_entry(msg.master.uri, msg.master.name, host)
            msg.master.name = nm.nameres().mastername(msg.master.uri)
            master = self.getMaster(msg.master.uri)
            update = master.master_state is None
            if master.master_state is not None:
                if master.master_state.last_change.secs != msg.master.last_change.secs \
                    or master.master_state.last_change.nsecs != msg.master.last_change.nsecs:
                    update = True
            if update:
                master.master_state = msg.master
                self._assigne_icon(msg.master.name)
                self.master_model.updateMaster(msg.master)
                if nm.settings().autoupdate:
                    self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
                else:
                    rospy.loginfo("Autoupdate disabled, the data will not be updated for %s" % msg.master.uri)
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
                nm.nameres().remove_master_entry(msg.master.uri)
                self._setLocalMonitoring(True)
            else:
                nm.nameres().remove_master_entry(msg.master.uri)
                self.master_model.removeMaster(msg.master.name)
                self.setMasterOnline(msg.master.uri, False)
#                self.removeMaster(msg.master.uri)
        if msg.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
            # start master_sync, if it was selected in the start dialog to start with master_dsicovery
            if self._syncs_to_start:
                # we don't know which name for host was used to start master discovery
                if host in self._syncs_to_start:
                    self._syncs_to_start.remove(host)
                    self.on_sync_start(msg.master.uri)
                elif msg.master.name in self._syncs_to_start:
                    self._syncs_to_start.remove(msg.master.name)
                    self.on_sync_start(msg.master.uri)
                else:
                    addresses = nm.nameres().addresses(msg.master.uri)
                    for address in addresses:
                        if address in self._syncs_to_start:
                            self._syncs_to_start.remove(address)
                            self.on_sync_start(msg.master.uri)
            # start daemon, if it was selected in the start dialog to start with master_dsicovery
            if self._daemons_to_start:
                # we don't know which name for host was used to start master discovery
                if host in self._daemons_to_start:
                    self._daemons_to_start.remove(host)
                    self.on_daemon_start(msg.master.uri)
                elif msg.master.name in self._daemons_to_start:
                    self._daemons_to_start.remove(msg.master.name)
                    self.on_daemon_start(msg.master.uri)
                else:
                    addresses = nm.nameres().addresses(msg.master.uri)
                    for address in addresses:
                        if address in self._daemons_to_start:
                            self._daemons_to_start.remove(address)
                            self.on_daemon_start(msg.master.uri)
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
        @type minfo: U{fkie_master_discovery.MasterInfo<http://docs.ros.org/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>}
        '''
        if hasattr(self, "on_finish"):
            rospy.logdebug("ignore changes on %s, because currently on closing...", minfo.masteruri)
            return
        rospy.logdebug("MASTERINFO from %s (%s) received", minfo.mastername, minfo.masteruri)
        self._con_tries[minfo.masteruri] = 0
#    cputimes_m = os.times()
#    cputime_init_m = cputimes_m[0] + cputimes_m[1]
        if minfo.masteruri in self.masters:
            has_master_sync = False
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
#                        elif nm.is_local(get_hostname(master.master_info.masteruri)) or self.restricted_to_one_master:
                        elif master.master_info.masteruri == masteruri_from_master() or self.restricted_to_one_master:
                            if new_info:
                                has_discovery_service = self.hasDiscoveryService(minfo)
                                if (not self.own_master_monitor.isPaused() or not self.masterTableView.isEnabled()) and has_discovery_service:
                                    self._subscribe()
                            if self.currentMaster is None and (not self._history_selected_robot or self._history_selected_robot == minfo.mastername):
                                self.setCurrentMaster(master)
                                # this info is collected by daemon
                                # if not hasattr(self, "_sub_extended_log"):
                                #     agg_suffix = '_agg' if nm.settings().use_diagnostics_agg else ''
                                #     self._sub_extended_log = rospy.Subscriber('/diagnostics_agg' % agg_suffix, DiagnosticArray, self._callback_diagnostics)
                        # update the list view, whether master is synchronized or not
                        if master.master_info.masteruri == minfo.masteruri:
                            self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
                            if self.default_profile_load:
                                self.default_profile_load = False
                                QTimer.singleShot(2000, self._load_default_profile_slot)
                        has_master_sync = has_master_sync or 'fkie_multimaster_msgs/GetSyncInfo' in [srv.type for srv in master.master_info.services.values()]
                    self.capabilitiesTable.updateState(minfo.masteruri, minfo)
                except Exception as e:
                    rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, utf8(e))
            for _, master in self.masters.items():
                master.has_master_sync = has_master_sync
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
        if not hasattr(self, "on_finish"):
            self.profiler.on_load_profile_file(self.default_load_launch)

    def on_master_errors_retrieved(self, masteruri, error_list):
        self.master_model.updateMasterErrors(nm.nameres().mastername(masteruri), error_list)

    def on_master_timediff_retrieved(self, masteruri, timediff):
        self.master_model.updateTimeDiff(nm.nameres().mastername(masteruri), timediff)

    def on_master_username_retrieved(self, masteruri, username):
        master = self.getMaster(masteruri, create_new=False)
        if master is not None:
            master.current_user = username

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
        if master and master.master_state is not None and master.online:
            self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri, self.DELAYED_NEXT_REQ_ON_ERR)

    def on_conn_stats_updated(self, stats):
        '''
        Handle the retrieved connection statistics.
          1. update the ROS Network view
        @param stats: a list with connection statistics
        @type stats: C{[U{fkie_master_discovery.msg.LinkState<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/LinkState.html>}]}
        '''
        for stat in stats.links:
            self.master_model.updateMasterStat(stat.destination, stat.quality)

# ======================================================================================================================
# Handling of master info frame
# ======================================================================================================================

    def on_info_clicked(self):
        text = '<dl>'
        text = '%s<dt><b>Maintainer</b>: Alexander Tiderko <font color=gray>alexander.tiderko@gmail.com</font></dt>' % text
        text = '%s<dt><b>Author</b>: Alexander Tiderko, Timo Roehling</dt>' % text
        text = '%s<dt><b>License</b>: BSD, some icons are licensed under the GNU Lesser General Public License (LGPL) or Creative Commons Attribution-Noncommercial 3.0 License</dt>' % text
        text = '%s</dl>' % text
        if nm.__date__ == 'unknown':
            text = '%s<dt><b>Version</b>: %s</dt>' % (text, nm.__version__)
        else:
            text = '%s<dt><b>Version</b>: %s (%s)</dt>' % (text, nm.__version__, nm.__date__)
        text = '%s<dt><b>URL</b>: <a href="https://github.com/fkie/multimaster_fkie">https://github.com/fkie/multimaster_fkie</a></dt>' % (text)
        MessageBox.about(self, 'About Node Manager', text)

    def on_master_log_clicked(self):
        '''
        Tries to get the log of master_discovery node on the machine requested by a dialog.
        '''
        # get the history list
        user_list = [self.userComboBox.itemText(i) for i in reversed(range(self.userComboBox.count()))]
        user_list.insert(0, 'last used')
        params = {'Host': {':type': 'string', ':value': 'localhost'},
                  'Show master discovery log': {':type': 'bool', ':value': True},
                  'Show master sync log': {':type': 'bool', ':value': False},
                  'Show daemon log': {':type': 'bool', ':value': False},
                  'Username': {':type': 'string', ':value': user_list},
                  'Only screen log': {':type': 'bool', ':value': True, ':hint': 'There are two logs: ROS-Log and SCREEN-Log'},
                  # 'Optional Parameter': ('list', params_optional)
                  }
        dia = ParameterDialog(params, sidebar_var='Host', store_geometry="master_log_dialog")
        dia.setFilterVisible(False)
        dia.setWindowTitle('Show log')
        dia.setFocusField('Host')
        if dia.exec_():
            try:
                params = dia.getKeywords(only_changed=False, with_tags=False)
                print("params", params)
                hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
                log_master_discovery = params['Show master discovery log']
                log_master_sync = params['Show master sync log']
                log_nm_daemon = params['Show daemon log']
                username = params['Username']
                screen_only = params['Only screen log']
                for hostname in hostnames:
                    try:
                        usr = username
                        if username == 'last used':
                            usr = nm.settings().host_user(hostname)
                        else:
                            nm.settings().set_host_user(hostname, usr)
                        if log_master_discovery:
                            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                           '%s: show log of master discovery' % hostname,
                                                           nm.starter().openLog,
                                                           {'nodename' : '/master_discovery',
                                                            'host': hostname,
                                                            'user': usr,
                                                            'only_screen': screen_only
                                                           })
                        if log_master_sync:
                            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                           '%s: show log of master sync' % hostname,
                                                           nm.starter().openLog,
                                                           {'nodename' : '/master_sync',
                                                            'host': hostname,
                                                            'user': usr,
                                                            'only_screen': screen_only
                                                           })
                        if log_nm_daemon:
                            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                           '%s: show log of nm daemon' % hostname,
                                                           nm.starter().openLog,
                                                           {'nodename' : '/node_manager_daemon',
                                                            'host': hostname,
                                                            'user': usr,
                                                            'only_screen': screen_only
                                                           })
                    except (Exception, nm.StartException) as err:
                        import traceback
                        print(traceback.format_exc(1))
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
        master2update = self.currentMaster
        if master2update is not None:  # and not master2update.is_local:
            time_dialog = QDialog()
            ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'TimeInput.ui')
            loadUi(ui_file, time_dialog)
            host = get_hostname(master2update.master_state.uri)
            time_dialog.setWindowTitle('Set time on %s' % host)
            time_dialog.hostsComboBox.addItems(nm.history().cachedParamValues('/ntp'))
            if master2update.is_local:
                time_dialog.dateFrame.setVisible(False)
            if time_dialog.exec_():
                running_nodes = master2update.get_nodes_runningIfLocal(remove_system_nodes=True)
                restart_ret = MessageBox.No
                if running_nodes:
                    restart_ret = MessageBox.question(self, 'Set Time', 'There are running nodes. Stop them?\nNote: on "YES" only system nodes will be restarted automatically!', buttons=MessageBox.Yes | MessageBox.No)
                    if restart_ret == MessageBox.Yes:
                        if '/node_manager' in running_nodes:
                            del running_nodes['/node_manager']
                        if '/node_manager_daemon' in running_nodes:
                            master2update.reloading_files = master2update.launchfiles
                            master2update.restarting_daemon = True
                        master2update.stop_nodes_by_name(running_nodes, force=True)
                        master2update._progress_queue.start()

                if time_dialog.dateRadioButton.isChecked():
                    try:
                        rospy.loginfo("Set remote host time to local time: %s" % master2update.master_state.uri)
                        socket.setdefaulttimeout(10)
                        p = xmlrpcclient.ServerProxy(master2update.master_state.monitoruri)
                        uri, success, newtime, errormsg = p.setTime(time.time()) 
                        if not success:
                            errormsg = str(errormsg)
                            if errormsg.find('password') > -1:
                                errormsg += "\nPlease modify /etc/sudoers with sudoedit and add user privilege, e.g:"
                                errormsg += "\n%s  ALL=NOPASSWD: /bin/date" % master2update.current_user
                                errormsg += "\n!!!needed to be at the very end of file, don't forget a new line at the end!!!"
                                errormsg += "\n\nBe aware, it does not replace the time synchronization!"
                                errormsg += "\nIt sets approximate time without undue delays on communication layer."
                            MessageBox.warning(self, "Time set error",
                                               'Error while set time on %s' % uri, '%s' % utf8(errormsg))
                        else:
                            timediff = time.time() - newtime
                            rospy.loginfo("  New time difference to %s is approx.: %.3fs" % (master2update.master_state.uri, timediff))
                            self.on_master_timediff_retrieved(master2update.master_state.uri, timediff)
                    except Exception as e:
                        errormsg = '%s' % e
                        if errormsg.find('setTime') > -1:
                            errormsg += "\nUpdate remote fkie_multimaster!"
                        import traceback
                        rospy.logwarn("Error while set time on %s: %s" % (master2update.master_state.uri, utf8(errormsg)))
                        MessageBox.warning(self, "Time sync error",
                                           'Error while set time on %s' % master2update.master_state.uri,
                                           '%s' % utf8(errormsg))
                    finally:
                        socket.setdefaulttimeout(None)
                elif time_dialog.ntpdateRadioButton.isChecked():
                    ntp_host = time_dialog.hostsComboBox.currentText()
                    nm.history().addParamCache('/ntp', ntp_host)
                    cmd = "%s %s" % ('sudo ntpdate -v -u -t 1', ntp_host)
                    nm.starter().ntpdate(host, cmd)
                # now start system nodes
                if running_nodes and restart_ret == MessageBox.Yes:
                    if '/node_manager_daemon' in running_nodes:
                        master2update.start_daemon()
                    if '/master_discovery' in running_nodes:
                        master2update._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                    'start discovering on %s' % host,
                                                    nm.starter().runNodeWithoutConfig,
                                                    {'host': utf8(host),
                                                        'package': 'fkie_master_discovery',
                                                        'binary': 'master_discovery',
                                                        'name': '/master_discovery',
                                                        'args': [],
                                                        'masteruri': master2update.master_state.uri,
                                                        'use_nmd': False,
                                                        'auto_pw_request': False,
                                                        'user': master2update.current_user
                                                    })
                    if '/master_sync' in running_nodes:
                        master2update._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                            'start sync on ' + utf8(host),
                                                            nm.starter().runNodeWithoutConfig,
                                                            {'host': utf8(host),
                                                            'package': 'fkie_master_sync',
                                                            'binary': 'master_sync',
                                                            'name': 'master_sync',
                                                            'args': [],
                                                            'masteruri': master2update.master_state.uri,
                                                            'use_nmd': False,
                                                            'auto_pw_request': False,
                                                            'user': master2update.current_user
                                                            })
                    master2update._progress_queue.start()

    def on_refresh_master_clicked(self):
        if self.currentMaster is not None:
            rospy.loginfo("Request an update from %s", utf8(self.currentMaster.master_state.monitoruri))
            if self.currentMaster.master_info is not None:
                check_ts = self.currentMaster.master_info.check_ts
                self.currentMaster.master_info.timestamp = self.currentMaster.master_info.timestamp - 1.0
                self.currentMaster.master_info.check_ts = check_ts
                self.currentMaster.perform_master_checks()
            if self.currentMaster.master_state is not None:
                self._update_handler.requestMasterInfo(self.currentMaster.master_state.uri, self.currentMaster.master_state.monitoruri)
            self.currentMaster.force_next_update()
#      self.currentMaster.remove_all_def_configs()

    def on_run_node_clicked(self):
        '''
        Open a dialog to run a ROS node without a configuration
        '''
        from .run_dialog import RunDialog
        if self.currentMaster is not None:
            dia = RunDialog(get_hostname(self.currentMaster.masteruri), self.currentMaster.masteruri)
            if dia.exec_():
                params = dia.run_params()
                if params:
                    params['use_nmd'] = True
                    params['auto_pw_request'] = False  # autorequest must be false
                    params['user'] = self.currentMaster.current_user
                    try:
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'run `%s` on %s' % (params['binary'], params['host']),
                                                       nm.starter().runNodeWithoutConfig,
                                                       params)
                        self._progress_queue.start()
                    except (Exception, nm.StartException) as e:
                        rospy.logwarn("Error while run `%s` on %s: %s" % (params['binary'], params['host'], utf8(e)))
                        MessageBox.warning(self, "Run error",
                                           'Error while run node %s [%s]' % (params['binary'], params['package']),
                                           utf8(e))
                else:
                    MessageBox.critical(self, "Run error",
                                        'No binary specified')

    def on_rqt_plugin_start(self, name, plugin):
        if self.currentMaster is not None:
            try:
                if name == 'Terminal':
                    host = get_hostname(self.currentMaster.master_state.uri)
                    nm.starter().open_terminal(host)
                    return
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
                    current_tab = self.currentMaster.ui.tabWidget.tabText(self.currentMaster.ui.tabWidget.currentIndex())
                    if (current_tab == 'Nodes'):
                        nodes = self.currentMaster.nodesFromIndexes(self.currentMaster.ui.nodeTreeView.selectionModel().selectedIndexes())
                        if nodes:
                            for n in nodes:
                                topic_names.extend(n.published)
                    else:
                        topics = self.currentMaster.topicsFromIndexes(self.currentMaster.ui.topicsView.selectionModel().selectedIndexes())
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
                                                             {'host': 'localhost',
                                                              'package': package,
                                                              'binary': binary,
                                                              'name': nm.nameres().normalize_name(node_name),
                                                              'args': args,
                                                              'masteruri': '%s' % self.currentMaster.master_state.uri,
                                                              'use_nmd': True,
                                                              'auto_pw_request': False
                                                             })
            except (Exception, nm.StartException) as e:
                import traceback
                print(utf8(traceback.format_exc(1)))
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
                    if self._sync_dialog.interface_filename is not None and not nm.is_local(host):
                        nmd_uri = nmdurl.nmduri(master.masteruri)
                        sync_file = nmdurl.join(nmdurl.nmduri(), self._sync_dialog.interface_filename)
                        # copy the interface file to remote machine
                        self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                            'Transfer sync interface to %s' % nmd_uri,
                                                            nm.starter().transfer_file_nmd,
                                                            {'grpc_url': '%s' % nmd_uri,
                                                             'path': sync_file,
                                                             'auto_pw_request': False,
                                                             'user': master.current_user
                                                            })
                    self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                        'Start sync on %s' % host,
                                                        nm.starter().runNodeWithoutConfig,
                                                        {'host': '%s' % host,
                                                         'package': 'fkie_master_sync',
                                                         'binary': 'master_sync',
                                                         'name': 'master_sync',
                                                         'args': self._sync_dialog.sync_args,
                                                         'masteruri': '%s' % master.masteruri,
                                                         'use_nmd': False,
                                                         'auto_pw_request': False,
                                                         'user': master.current_user
                                                        })
                    self._progress_queue_sync.start()
                except Exception:
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
                                                        {'host': utf8(host),
                                                         'package': 'fkie_master_sync',
                                                         'binary': 'master_sync',
                                                         'name': 'master_sync',
                                                         'args': default_sync_args,
                                                         'masteruri': utf8(master.masteruri),
                                                         'use_nmd': False,
                                                         'auto_pw_request': False,
                                                         'user': master.current_user
                                                        })
                    self._progress_queue_sync.start()
                except Exception:
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

    def on_daemon_start(self, masteruri=None):
        master = self.currentMaster
        if masteruri is not None:
            master = self.getMaster(masteruri, False)
        if master is not None:
            master.start_daemon()

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
                self.userComboBox.setEditText(self.currentMaster.current_user)
                if not master.is_valid_user_master_daemon():
                    self.showMasterName(masteruri, name, self.timestampStr(master.master_info.check_ts), master.master_state.online, 'daemon is running with different user: %s' % master.daemon_user)
            else:
                self.showMasterName('', 'No robot selected', None, False)
        if (current_time - self._refresh_time > 30.0):
            masteruri = self.getMasteruri()
            if masteruri is not None:
                master = self.getMaster(masteruri)
                if master is not None and master.master_state is not None and nm.settings().autoupdate:
                    self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri)
                self._refresh_time = current_time
        for _, master in self.masters.items():
            if master == self.currentMaster:
                master.perform_diagnostic_requests()
            elif int(current_time) % 3 == 0:
                master.perform_diagnostic_requests()

    def showMasterName(self, masteruri, name, timestamp, online=True, force_warning=''):
        '''
        Update the view of the info frame.
        '''
        con_err = ''
        user_warning = ''
        force_color_update = False
        if not force_warning:
            force_color_update = 'daemon is running with different user:' in self.masterInfoLabel.text()
        else:
            user_warning = '<span style=" color:red;">  %s</span>' % utf8(force_warning)
        try:
            tries = self._con_tries[masteruri]
            if tries > 1:
                con_err = '<span style=" color:red;">connection problems (%s tries)! </span>' % utf8(tries)
        except Exception:
            pass
        if self.__current_master_label_name != name or force_color_update:
            self.__current_master_label_name = name
            show_name = name if nm.settings().show_domain_suffix else subdomain(name)
            self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600; color:black">%s</span>' % show_name)
            color = QColor.fromRgb(nm.settings().host_color(self.__current_master_label_name, self._default_color.rgb()))
            self._new_color(color)
        ts = 'updated: %s' % utf8(timestamp) if timestamp is not None else ''
        if not nm.settings().autoupdate:
            ts = '%s<span style=" color:orange;"> AU off</span>' % ts
        self.masterInfoLabel.setText('<span style=" font-size:8pt; color:black">%s%s%s</span>' % (con_err, ts, user_warning))

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
        if not self.log_dock.isVisible() and self.log_dock.count_warn():
            if self.logButton.text():
                self.logButton.setIcon(self.__icons['log_warning'][0])
                self.logButton.setText('')
            else:
                self.logButton.setText('%d' % self.log_dock.count_warn())
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
            if m.online and m.master_state is not None and m.master_state.online:
                running_nodes.update(m.get_nodes_runningIfLocal())
        for _, m in self.masters.items():
            if m.master_state is not None:
                m.set_duplicate_nodes(running_nodes)

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
            if not nm.nmd().file.get_packages(item.master.uri):
                nm.nmd().file.list_packages_threaded(nmdurl.nmduri(item.master.uri))
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
        if self.currentMaster is not None:
            self.launch_dock.set_current_master(self.currentMaster.masteruri, self.currentMaster.master_state.name)
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
        except Exception:
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
        params_optional = {'Discovery type': {':type': 'string', ':value': ['master_discovery', 'zeroconf']},
                           'ROS Master Name': {':type': 'string', ':value': 'autodetect'},
                           'ROS Master URI': {':type': 'string', ':value': 'ROS_MASTER_URI'},
                           'Robot hosts': {':type': 'string', ':value': ''},
                           'Username': {':type': 'string', ':value': user_list},
                           'MCast Group': {':type': 'string', ':value': '226.0.0.0'},
                           'Heartbeat [Hz]': {':type': 'float', ':value': 0.5}
                           }
        params = {'Host': {':type': 'string', ':value': 'localhost'},
                  'Network(0..99)': {':type': 'int', ':value': str(self.mcast_port - 11511)},
                  'Start sync': {':type': 'bool', ':value': nm.settings().start_sync_with_discovery},
                  'Start daemon': {':type': 'bool', ':value': nm.settings().start_daemon_with_discovery},
                  'Optional Parameter': params_optional
                  }
        dia = ParameterDialog(params, sidebar_var='Host', store_geometry="start_robot_dialog")
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start discovery')
        dia.setFocusField('Host')
        if dia.exec_():
            try:
                params = dia.getKeywords(only_changed=False)
                hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
                port = params['Network(0..99)']
                start_sync = params['Start sync']
                start_daemon = params['Start daemon']
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
                        else:
                            nm.settings().set_host_user(hostname, usr)
                        muri = None if masteruri == 'ROS_MASTER_URI' else utf8(masteruri)
                        # stop if master_discovery already running
                        self._append_stop_for('/%s' % utf8(discovery_type), hostname, muri, self._progress_queue)
                        self._progress_queue.start()
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'start discovering on %s' % hostname,
                                                       nm.starter().runNodeWithoutConfig,
                                                       {'host': utf8(hostname),
                                                        'package': 'fkie_master_discovery',
                                                        'binary': utf8(discovery_type),
                                                        'name': utf8(discovery_type),
                                                        'args': args,
                                                        'masteruri': muri,
                                                        'use_nmd': False,
                                                        'auto_pw_request': False,
                                                        'user': usr
                                                       })

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
                                self._append_stop_for('/master_sync', hostname, muri, self._progress_queue_sync)
                                self._progress_queue_sync.start()
                                self._progress_queue_sync.add2queue(utf8(uuid.uuid4()),
                                                               'start sync on %s' % hostname,
                                                               nm.starter().runNodeWithoutConfig,
                                                               {'host': utf8(hostname),
                                                                'package': 'fkie_master_sync',
                                                                'binary': 'master_sync',
                                                                'name': 'master_sync',
                                                                'args': default_sync_args,
                                                                'masteruri': muri,
                                                                'use_nmd': False,
                                                                'auto_pw_request': False,
                                                                'user': usr
                                                               })
                            else:
                                if hostname not in self._syncs_to_start:
                                    self._syncs_to_start.append(hostname)
                        if start_daemon:
                            # local daemon will be started anyway
                            if not nm.is_local(hostname):
                                if hostname not in self._daemons_to_start:
                                    self._daemons_to_start.append(hostname)
                    except (Exception, nm.StartException) as e:
                        import traceback
                        print(traceback.format_exc(1))
                        rospy.logwarn("Error while start master_discovery for %s: %s" % (utf8(hostname), utf8(e)))
                        MessageBox.warning(self, "Start error",
                                           'Error while start master_discovery',
                                           utf8(e))
                    self._progress_queue.start()
                    self._progress_queue_sync.start()
            except Exception as e:
                MessageBox.warning(self, "Start error",
                                   'Error while parse parameter',
                                   utf8(e))

    def _append_stop_for(self, nodename, hostname, muri, queue):
        '''
        Appends stop command to given queue for given node
        '''
        cmuri = muri
        if hostname == 'localhost':
            lmuri = self.getMasteruri()
            if cmuri is None:
                cmuri = lmuri
            else:
                cmuri = cmuri.replace('localhost', get_hostname(lmuri))
        elif cmuri is None:
            cmuri = nm.nameres().masteruribyaddr(utf8(hostname))
        if cmuri is not None:
            master = self.getMaster(cmuri.rstrip('/') + '/', create_new=False)
            if master is not None:
                found_nodes = master._get_nodes_by_name([nodename])
                for node in found_nodes:
                    queue.add2queue(utf8(uuid.uuid4()), 'stop %s' % node.name, master.stop_node, {'node': node, 'force': True})

    def _join_network(self, network):
        try:
            master = self.getMaster(self.getMasteruri())
            if master is not None:
                # we need to stop master_discovery node first. In other case the new (and old) one will be stopped by ROS if one is running.
                master.stop_nodes_by_name(['/master_discovery'])
                time.sleep(0.5)
            hostname = 'localhost'
            args = []
            if network < 100 and network >= 0:
                args.append(''.join(['_mcast_port:=', utf8(11511 + int(network))]))
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'start discovering on ' + utf8(hostname),
                                           nm.starter().runNodeWithoutConfig,
                                           {'host': utf8(hostname),
                                            'package': 'fkie_master_discovery',
                                            'binary': 'master_discovery',
                                            'name': 'master_discovery',
                                            'args': args,
                                            'masteruri': None,
                                            'use_nmd': False,
                                            'auto_pw_request': False
                                           })
            self._progress_queue.start()
        except (Exception, nm.StartException) as e:
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
                                           {'host': '%s' % host})
            masteruris = nm.nameres().masterurisbyaddr(host)
            for masteruri in masteruris:
                master = self.getMaster(masteruri)
                master.stop_nodes_by_name(['/master_discovery'])
            self._progress_queue.start()
            self.on_description_update('Description', '')
            self.launch_dock.raise_()
        except (Exception, nm.StartException) as e:
            rospy.logwarn("Error while poweroff %s: %s", host, utf8(e))
            MessageBox.warning(self, "Run error",
                               'Error while poweroff %s' % host,
                               '%s' % utf8(e))

    def rosclean(self, masteruri):
        try:
            host = get_hostname(masteruri)
            nuri = nmdurl.nmduri(masteruri)
            ret = MessageBox.warning(self, "ROS Node Manager",
                                     "Do you really want delete all logs on `%s`?" % host,
                                     buttons=MessageBox.Ok | MessageBox.Cancel)
            if ret == MessageBox.Cancel:
                return
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'rosclean `%s`' % nuri,
                                           nm.starter().rosclean,
                                           {'grpc_uri': '%s' % nuri})
            master = self.getMaster(masteruri, create_new=False)
            if master is not None:
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                            'update `%s`' % nuri,
                                            master.perform_nmd_requests,
                                            {})
            self._progress_queue.start()
            self.launch_dock.raise_()
        except (Exception, nm.StartException) as e:
            rospy.logwarn("Error while rosclean %s: %s", masteruri, utf8(e))
            MessageBox.warning(self, "Run error",
                               'Error while rosclean %s' % masteruri,
                               '%s' % utf8(e))

# ======================================================================================================================
# Handling of the launch view signals
# ======================================================================================================================

    def on_load_launch_file(self, path, args={}, masteruri=None):
        '''
        Load the launch file. A ROS master must be selected first.
        :param str path: the path of the launch file.
        '''
        master_proxy = None
        if masteruri is not None:
            master_proxy = self.getMaster(masteruri, False)
        if master_proxy is None:
            master_proxy = self.stackedLayout.currentWidget()
        if isinstance(master_proxy, MasterViewProxy):
            try:
                master_proxy.launchfiles = (path, args)
            except Exception as e:
                import traceback
                print(utf8(traceback.format_exc(1)))
                MessageBox.warning(self, "Loading launch file", path, '%s' % utf8(e))
#      self.setCursor(cursor)
        else:
            MessageBox.information(self, "Load of launch file", "Select a master first!",)

    def on_launch_edit(self, grpc_path, search_text='', trynr=1):
        '''
        Opens the given path in an editor. If file is already open, select the editor.
        If search text is given, search for the text in files an goto the line.
        :param str grpc_path: path with grpc prefix
        :param str search_text: A string to search in file
        '''
        if grpc_path:
            if grpc_path in self.editor_dialogs:
                try:
                    self.editor_dialogs[grpc_path].on_load_request(grpc_path, search_text, only_launch=True)
                    self.editor_dialogs[grpc_path].raise_()
                    self.editor_dialogs[grpc_path].activateWindow()
                except Exception:
                    if trynr > 1:
                        raise
                    import traceback
                    print(traceback.format_exc())
                    del self.editor_dialogs[grpc_path]
                    self.on_launch_edit(grpc_path, search_text, 2)
            else:
                editor = Editor([grpc_path], search_text, master_name=self.launch_dock.path2mastername(grpc_path))
                if editor.tabWidget.count() > 0:
                    self.editor_dialogs[grpc_path] = editor
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
        :type file: [str]
        '''
        # use node manager daemon
        if files:
            nmd_url = nmdurl.nmduri()
            if self.currentMaster is not None:
                nmd_url = get_hostname(self.currentMaster.masteruri)
            params = {'master': {':type': 'string', ':value': self.currentMaster.masteruri},
                      'recursive': {':type': 'bool', ':value': False}
                      }
            dia = ParameterDialog(params, store_geometry="launch_transfer_dialog")
            dia.setFilterVisible(False)
            dia.setWindowTitle('Transfer file')
            dia.setFocusField('master')
            if dia.exec_():
                try:
                    params = dia.getKeywords()
                    nmd_url = params['master']
                    recursive = params['recursive']
                    for path in files:
                        nmd_url = nmdurl.nmduri(nmd_url)
                        rospy.loginfo("TRANSFER to %s: %s" % (nmd_url, path))
                        self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                                  'transfer files to %s' % nmd_url,
                                                                  nm.starter().transfer_file_nmd,
                                                                  {'grpc_url': '%s' % nmd_url,
                                                                   'path': path,
                                                                   'auto_pw_request': False
                                                                  })
                        if recursive:
                            self.launch_dock.progress_queue.add2queue('%s' % uuid.uuid4(),
                                                                      "transfer recursive '%s' to %s" % (path, nmd_url),
                                                                      self._recursive_transfer,
                                                                      {'path': path, 'nmd_url': nmd_url})
                    self.launch_dock.progress_queue.start()
                except Exception as e:
                    MessageBox.warning(self, "Transfer error",
                                       'Error while transfer files', '%s' % utf8(e))

    def _recursive_transfer(self, path, nmd_url):
        includes = nm.nmd().launch.get_included_files_set(path, True, search_in_ext=nm.settings().SEARCH_IN_EXT)
        copy_set = set()
        for inc_file in includes:
            copy_set.add(inc_file)
        for cppath in copy_set:
            self.launch_dock.progress_queue.add2queue(utf8(uuid.uuid4()),
                                                      'transfer file %s to %s' % (cppath, nmd_url),
                                                      nm.starter().transfer_file_nmd,
                                                      {'grpc_url': '%s' % nmd_url,
                                                       'path': cppath
                                                      })
        self.launch_dock.progress_queue.start()

    def _reload_globals_at_next_start(self, launch_file):
        if self.currentMaster is not None:
            self.currentMaster.reload_global_parameter_at_next_start(launch_file)

# ======================================================================================================================
# Change file detection
# ======================================================================================================================

    def changeEvent(self, event):
        '''
        '''
        if self.isActiveWindow() and self.isActiveWindow() != self._last_window_state:
            if hasattr(self, 'currentMaster') and self.currentMaster is not None:
                # perform delayed checks for changed files or multiple screens
                QTimer.singleShot(250, self.currentMaster.perform_master_checks)
        self._last_window_state = self.isActiveWindow()
        QMainWindow.changeEvent(self, event)

    def enterEvent(self, event):
        '''
        Check for changed files, if the main gui was entered.
        '''
        QMainWindow.enterEvent(self, event)

# ======================================================================================================================
# Capabilities handling
# ======================================================================================================================

    def on_start_nodes(self, masteruri, cfg, nodes):
        if masteruri is not None:
            master = self.getMaster(masteruri)
            master.start_nodes_by_name(nodes, cfg)

    def on_stop_nodes(self, masteruri, nodes):
        if masteruri is not None:
            master = self.getMaster(masteruri)
            master.stop_nodes_by_name(nodes)

    def on_description_update(self, title, text, force=False):
        # ignore updates if we are currently browse in text dialog
        if self._description_accept:
            if self._description_accept != title:
                if not force:
                    return
                elif not title.endswith(' diagnostic'):  # add 'back'-link if title ends with ' diagnostic'
                    self._description_accept = ''
        wtitle = self.descriptionDock.windowTitle().replace('&', '')
        same_title = wtitle == title
        valid_sender = self.sender() == self.currentMaster or not isinstance(self.sender(), MasterViewProxy)
        no_focus = not self.descriptionTextEdit.hasFocus()
        if (valid_sender) and (same_title or no_focus or self._accept_next_update):
            self._accept_next_update = False
            # _description_accept is set to True on click on link of {node, topic, service}
            if not same_title:
                if self._description_accept:
                    self._description_history.append((wtitle, self.descriptionTextEdit.toHtml()))
                else:
                    del self._description_history[:]
            # prepend 'back' link the text
            if self._description_history:
                if len(self._description_history) > 15:
                    self._description_history.pop(0)
                text = '<a href="back://" title="back">back</a>%s' % text
            self.descriptionDock.setWindowTitle(title)
            vbar = self.descriptionTextEdit.verticalScrollBar()
            stored_vpos = vbar.value()
            self.descriptionTextEdit.setText(text)
            vbar.setValue(stored_vpos)
            if text and force:
                self.descriptionDock.raise_()

    def on_description_update_cap(self, title, text):
        self.descriptionDock.setWindowTitle(title)
        self.descriptionTextEdit.setText(text)

    def on_description_anchorClicked(self, url):
        self._description_accept = self.descriptionDock.windowTitle().replace('&', '')
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
                self._description_accept = self._url_path(url)
                self.currentMaster.on_node_selection_changed(None, None, True, self._description_accept)
        elif url.toString().startswith('topic://'):
            if self.currentMaster is not None:
                self._description_accept = self._url_path(url)
                self.currentMaster.on_topic_selection_changed(None, None, True, self._description_accept)
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
                self._description_accept = self._url_path(url)
                self.currentMaster.on_service_selection_changed(None, None, True, self._description_accept)
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
        elif url.toString().startswith('kill-pid://pid'):
            if self.currentMaster is not None:
                self.currentMaster.on_kill_pid(int(url.toString().replace('kill-pid://pid', '')))
        elif url.toString().startswith('kill-screen://'):
            if self.currentMaster is not None:
                self.currentMaster.on_kill_screens()
        elif url.toString().startswith('copy-log-path://'):
            if self.currentMaster is not None:
                self.currentMaster.on_log_path_copy()
        elif url.toString().startswith('copy://'):
            QApplication.clipboard().setText(url.toString().replace('copy://', ''))
        elif url.toString().startswith('launch://'):
            self.on_launch_edit(self._url_path(url), '')
        elif url.toString().startswith('reload-globals://'):
            self._reload_globals_at_next_start(url.toString().replace('reload-globals://', 'grpc://'))
        elif url.toString().startswith('poweroff://'):
            self.poweroff_host(self._url_host(url))
        elif url.toString().startswith('rosclean://'):
            self.rosclean(url.toString().replace('rosclean', 'http'))
        elif url.toString().startswith('sysmon-switch://'):
            self.sysmon_active_update(url.toString().replace('sysmon-switch', 'http'))
        elif url.toString().startswith('nmd-cfg://'):
            self.nmd_cfg(url.toString().replace('nmd-cfg', 'http'))
        elif url.toString().startswith('nm-cfg://'):
            self._on_settings_button_clicked()
        elif url.toString().startswith('show-all-diagnostics://'):
            if self.currentMaster is not None:
                self.currentMaster.show_diagnostic_messages(self._url_path(url))
        elif url.toString().startswith('open-edit://'):
            self.on_launch_edit(url.toString().replace('open-edit://', 'grpc://'))
        elif url.toString().startswith('show-log://'):
            if self.currentMaster is not None:
                try:
                    dest = url.toString().replace('show-log://', '').split('@')
                    self.currentMaster.show_log(dest[0], dest[1], roslog=False)
                except Exception as e:
                    print(e)
        elif url.toString().startswith('show-roslog://'):
            if self.currentMaster is not None:
                try:
                    dest = url.toString().replace('show-roslog://', '').split('@')
                    self.currentMaster.show_log(dest[0], dest[1], roslog=True)
                except Exception as e:
                    print(e)
        elif url.toString().startswith('back://'):
            if self._description_history:
                # show last discription on click on back
                title, text = self._description_history[-1]
                self._description_accept = title
                del self._description_history[-1]
                self.descriptionDock.setWindowTitle(title)
                self.descriptionTextEdit.setText(text)
        else:
            try:
                from python_qt_binding.QtGui import QDesktopServices
                QDesktopServices.openUrl(url)
            except Exception as err:
                rospy.logwarn("can't open url %s: %s" % (url, err))
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

    def _restart_nodes_g(self):
        if self.currentMaster is not None:
            self.currentMaster.on_force_start_nodes(True)

    def keyPressEvent(self, event):
        '''
        '''
        QMainWindow.keyPressEvent(self, event)
        if event == QKeySequence.Find:
            focus_widget = QApplication.focusWidget()
            if not isinstance(focus_widget, EnhancedLineEdit):
                # set focus to filter line
                if self.currentMaster is not None:
                    self.currentMaster.focus_filter_line()

    def _show_section_menu(self, event=None):
        # self._timer_alt = None
        if self._select_index == 0:
            if self.currentMaster is not None:
                if self.currentMaster._is_current_tab_name('tabNodes'):
                    self.currentMaster.ui.nodeTreeView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabTopics'):
                    self.currentMaster.ui.topicsView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabServices'):
                    self.currentMaster.ui.servicesView.setFocus(Qt.TabFocusReason)
                elif self.currentMaster._is_current_tab_name('tabParameter'):
                    self.currentMaster.ui.parameterView.setFocus(Qt.TabFocusReason)
        elif self._select_index == 1:
            self.launch_dock.raise_()
            self.launch_dock.ui_file_view.setFocus(Qt.TabFocusReason)
        elif self._select_index == 2:
            self.descriptionDock.raise_()
            self.descriptionTextEdit.setFocus(Qt.TabFocusReason)
        elif self._select_index == 3:
            self.startRobotButton.setFocus(Qt.TabFocusReason)
        elif self._select_index == 4:
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
        if self.currentMaster is not None and self.currentMaster.ui.nodeTreeView.hasFocus():
            if event.key() == Qt.Key_F4 and not key_mod:
                if self.currentMaster.ui.editConfigButton.isEnabled():
                    self.currentMaster.on_edit_config_clicked()
                elif self.currentMaster.ui.editRosParamButton.isEnabled():
                    self.currentMaster.on_edit_rosparam_clicked()
            elif event.key() == Qt.Key_F3 and not key_mod and self.currentMaster.ui.ioButton.isEnabled():
                self.currentMaster.on_io_clicked()
        QMainWindow.keyReleaseEvent(self, event)

    def image_mouseDoubleClickEvent(self, event):
        '''
        Set the robot image
        '''
        if self.currentMaster:
            try:
                if not os.path.isdir(nm.settings().robots_path):
                    os.makedirs(nm.settings().robots_path)
                (fileName, _) = QFileDialog.getOpenFileName(self,
                                                            "Set robot image",
                                                            nm.settings().robots_path,
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
        One of the robot icons was changed. Update the icon.
        '''
        master = self.getMaster(masteruri, False)
        if master:
            self._assigne_icon(master.mastername, resolve_url(path))

    def _callback_system_diagnostics(self, data, grpc_url=''):
        try:
            muri = nmdurl.masteruri(grpc_url)
            master = self.getMaster(muri, create_new=False)
            if master:
                master.update_system_diagnostics(data)
                self.master_model.update_master_diagnostic(nm.nameres().mastername(muri), data)
        except Exception as err:
            rospy.logwarn('Error while process system diagnostic messages: %s' % utf8(err))

    def _callback_diagnostics(self, data, grpc_url=''):
        try:
            for diagnostic in data.status:
                self.diagnostics_signal.emit(diagnostic)
        except Exception as err:
            rospy.logwarn('Error while process diagnostic messages: %s' % utf8(err))

    def _callback_username(self, username, grpc_url=''):
        try:
            muri = nmdurl.masteruri(grpc_url)
            master = self.getMaster(muri, create_new=False)
            if master:
                master.daemon_user = username
        except Exception as err:
            rospy.logwarn('Error while process username from daemon: %s' % utf8(err))

    def sysmon_active_update(self, masteruri):
        master = self.getMaster(masteruri, create_new=False)
        if master is not None:
            master.sysmon_active_update()

    def nmd_cfg(self, masteruri):
        nmd_uri = nmdurl.nmduri(masteruri)
        nm.nmd().settings.get_config_threaded(nmd_uri)

    def _nmd_yaml_cfg(self, data, nmdurl):
        params = {}
        try:
            params = ruamel.yaml.load(data, Loader=ruamel.yaml.Loader)
        except Exception as err:
            rospy.logwarn("Error while parse daemon configuration: %s" % utf8(err))
        dia = ParameterDialog(params, store_geometry="nmd_cfg_dialog")
        dia.setWindowTitle('Daemon Configuration')
        dia.setFocusField('load_warn_level')
        if dia.exec_():
            try:
                params = dia.getKeywords(with_tags=True)
                buf = ruamel.yaml.compat.StringIO()
                ruamel.yaml.dump(params, buf, Dumper=ruamel.yaml.RoundTripDumper)
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                '%s: set configuration for daemon' % nmdurl,
                                                nm.nmd().settings.set_config,
                                                {'grpc_url': nmdurl,
                                                 'data': buf.getvalue()
                                                })
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                '%s: get system diagnostics' % nmdurl,
                                                nm.nmd().monitor.get_system_diagnostics_threaded,
                                                {'grpc_url': nmdurl})
                self._progress_queue.start()
            except Exception as err:
                import traceback
                print(traceback.format_exc())
                MessageBox.warning(self, "Daemon configuration error",
                                    'Error while parse parameter',
                                    '%s' % utf8(err))

    def _throttle_nmd_errrors(self, reason, url, error, delay=60):
        now = time.time()
        doprint = False
        try:
            key = (reason, url, error.details())
        except Exception:
            key = (reason, url, utf8(error))
        if key not in self._nmd_last_errors.keys():
            doprint = True
        elif now - self._nmd_last_errors[key] > delay:
            doprint = True
        if doprint:
            rospy.logwarn("Error while %s from %s: %s" % (reason, url, utf8(error)))
            self._nmd_last_errors[key] = now
        if now - self._ts_nmd_error_last_check > 120:
            # clean old messages
            self._ts_nmd_error_last_check = now
            for key, ts in self._nmd_last_errors.items():
                if now - ts > 240:
                    del self._nmd_last_errors[key]

    def on_nmd_err(self, method, url, path, error):
        '''
        Handles the error messages from node_manager_daemon.

        :param str method: name of the method caused this error.
        :param str url: the URI of the node manager daemon.
        :param Exception error: on occurred exception.
        '''
        muri = nmdurl.masteruri(url)
        master = self.getMaster(muri, False)
        if master is not None and not master._has_nmd:
            # no daemon for this master available, ignore errors
            return
        reason = method
        if method == '_get_nodes':
            reason = 'get launch configuration'
        self._throttle_nmd_errrors(reason, url, error, 60)
        if hasattr(error, 'code'):
            if error.code() == grpc.StatusCode.UNIMPLEMENTED:
                muri = nmdurl.masteruri(url)
                master = self.getMaster(muri, create_new=False)
                if master:
                    self.master_model.add_master_error(nm.nameres().mastername(muri), 'node_manager_daemon has unimplemented methods! Please update!')
                    master.set_diagnostic_warn('/node_manager_daemon', 'unimplemented methods detected! Please update!')

# ======================================================================================================================
# Help site handling
# ======================================================================================================================

    def _on_help_go_back(self):
        self._on_help_link_clicked(QUrl(''), history_idx=-1)

    def _on_help_go_home(self):
        self._on_help_link_clicked(self._help_root_url)

    def _on_help_go_forward(self):
        self._on_help_link_clicked(QUrl(''), history_idx=1)

    def _on_help_link_clicked(self, link, history_idx=0):
        if link.isEmpty():
            # read from history if given link is empty
            try:
                link = self._help_history[self._help_history_idx + history_idx]
                self._help_history_idx += history_idx
            except Exception:
                pass
        if not link.isEmpty():
            if history_idx == 0:
                # it was not a history request -> add link to history
                current_link = self.ui_help_web_view.url()
                if current_link != link:
                    # if we navigate in the history previously remove forward items
                    if len(self._help_history) - 1 > self._help_history_idx:
                        self._help_history = self._help_history[:self._help_history_idx + 1]
                    self._help_history_idx += 1
                    self._help_history.append(link)
            if link.scheme() == 'file':
                try:
                    fpath = link.toLocalFile()
                    if fpath.endswith('.rst'):
                        # render .rst files
                        with open(fpath) as f:
                            self.ui_help_web_view.setHtml(examples.html_body(utf8(f.read())), link)
                    else:
                        self.ui_help_web_view.setUrl(link)
                except Exception:
                    import traceback
                    msg = "Error while generate help: %s" % traceback.format_exc(2)
                    rospy.logwarn(msg)
            else:
                QDesktopServices.openUrl(link)
        # update navigation buttons
        self.ui_help_back.setEnabled(self._help_history_idx > 0)
        self.ui_help_forward.setEnabled(self._help_history_idx < len(self._help_history) - 1)

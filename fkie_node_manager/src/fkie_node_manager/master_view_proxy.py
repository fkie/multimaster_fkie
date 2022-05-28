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
from python_qt_binding.QtCore import QRegExp, Qt, QTimer, QSize, Signal
from python_qt_binding.QtGui import QKeySequence  # , QBrush, QPen
from rosgraph.names import is_legal_name
import getpass
import os
import roslib
import rospy
import ruamel.yaml
import socket
import time
import traceback
import threading
import uuid
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient


from fkie_master_discovery.common import masteruri_from_ros
from fkie_master_discovery.master_info import NodeInfo
from fkie_node_manager_daemon.common import interpret_path, sizeof_fmt, isstring, utf8
from fkie_node_manager_daemon.host import get_hostname, get_port
from fkie_node_manager_daemon import exceptions
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon.version import detect_version
from .common import package_name
from .detailed_msg_box import MessageBox, DetailedError
from .echo_dialog import EchoDialog
from .html_delegate import HTMLDelegate
from .launch_config import LaunchConfig  # , LaunchConfigException
from .launch_enhanced_line_edit import EnhancedLineEdit
from .launch_server_handler import LaunchServerHandler
from .message_frame import MessageData, MessageFrame
from .node_tree_model import NodeTreeModel, NodeItem, GroupItem, HostItem, NodeInfoIconsDelegate
from .parameter_dialog import ParameterDialog, MasterParameterDialog, ServiceDialog
from .parameter_handler import ParameterHandler
from .parameter_list_model import ParameterModel, ParameterNameItem, ParameterValueItem
from .progress_queue import ProgressQueue  # , InteractionNeededError, ProgressThread
from .select_dialog import SelectDialog
from .service_list_model import ServiceModel, ServiceItem, ServiceGroupItem
from .supervised_popen import SupervisedPopen
from .topic_list_model import TopicModel, TopicItem, TopicGroupItem
import fkie_node_manager as nm
try:
    from python_qt_binding.QtGui import QAction, QFileDialog, QFrame, QMenu, QShortcut, QWidget
    from python_qt_binding.QtGui import QApplication, QVBoxLayout
except Exception:
    from python_qt_binding.QtWidgets import QAction, QFileDialog, QFrame, QMenu, QShortcut, QWidget
    from python_qt_binding.QtWidgets import QApplication, QVBoxLayout


try:
    from python_qt_binding.QtGui import QItemSelection, QItemSelectionModel, QItemSelectionRange, QSortFilterProxyModel
except Exception:
    from python_qt_binding.QtCore import QItemSelection, QItemSelectionModel, QItemSelectionRange, QSortFilterProxyModel

# from python_qt_binding import QtUiTools
try:
    from diagnostic_msgs.msg import DiagnosticStatus
    DIAGNOSTICS_AVAILABLE = True
except Exception:
    DIAGNOSTICS_AVAILABLE = False


class MasterViewProxy(QWidget):
    '''
    This class stores the informations about a ROS master and shows it on request.
    '''

    updateHostRequest = Signal(str)
    host_description_updated = Signal(str, str, str)
    ''':ivar str,str,str host_description_updated: the signal is emitted on description changes and contains the ROS Master URI, host address and description a parameter.'''

    capabilities_update_signal = Signal(str, str, str, list)
    ''':ivar str,str,str,list(fkie_node_manager_daemon.launch_description.RobotDescription) capabilities_update_signal: the signal is emitted if a description with capabilities is received and has the ROS master URI, host address, the name of the config and a list with descriptions.'''
    remove_config_signal = Signal(str)
    ''':ivar str remove_config_signal: the signal is emitted if a default_cfg was removed'''

    description_signal = Signal(str, str, bool)
    ''':ivar str,str,bool description_signal: the signal is emitted to show a description (title, description)'''

    request_xml_editor = Signal(str, str)
    ''':ivar str,str request_xml_editor: the signal to open a xml editor dialog (configuration path, search text)'''

    stop_nodes_signal = Signal(str, list)
    ''':ivar str,list(str) stop_nodes_signal: the signal is emitted to stop on masteruri the nodes described in the list.'''

    robot_icon_updated = Signal(str, str)
    ''':ivar str, str robot_icon_updated: the signal is emitted, if the robot icon was changed by a configuration (masteruri, path)'''

    loaded_config = Signal(object, list)
    ''':ivar LaunchConfig,list(str) loaded_config: the signal is emitted, after a launchfile is successful loaded (LaunchConfig, [changed nodes (str)])'''

    DIAGNOSTIC_LEVELS = {0: 'OK',
                         1: 'WARN',
                         2: 'ERROR',
                         3: 'STALE',
                         4: 'UNKNOWN',
                         5: 'UNKNOWN'}

    def __init__(self, masteruri, parent=None):
        '''
        Creates a new master.

        :param str masteruri: the URI of the ROS master
        '''
        QWidget.__init__(self, parent)
        self.setObjectName(' - '.join(['MasterViewProxy', masteruri]))
        self.masteruri = masteruri
        self.mastername = masteruri
        self.main_window = parent
        try:
            self.mastername = get_hostname(self.masteruri)
        except Exception:
            pass

        self._tmpObjects = []
        self.__master_state = None
        self.__master_info = None
        self.__force_update = False
        self.__configs = dict()  # file name (str): LaunchConfig
        self._loaded_args = dict()
#        self.__config_mtimes = dict()  # file name (str): mtime(float)
#        self.__config_includes = dict()  # file name (str): dict(included file(str): mtime(float)))
        self.__expanded_items = dict()  # [file name] : list of expanded nodes
        self.__online = False
        self.__run_id = ''
#    self.rosconfigs = dict() # [launch file path] = LaunchConfig()
        self.__in_question = []  # stores the changed files, until the user is interacted
#    self.__uses_confgs = dict() # stores the decisions of the user for used configuration to start of node
        ''':ivar list(str) __in_question: stored the question dialogs for changed files '''
        self._stop_ignores = ['rosout', rospy.get_name(), 'node_manager', 'node_manager_daemon', 'master_discovery', 'master_sync', 'default_cfg', 'zeroconf']
        self.__echo_topics_dialogs = set()  # set with subscibed topics
        self.__last_info_text = None
        self.__use_sim_time = False
        self.__current_user = nm.settings().host_user(self.mastername)
        self.__daemon_user = ''
        self.__robot_icons = []
        self.__current_robot_icon = None
        self.__current_parameter_robot_icon = ''
        self.__republish_params = {}  # { topic : params, created by dialog}
        self.__current_icon_height = 8
        self.__last_selection = 0
        self.__last_node_activation = 0
        self.__last_question_start_nmd = 0
        self._on_stop_kill_roscore = False
        self._on_stop_poweroff = False
        self._start_nodes_after_load_cfg = dict()
        self._cfg_changed_nodes = dict()
        self._stored_diagnostic_messages = dict()  # dict(time in seconds: diagnostic_status)
        # store the running_nodes to update to duplicates after load a launch file
        self.__running_nodes = dict()  # dict (node name : masteruri)
        self._nodelets = dict()  # dict(launchfile: dict(nodelet manager: list(nodes))
        self._associations_lock = threading.RLock()
        self._associations = dict()  # dict(launchfile: dict(node: list(nodes))
        self._first_launch = True
        self._has_nmd = False
        self._changed_binaries = dict()
        self.default_load_launch = ''
        self._nmd_version, self._nmd_date = detect_version('fkie_node_manager_daemon')
        self._diag_nmd_version = None
        self._diag_log_dir_size = None
        self._timer_nmd_request = QTimer()
        self._timer_nmd_request.timeout.connect(self._sysmon_update_callback)
        self._timer_nmd_request.setSingleShot(True)
        self._ts_last_diagnostic_request = 0
        self._has_diagnostics = False
        self.has_master_sync = False
        self.restarting_daemon = False
        self.reloading_files = {}
#         self.default_cfg_handler = DefaultConfigHandler()
#         self.default_cfg_handler.node_list_signal.connect(self.on_default_cfg_nodes_retrieved)
#         self.default_cfg_handler.description_signal.connect(self.on_default_cfg_descr_retrieved)
#         self.default_cfg_handler.err_signal.connect(self.on_default_cfg_err)

        self.__launch_servers = {}  # uri : (pid, nodes)
        self.launch_server_handler = LaunchServerHandler()
        self.launch_server_handler.launch_server_signal.connect(self.on_launch_server_retrieved)
        self.launch_server_handler.error_signal.connect(self.on_launch_server_err)

        self.ui = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'MasterTab.ui')
        loadUi(ui_file, self.ui, custom_widgets={'EnhancedLineEdit': EnhancedLineEdit})
        tabLayout = QVBoxLayout(self)
        tabLayout.setContentsMargins(0, 0, 0, 0)
        tabLayout.addWidget(self.ui)
        # set icons
        self.ui.progressCancelPrioButton.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.ui.progressCancelButton.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.ui.startButton.setIcon(nm.settings().icon('sekkyumu_play.png'))
        self.ui.stopButton.setIcon(nm.settings().icon('sekkyumu_stop.png'))
        self.ui.ioButton.setIcon(nm.settings().icon('crystal_clear_show_io.png'))
        self.ui.logButton.setIcon(nm.settings().icon('crystal_clear_show_log.png'))
        self.ui.logDeleteButton.setIcon(nm.settings().icon('crystal_clear_show_delete_log.png'))
        self.ui.dynamicConfigButton.setIcon(nm.settings().icon('crystal_clear_dyncfg.png'))
        self.ui.editRosParamButton.setIcon(nm.settings().icon('default_cfg_edit.png'))
        self.ui.editConfigButton.setIcon(nm.settings().icon('crystal_clear_edit_launch.png'))
        self.ui.closeCfgButton.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.ui.echoTopicButton.setIcon(nm.settings().icon('sekkyumu_topic_echo.png'))
        self.ui.hzTopicButton.setIcon(nm.settings().icon('sekkyumu_topic_hz.png'))
        self.ui.hzSshTopicButton.setIcon(nm.settings().icon('sekkyumu_topic_echo_ssh_hz.png'))
        self.ui.pubTopicButton.setIcon(nm.settings().icon('sekkyumu_topic_pub.png'))
        self.ui.pubStopTopicButton.setIcon(nm.settings().icon('sekkyumu_topic_pub_stop.png'))
        self.ui.callServiceButton.setIcon(nm.settings().icon('sekkyumu_call_service.png'))
        self.ui.getParameterButton.setIcon(nm.settings().icon('crystal_clear_action_db_update.png'))
        self.ui.addParameterButton.setIcon(nm.settings().icon('crystal_clear_action_db_add.png'))
        self.ui.deleteParameterButton.setIcon(nm.settings().icon('crystal_clear_action_db_remove.png'))
        self.ui.saveParameterButton.setIcon(nm.settings().icon('save.png'))
        self.ui.transferParameterButton.setIcon(nm.settings().icon('crystal_clear_launch_file_transfer.png'))

        self._progress_queue_prio = ProgressQueue(self.ui.progressPrioFrame, self.ui.progressPrioBar, self.ui.progressCancelPrioButton, 'Prio Master - %s' % self.mastername)
        self._progress_queue = ProgressQueue(self.ui.progressFrame, self.ui.progressBar, self.ui.progressCancelButton, 'Master - %s' % self.mastername)
        self._progress_queue_prio.no_screen_error_signal.connect(self._on_no_screen_error)
        self._progress_queue.no_screen_error_signal.connect(self._on_no_screen_error)

        # setup the node view
        self.node_tree_model = NodeTreeModel(nm.nameres().address(self.masteruri), self.masteruri)
        self.node_proxy_model = NodesSortFilterProxyModel(self)
        self.node_proxy_model.setSourceModel(self.node_tree_model)
        self.ui.nodeTreeView.setModel(self.node_proxy_model)
        self.node_tree_model.hostInserted.connect(self.on_host_inserted)
        for i, (_, width) in enumerate(NodeTreeModel.header):  # _:=name
            self.ui.nodeTreeView.setColumnWidth(i, width)
        check_for_ros_names = not nm.settings().group_nodes_by_namespace
        self.nodeNameDelegate = HTMLDelegate(check_for_ros_names=check_for_ros_names, dec_ascent=True, is_node=True, palette=self.palette())
        self.ui.nodeTreeView.setItemDelegateForColumn(0, self.nodeNameDelegate)
        self.node_delegate = NodeInfoIconsDelegate()
        self.ui.nodeTreeView.setItemDelegateForColumn(1, self.node_delegate)
        # self.ui.nodeTreeView.collapsed.connect(self.on_node_collapsed)
        self.ui.nodeTreeView.expanded.connect(self.on_node_expanded)
        sm = self.ui.nodeTreeView.selectionModel()
        sm.selectionChanged.connect(self.on_node_selection_changed)
        self.ui.nodeTreeView.activated.connect(self.on_node_activated)
        self.ui.nodeTreeView.clicked.connect(self.on_node_clicked)
#    self.ui.nodeTreeView.setAcceptDrops(True)
#    self.ui.nodeTreeWidget.setSortingEnabled(True)

        # setup the topic view
        self.topic_model = TopicModel()
        self.topic_proxyModel = TopicsSortFilterProxyModel(self)
        self.topic_proxyModel.setSourceModel(self.topic_model)
        self.ui.topicsView.setModel(self.topic_proxyModel)
        self.ui.topicsView.expandAll()
        self.ui.topicsView.sortByColumn(0, Qt.AscendingOrder)
#    self.ui.topicsView.setModel(self.topic_model)
        for i, (_, width) in enumerate(TopicModel.header):  # _:=name
            self.ui.topicsView.setColumnWidth(i, width)
        self.topicNameDelegate = HTMLDelegate(check_for_ros_names=check_for_ros_names, dec_ascent=True, is_node=True, palette=self.palette())
        self.topicTypeDelegate = HTMLDelegate(dec_ascent=True)
        self.ui.topicsView.setItemDelegateForColumn(0, self.topicNameDelegate)
        self.ui.topicsView.setItemDelegateForColumn(3, self.topicTypeDelegate)
        sm = self.ui.topicsView.selectionModel()
        sm.selectionChanged.connect(self.on_topic_selection_changed)
        self.ui.topicsView.activated.connect(self.on_topic_activated)
        self.ui.topicsView.clicked.connect(self.on_topic_clicked)
        self.ui.topicsView.setSortingEnabled(True)

        # setup the service view
        self.service_model = ServiceModel()
        self.service_proxyModel = ServicesSortFilterProxyModel(self)
        self.service_proxyModel.setSourceModel(self.service_model)
        self.ui.servicesView.setModel(self.service_proxyModel)
        self.ui.servicesView.expandAll()
        self.ui.servicesView.sortByColumn(0, Qt.AscendingOrder)
        for i, (_, width) in enumerate(ServiceModel.header):  # _:=name
            self.ui.servicesView.setColumnWidth(i, width)
        self.serviceNameDelegate = HTMLDelegate(check_for_ros_names=check_for_ros_names, dec_ascent=True, is_node=True, palette=self.palette())
        self.serviceTypeDelegate = HTMLDelegate(dec_ascent=True)
        self.ui.servicesView.setItemDelegateForColumn(0, self.serviceNameDelegate)
        self.ui.servicesView.setItemDelegateForColumn(1, self.serviceTypeDelegate)
        sm = self.ui.servicesView.selectionModel()
        sm.selectionChanged.connect(self.on_service_selection_changed)
        self.ui.servicesView.activated.connect(self.on_service_activated)
        self.ui.servicesView.clicked.connect(self.on_service_clicked)
        self.ui.servicesView.setSortingEnabled(True)

        # setup the parameter view
        self.parameter_model = ParameterModel()
        self.parameter_model.itemChanged.connect(self._on_parameter_item_changed)
        self.parameter_proxyModel = ParameterSortFilterProxyModel(self)
        self.parameter_proxyModel.setSourceModel(self.parameter_model)
        self.ui.parameterView.setModel(self.parameter_proxyModel)
        for i, (_, width) in enumerate(ParameterModel.header):  # _:=name
            self.ui.parameterView.setColumnWidth(i, width)
        self.parameterNameDelegate = HTMLDelegate(dec_ascent=True, palette=self.palette())
        self.ui.parameterView.setItemDelegateForColumn(0, self.parameterNameDelegate)
        sm = self.ui.parameterView.selectionModel()
        sm.selectionChanged.connect(self.on_parameter_selection_changed)
        self.ui.parameterView.setSortingEnabled(True)

#    self.parameter_proxyModel.filterAcceptsRow = _filterParameterAcceptsRow
#    self.ui.parameterView.activated.connect(self.on_service_activated)

        # connect the buttons
        self.ui.startButton.clicked.connect(self.on_start_clicked)
        self.ui.stopButton.clicked.connect(self.on_stop_clicked)
#    self.ui.stopContextButton.toggled.connect(self.on_stop_context_toggled)
        self.ui.ioButton.clicked.connect(self.on_io_clicked)
        self.ui.logButton.clicked.connect(self.on_log_clicked)
        self.ui.logDeleteButton.clicked.connect(self.on_log_delete_clicked)
        self.ui.dynamicConfigButton.clicked.connect(self.on_dynamic_config_clicked)
        self.ui.editConfigButton.clicked.connect(self.on_edit_config_clicked)
        self.ui.editRosParamButton.clicked.connect(self.on_edit_rosparam_clicked)
        self.ui.closeCfgButton.clicked.connect(self.on_close_clicked)

        self.ui.echoTopicButton.clicked.connect(self.on_topic_echo_clicked)
        self.ui.hzTopicButton.clicked.connect(self.on_topic_hz_clicked)
        self.ui.hzSshTopicButton.clicked.connect(self.on_topic_hz_ssh_clicked)
        self.ui.pubTopicButton.clicked.connect(self.on_topic_pub_clicked)
        self.ui.pubStopTopicButton.clicked.connect(self.on_topic_pub_stop_clicked)

        self.ui.callServiceButton.clicked.connect(self.on_service_call_clicked)
        self.ui.nodeFilterInput.textChanged.connect(self.on_node_filter_changed)
        self.ui.topicFilterInput.textChanged.connect(self.on_topic_filter_changed)
        self.ui.serviceFilterInput.textChanged.connect(self.on_service_filter_changed)
        self.ui.parameterFilterInput.textChanged.connect(self.on_parameter_filter_changed)
        self.ui.getParameterButton.clicked.connect(self.on_get_parameter_clicked)
        self.ui.addParameterButton.clicked.connect(self.on_add_parameter_clicked)
        self.ui.deleteParameterButton.clicked.connect(self.on_delete_parameter_clicked)
        self.ui.saveParameterButton.clicked.connect(self.on_save_parameter_clicked)
        self.ui.transferParameterButton.clicked.connect(self.on_transfer_parameter_clicked)

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

        self.ui.ioButton.setEnabled(True)
        self.ui.tabWidget.currentChanged.connect(self.on_tab_current_changed)
        self._shortcut_screen_show_all = QShortcut(QKeySequence(self.tr("Shift+S", "Show all available screens")), self)
        self._shortcut_screen_show_all.activated.connect(self.on_show_all_screens)
        self._shortcut_screen_kill = QShortcut(QKeySequence(self.tr("Shift+Backspace", "Kill Screen")), self)
        self._shortcut_screen_kill.activated.connect(self.on_kill_screens)

        self.loaded_config.connect(self._apply_launch_config)
        nm.nmd().launch.mtimes.connect(self._apply_mtimes)
        nm.nmd().launch.changed_binaries.connect(self._apply_changed_binaries)
        nm.nmd().launch.launch_nodes.connect(self.on_launch_description_retrieved)
        nm.nmd().version.version_signal.connect(self.on_nmd_version_retrieved)
        nm.nmd().screen.log_dir_size_signal.connect(self.on_log_dir_retrieved)

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
        self._shortcut_expand_all.activated.connect(self.ui.nodeTreeView.expandAll)
        self._shortcut_run = QShortcut(QKeySequence(self.tr("Alt+R", "run selected nodes")), self)
        self._shortcut_run.activated.connect(self.on_start_clicked)
        self._shortcut_stop = QShortcut(QKeySequence(self.tr("Alt+S", "stop selected nodes")), self)
        self._shortcut_stop.activated.connect(self.on_stop_clicked)

        self.message_frame = MessageFrame()
        self.ui.questionFrameLayout.addWidget(self.message_frame.ui)
        self.message_frame.accept_signal.connect(self._on_question_ok)
        self.message_frame.cancel_signal.connect(self._on_question_cancel)

        self.info_frame = MessageFrame(info=True)
        self.ui.infoFrameLayout.addWidget(self.info_frame.ui)
        self.info_frame.accept_signal.connect(self._on_info_ok)

        nm.nmd().file.changed_file.connect(self.on_changed_file)
        nm.nmd().screen.multiple_screens.connect(self.on_multiple_screens)
        self._sysmon_timer = None
        self._sysmon_enabled = False
        self._sysmon_timer_idle = QTimer()
        self._sysmon_timer_idle.timeout.connect(self._sysmon_update_callback)
        self._sysmon_timer_idle.start(nm.settings().sysmon_default_interval * 1000)

#        self._shortcut_copy = QShortcut(QKeySequence(self.tr("Ctrl+C", "copy selected values to clipboard")), self)
#        self._shortcut_copy.activated.connect(self.on_copy_c_pressed)
        self._shortcut_copy = QShortcut(QKeySequence(self.tr("Ctrl+X", "copy selected alternative values to clipboard")), self)
        self._shortcut_copy.activated.connect(self.on_copy_x_pressed)
        self.ui.controlNodesFrame.resizeEvent = self.resizeEventButtons

#    print "================ create", self.objectName()
#
#     def __del__(self):
#        print "    Destroy mester view proxy", self.objectName(), " ..."
#        print "    ", self.objectName(), "destroyed"

    def closeEvent(self, event):
        print('  Shutdown master %s ...' % self.masteruri)
        if self._sysmon_timer is not None:
            self._sysmon_timer.stop()
        self._sysmon_timer_idle.stop()
        nm.nmd().file.changed_file.disconnect(self.on_changed_file)
        nm.nmd().screen.multiple_screens.disconnect(self.on_multiple_screens)
        self.launch_server_handler.stop()
        self._progress_queue_prio.stop()
        self._progress_queue.stop()
        if self._on_stop_kill_roscore:
            self.killall_roscore()
        QWidget.closeEvent(self, event)
        print('  Master %s is down!' % self.masteruri)

    def stop_echo_dialogs(self):
        # stop launched echo dialogs
        if self.__echo_topics_dialogs:
            self.stop_nodes_by_name(self.__echo_topics_dialogs, only_local=False)
            self.__echo_topics_dialogs.clear()

    def resizeEventButtons(self, event):
        ch_height = 0
        increment = 4
        min_spacer_size = 8
        const_size = 10 * self.ui.verticalLayout_4.spacing() + min_spacer_size + self.ui.line2_2.size().height() + self.ui.line1_2.size().height()
        button_size = 9 * (self.ui.startButton.size().height() - self.__current_icon_height)
        while (ch_height + increment) * 10 + const_size + button_size <= self.ui.controlNodesFrame.size().height() and ch_height < 32:
            ch_height += increment
        if ch_height < 8:
            ch_height = 8
        if ch_height != self.__current_icon_height:
            self.__current_icon_height = ch_height
            new_size = QSize(self.__current_icon_height, self.__current_icon_height)
            self.ui.startButton.setIconSize(new_size)
            self.ui.stopButton.setIconSize(new_size)
            self.ui.ioButton.setIconSize(new_size)
            self.ui.logButton.setIconSize(new_size)
            self.ui.logDeleteButton.setIconSize(new_size)
            self.ui.dynamicConfigButton.setIconSize(new_size)
            self.ui.editConfigButton.setIconSize(new_size)
            self.ui.editRosParamButton.setIconSize(new_size)
            self.ui.closeCfgButton.setIconSize(new_size)

            self.ui.echoTopicButton.setIconSize(new_size)
            self.ui.hzTopicButton.setIconSize(new_size)
            self.ui.hzSshTopicButton.setIconSize(new_size)
            self.ui.pubTopicButton.setIconSize(new_size)
            self.ui.pubStopTopicButton.setIconSize(new_size)

            self.ui.callServiceButton.setIconSize(new_size)
            self.ui.getParameterButton.setIconSize(new_size)
            self.ui.addParameterButton.setIconSize(new_size)
            self.ui.deleteParameterButton.setIconSize(new_size)
            self.ui.saveParameterButton.setIconSize(new_size)
            self.ui.transferParameterButton.setIconSize(new_size)
        QFrame.resizeEvent(self, event)

    @property
    def current_user(self):
        return self.__current_user

    @current_user.setter
    def current_user(self, user):
        self.__current_user = user
        nm.settings().set_host_user(self.mastername, user)

    @property
    def daemon_user(self):
        return self.__daemon_user

    @daemon_user.setter
    def daemon_user(self, user):
        self.__daemon_user = user
        if user != self.current_user:
            self.message_frame.show_question(MessageFrame.TYPE_NMD_RESTART, "node_manager_daemon is running with different user: '%s'.\nMany features may not function properly!\n\nRestart with user '%s'?\n\nNote: you should first close all open editors related to this host!" % (user, self.current_user), MessageData(self.masteruri))
        else:
            self.message_frame.hide_question([MessageFrame.TYPE_NMD_RESTART])

    @property
    def is_local(self):
        return nm.is_local(get_hostname(self.masteruri), wait=False)

    @property
    def online(self):
        '''
        The online meens that master is discovered and master_info was received.
        '''
        return self.__online

    @online.setter
    def online(self, state):
        self.__online = state
        self._start_queue(self._progress_queue)
        self._start_queue(self._progress_queue_prio)

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

        :param master_info: the mater information object
        :type master_info: :class:`fkie_master_discovery.msg.MasterInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>
        '''
        try:
            update_result = (set(), set(), set(), set(), set(), set(), set(), set(), set())
            my_masterinfo = nmdurl.equal_uri(master_info.masteruri, self.masteruri)
            if self.__master_info is None:
                if my_masterinfo:
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
            if my_masterinfo:
                nmd_node = master_info.getNode('/node_manager_daemon')
                if nmd_node is None:  # do not test for PID. It can be None if daemon is busy on load big launch files
                    self._has_nmd = False
                    if time.time() - self.__last_question_start_nmd > 10.:
                        self.__last_question_start_nmd = time.time()
                        if not self.is_local:
                            self.message_frame.show_question(MessageFrame.TYPE_NMD, "node_manager_daemon not found for '%s'.\nShould it be started?" % self.masteruri, MessageData(self.masteruri))
                        else:
                            self._on_question_ok(MessageFrame.TYPE_NMD, MessageData(self.masteruri))
                else:
                    self.message_frame.hide_question([MessageFrame.TYPE_NMD])
                    if not self._timer_nmd_request.isActive():
                        timeout = 2000 if not self._has_nmd else 200
                        self._timer_nmd_request.start(timeout)
                    self._has_nmd = True
                self.perform_nmd_requests()
            try:
                if my_masterinfo:
                    self.update_system_parameter()
                    self.online = True
                # request the info of new remote nodes
                hosts2update = set([get_hostname(self.__master_info.getNode(nodename).uri) for nodename in update_result[0]])
                hosts2update.update([get_hostname(self.__master_info.getService(nodename).uri) for nodename in update_result[6]])
                for host in hosts2update:
                    if host != get_hostname(self.masteruri):
                        self.updateHostRequest.emit(host)
            except Exception:
                pass
#      cputimes = os.times()
#      cputime_init = cputimes[0] + cputimes[1]
            # update nodes in the model
            if update_result[0] or update_result[1] or update_result[2] or self.__force_update:
                self._update_running_nodes_in_model(self.__master_info)
                self.on_node_selection_changed(None, None)
                # update diagnostics
                new_diagnostic_dict = {}
                now = time.time()
                for sec, ds in self._stored_diagnostic_messages.items():
                    added = self.append_diagnostic(ds, False)
                    if not added and now - sec < 10:
                        new_diagnostic_dict[sec] = ds
                self._stored_diagnostic_messages = new_diagnostic_dict
            # Updates the topic view based on the current master information.
            if update_result[3] or update_result[4] or update_result[5] or self.__force_update:
                self.topic_model.update_model_data(self.__master_info.topics, update_result[3], update_result[4], update_result[5])
                self.on_topic_selection_changed(None, None)
            # Updates the service view based on the current master information.
            if update_result[6] or update_result[7] or update_result[8] or self.__force_update:
                self.service_model.update_model_data(self.__master_info.services, update_result[6], update_result[7], update_result[8])
                self.on_service_selection_changed(None, None)
                # update the default configuration
                # self.updateDefaultConfigs(self.__master_info)
            self.__force_update = False
#      cputimes = os.times()
#      cputime = cputimes[0] + cputimes[1] - cputime_init
#      print "  update on ", self.__master_info.mastername if not self.__master_info is None else self.__master_state.name, cputime
        except Exception:
            print(traceback.format_exc(3))

    def perform_nmd_requests(self):
        nmd_uri = nmdurl.nmduri(self.masteruri)
        if self._has_nmd:
            # only try to get updates from daemon if it is running
            nm.nmd().launch.get_nodes_threaded(nmd_uri, self.masteruri)
            # self.set_diagnostic_ok('/node_manager_daemon')
            nm.nmd().version.get_version_threaded(nmd_uri)
            nm.nmd().screen.log_dir_size_threaded(nmd_uri)
            nm.nmd().monitor.get_user_threaded(nmd_uri)
            self.perform_diagnostic_requests(force=True)

    def is_valid_user_master_daemon(self):
        if self.__daemon_user:
            return self.__daemon_user == self.current_user
        return True

    def _start_queue(self, queue):
        if self.online and self.master_info is not None and isinstance(queue, ProgressQueue):
            queue.start()

    @property
    def use_sim_time(self):
        return self.__use_sim_time

    def in_process(self):
        return self._progress_queue.count() > 0 or self._progress_queue_prio.count() > 0

    def force_next_update(self):
        self.__force_update = True

    def update_system_parameter(self):
        self.parameterHandler_sim.requestParameterValues(self.masteruri, ["/run_id", "/use_sim_time", "/robot_icon", "/roslaunch/uris"])

    def set_duplicate_nodes(self, running_nodes):
        '''
        Marks all nodes, which are not running and in a given list as a duplicates nodes.

        :param list(str) running_nodes: The list with names of running nodes
        '''
        # store the running_nodes to update to duplicates after load a launch file
        self.__running_nodes = running_nodes
        self.node_tree_model.set_duplicate_nodes(running_nodes, (self.master_info is not None and self.master_info.getNodeEndsWith('master_sync')))

    def get_nodes_runningIfSync(self):
        '''
        Returns the list with all running nodes, which are registered by this ROS
        master. Also the nodes, which are physically running on remote hosts.

        :return: The list with names of running nodes
        :rtype: list(str)
        '''
        if self.master_info is not None and self.master_info.getNodeEndsWith('master_sync'):
            return self.master_info.node_names
        return []

    def get_nodes_runningIfLocal(self, remove_system_nodes=False):
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
                    if remove_system_nodes or not self._is_in_ignore_list(node.name):
                        result[node.name] = self.master_info.masteruri
        return result

    def _update_running_nodes_in_model(self, master_info):
        '''
        Creates the dictionary with ExtendedNodeInfo objects and updates the nodes view.

        :param master_info: the mater information object
        :type master_info: :class:`fkie_master_discovery.msg.MasterInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#module-fkie_master_discovery.master_info>
        '''
        if master_info is not None:
            updated_nodes = self.node_tree_model.update_model_data(master_info.nodes, master_info.masteruri)
            if updated_nodes:
                for node in updated_nodes:
                    self.main_window.screen_dock.update_node(node)
            self.updateButtons()

    def getNode(self, node_name):
        '''
        :param str node_name: The name of the node.
        :return: The list the nodes with given name.
        :rtype: list(str)
        '''
        return self.node_tree_model.get_tree_node("%s" % node_name, self.masteruri)

    def updateButtons(self, selected_nodes=None):
        '''
        Updates the enable state of the buttons depending of the selection and
        running state of the selected node.
        '''
        selectedNodes = selected_nodes
        if selectedNodes is None:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
        has_running = False
        has_stopped = False
        for node in selectedNodes:
            if node.uri is not None:
                has_running = True
            else:
                has_stopped = True
        self.ui.startButton.setEnabled(True)
        self.ui.stopButton.setEnabled(True)
#    self.ui.ioButton.setEnabled(has_running or has_stopped)
        self.ui.logButton.setEnabled(True)
#    self.ui.logButton.setEnabled(has_running or has_stopped)
        self.ui.logDeleteButton.setEnabled(has_running or has_stopped)
        # test for available dynamic reconfigure services
        if self.master_info is not None:
            dyn_cfg_available = False
            for n in selectedNodes:
                for srv_name, srv in self.master_info.services.items():
                    if (srv_name.endswith('/set_parameters')) and n.name in srv.serviceProvider:
                        dyn_cfg_available = True
                        break
            self.ui.dynamicConfigButton.setEnabled(dyn_cfg_available)
        # the configuration is only available, if only one node is selected
        self.ui.editConfigButton.setEnabled(len(selectedNodes) == 1 and selectedNodes[0].has_configs())
        self.ui.editRosParamButton.setEnabled(len(selectedNodes) == 1)
        # enable the close button only for local configurations
        self.ui.closeCfgButton.setEnabled(True)
#    self.ui.closeCfgButton.setEnabled(len([path for path, _ in self.__configs.items() if (isinstance(path, tuple) and path[2] == self.masteruri) or not isinstance(path, tuple)]) > 0) #_:=cfg

    @property
    def launchfiles(self):
        '''
        Returns the copy of the dictionary with loaded launch files on this host

        :rtype: dict(str(file) : LaunchConfig)
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
        :param launchfile: the launch file path
        :type launchfile: str or tuple(launchfile, dictionary with args)
        '''
        lfile = launchfile
        args = {}
        if isinstance(launchfile, tuple):
            lfile, args = launchfile
        self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                            'Loading %s' % os.path.basename(lfile),
                                            self._load_launchfile,
                                            {'launchfile': lfile,
                                             'args_forced': args
                                            })
        self._start_queue(self._progress_queue_prio)

    def _load_launchfile(self, launchfile, args_forced={}, pqid=None):
        '''
        This method will be called in another thread. The configuration parameter
        of the launch file will be requested using `LaunchArgsSelectionRequest` and
        `InteractionNeededError`. After the file is successful loaded a
        `loaded_config` signal will be emitted.
        '''
        if args_forced:
            rospy.loginfo("LOAD launch: %s with args: %s" % (launchfile, args_forced))
        else:
            rospy.loginfo("LOAD launch: %s" % launchfile)
        # load launch configuration
        try:
            args = {}
            changed_nodes = []
            if launchfile in self.__configs:
                _launch_file, changed_nodes = nm.nmd().launch.reload_launch(launchfile, masteruri=self.masteruri)
            else:
                # nm.nmd().launch.load_launch(launchfile, argv_forced)  CREATE DICT
                # on_host should be an nmdurl
                _launch_file, args = nm.nmd().launch.load_launch(launchfile, masteruri=self.masteruri, host=self.masteruri, args=args_forced)
            # do not load if the loadings process was canceled
            if self._progress_queue_prio.has_id(pqid):
                cfg = LaunchConfig(launchfile, args=args)
                self._loaded_args[launchfile] = args
                self.loaded_config.emit(cfg, changed_nodes)
            nm.nmd().launch.get_nodes_threaded(launchfile)
        except nm.LaunchArgsSelectionRequest as lasr:
            raise nm.InteractionNeededError(lasr, self._load_launchfile, {'launchfile': launchfile, 'args_forced': args_forced})
        except exceptions.GrpcTimeout as tout:
            raise DetailedError("Timeout", "Timeout while load %s" % tout.remote, "Daemon not responded within %.2f seconds while"
                                "load launch file. You can try to increase the timeout for GRPC requests in node manager settings." % nm.settings().timeout_grpc)
        except Exception as e:
            print(traceback.format_exc())
            err_text = '%s loading failed!' % os.path.basename(launchfile)
            rospy.logwarn("Loading launch file: %s", utf8(e))
            raise DetailedError("Loading launch file", err_text, utf8(e))

    def _apply_launch_config(self, launchcfg, changed_nodes):
        filename = launchcfg.launchfile
        # store changed nodes for restart
        if changed_nodes:
            self._cfg_changed_nodes[filename] = changed_nodes
        if filename in self.__configs:
            # store expanded items
            self.__expanded_items[filename] = self._get_expanded_groups()

    def _apply_mtimes(self, launchfile, mtime, includes):
        if launchfile in self.__configs:
            cfg = self.__configs[launchfile]
            cfg.mtime = mtime
            cfg.includes = includes

    def _apply_changed_binaries(self, launchfile, nodes):
        muri = nmdurl.masteruri(launchfile)
        if nmdurl.equal_uri(muri, self.masteruri):
            for nodename, mtime in nodes.items():
                tnodes = self.node_tree_model.get_tree_node(nodename, self.masteruri)
                doask = False
                try:
                    if self._changed_binaries[nodename] < mtime:
                        doask = True
                except KeyError:
                    doask = True
                if doask:
                    for tnode in tnodes:
                        # ask for each node separately
                        self._changed_binaries[nodename] = mtime
                        self.question_restart_changed_binary(tnode)

    def perform_master_checks(self):
        grpc_url = nmdurl.nmduri(self.masteruri)
        lfiles = {}
        for path, cfg in self.__configs.items():
            if cfg.mtime > 0:
                lfiles[path] = cfg.mtime
                lfiles.update(cfg.includes)
        if self._has_nmd:
            # do not connect to the node manager daemon until it is not in the nodes list (not started)
            if lfiles:
                nm.nmd().file.check_for_changed_files_threaded(lfiles)
                nm.nmd().screen.multiple_screens_threaded(grpc_url)
            nodes = self.get_nodes_runningIfLocal(True)
            if nodes:
                nm.nmd().launch.get_changed_binaries_threaded(grpc_url, list(nodes.keys()))

    def perform_diagnostic_requests(self, force=False):
        now = time.time()
        if self._has_nmd and (self._has_diagnostics or force) and now - self._ts_last_diagnostic_request >= 1.0:
            nmd_uri = nmdurl.nmduri(self.masteruri)
            nm.nmd().monitor.get_system_diagnostics_threaded(nmd_uri)
            if not self.has_master_sync:
                nm.nmd().monitor.get_diagnostics_threaded(nmd_uri)
            elif nmdurl.equal_uri(self.masteruri, self.main_window.getMasteruri()):
                nm.nmd().monitor.get_diagnostics_threaded(nmd_uri)
            self._ts_last_diagnostic_request = now

    def get_files_for_change_check(self):
        result = {}
        for path, cfg in self.__configs.items():
            if cfg.mtime > 0:
                result[path] = cfg.mtime
                result.update(cfg.includes)
        return result

    def on_changed_file(self, grpc_path, mtime):
        for _path, cfg in self.__configs.items():
            if cfg.launchfile == grpc_path:
                # test launch file itself
                if cfg.mtime != mtime:
                    # it does not matter how the user response, we update the modification time to avoid a lot of questions
                    cfg.mtime = mtime
                    self.question_reload_changed_file(cfg.launchfile, cfg.launchfile)
                # continue, perhaps the file is an include in other launch files
            else:
                # test included files
                for incf, incmt in cfg.includes.items():
                    if incf == grpc_path:
                        if incmt != mtime:
                            cfg.includes[incf] = mtime
                            self.question_reload_changed_file(incf, cfg.launchfile)
                        break

    def reload_global_parameter_at_next_start(self, launchfile):
        try:
            self.__configs[launchfile].global_param_done = False
            self.on_node_selection_changed(None, None, True)
        except Exception:
            pass

    def question_restart_changed_binary(self, changed):
        self.message_frame.show_question(MessageFrame.TYPE_BINARY, 'Binaries changed, restart nodes?', MessageData(changed.name, [changed]))

    def question_reload_changed_file(self, changed, affected):
        _filename, file_extension = os.path.splitext(changed)
        if file_extension in nm.settings().launch_view_file_ext or changed.find('.launch.') > 0:
            changed_res = "%s[%s]" % (os.path.basename(changed), utf8(package_name(os.path.dirname(changed))[0]))
            self.message_frame.show_question(MessageFrame.TYPE_LAUNCH_FILE, 'Reload <b>%s</b>?<br>Changed files:' % os.path.basename(affected), MessageData(affected, [changed_res]))

    def question_transfer_changed_file(self, changed, affected):
        self.message_frame.show_question(MessageFrame.TYPE_TRANSFER,
                                         "Configuration file '%s' referenced by parameter in <b>%s</b> is changed.<br>Copy to remote host?"
                                         "<br>Don\'t forget to restart the corresponding nodes!" % (changed, os.path.basename(affected)), MessageData(changed))

    def _get_nodelets(self, nodename, configname=''):
        if configname and configname in self._nodelets:
            if nodename in self._nodelets[configname]:
                return self._nodelets[configname][nodename]
        else:
            for configname, mngrs in self._nodelets.items():
                if nodename in mngrs:
                    return mngrs[nodename]
        return []

    def _get_nodelet_manager(self, nodename, configname=''):
        if configname and configname in self._nodelets:
            for mngr, nodelets in self._nodelets[configname].items():
                if nodename in nodelets:
                    return mngr
        else:
            for configname, mngrs in self._nodelets.items():
                for mngr, nodelets in mngrs.items():
                    if nodename in nodelets:
                        return mngr
        return None

    def _get_expanded_groups(self):
        '''
        Returns a list of group names, which are expanded.
        '''
        result = []
        try:
            for r in range(self.ui.nodeTreeView.model().rowCount()):
                index_host = self.ui.nodeTreeView.model().index(r, 0)
                if index_host.isValid() and self.ui.nodeTreeView.isExpanded(index_host):
                    if self.ui.nodeTreeView.model().hasChildren(index_host):
                        for c in range(self.ui.nodeTreeView.model().rowCount(index_host)):
                            index_cap = self.ui.nodeTreeView.model().index(c, 0, index_host)
                            if index_cap.isValid() and self.ui.nodeTreeView.isExpanded(index_cap):
                                model_index = self.node_proxy_model.mapToSource(index_cap)
                                item = self.node_tree_model.itemFromIndex(model_index)
                                if isinstance(item, (GroupItem, HostItem)):
                                    result.append(item.name)
        except Exception:
            print(traceback.format_exc(3))
        return result

    def _expand_groups(self, groups=None):
        '''
        Expands all groups, which are in the given list. If no list is given,
        expands all groups of expanded hosts.
        '''
        try:
            for r in range(self.ui.nodeTreeView.model().rowCount()):
                index_host = self.ui.nodeTreeView.model().index(r, 0)
                if index_host.isValid() and self.ui.nodeTreeView.isExpanded(index_host):
                    if self.ui.nodeTreeView.model().hasChildren(index_host):
                        for c in range(self.ui.nodeTreeView.model().rowCount(index_host)):
                            index_cap = self.ui.nodeTreeView.model().index(c, 0, index_host)
                            if index_cap.isValid():
                                model_index = self.node_proxy_model.mapToSource(index_cap)
                                item = self.node_tree_model.itemFromIndex(model_index)
                                if isinstance(item, (GroupItem, HostItem)):
                                    if groups is None or item.name in groups:
                                        self.ui.nodeTreeView.setExpanded(index_cap, True)
        except Exception:
            print(traceback.format_exc(3))

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
            except Exception:
                pass
        self.__current_robot_icon = self.__current_parameter_robot_icon
        self.robot_icon_updated.emit(self.masteruri, utf8(self.__current_robot_icon))
        return self.__current_robot_icon

    def appendConfigToModel(self, launchfile, rosconfig):
        '''
        Update the node view

        :param str launchfile: the launch file path
        :param rosconfig: the configuration
        :type rosconfig: :class:`fkie_node_manager.launch_config.LaunchConfig`
        '''
        hosts = dict()  # dict(addr : dict(node : [config]) )
        addr = get_hostname(self.masteruri)
        masteruri = self.masteruri
        for n in rosconfig.nodes:
            if n.machine_name and not n.machine_name == 'localhost':
                if n.machine_name not in rosconfig.machines:
                    raise Exception(''.join(["ERROR: unknown machine [", n.machine_name, "]"]))
                addr = rosconfig.machines[n.machine_name].address
                masteruri = nm.nameres().masteruri(n.machine_name)
                if masteruri is None:
                    masteruri = nm.nameres().masteruribyaddr(n.machine_name)
            node = roslib.names.ns_join(n.namespace, n.name)
            if (masteruri, addr) not in hosts:
                hosts[(masteruri, addr)] = dict()
            hosts[(masteruri, addr)][node] = launchfile
        # add the configurations for each host separately
        for ((masteruri, addr), nodes) in hosts.items():
            self.node_tree_model.append_config(masteruri, addr, nodes)
        self.updateButtons()

    def remove_cfg_from_model(self, launchfile):
        '''
        Update the node view after removed configuration.

        :param str launchfile: the grpc path of the launch file
        '''
        self.remove_config_signal.emit(launchfile)
        self.node_tree_model.remove_config(launchfile)
        self.updateButtons()

    def on_launch_description_retrieved(self, url, launch_descriptions):
        '''
        Handles the new list with nodes from default configuration service.

        :param str url: the URI of the node manager daemon
        :param launch_descriptions: a list with configuration description.
        :type launch_descriptions: list(:class:`fkie_node_manager_daemon.launch_description.LaunchDescription`)
        '''
        if self._first_launch:
            self._first_launch = False
            if self.default_load_launch:
                lfile = nmdurl.join(nmdurl.nmduri(self.masteruri), self.default_load_launch)
                if os.path.isdir(self.default_load_launch):
                    self.main_window.launch_dock.launchlist_model.set_path(lfile)
                elif os.path.isfile(self.default_load_launch):
                    self.main_window.launch_dock.launchlist_model.set_path(os.path.dirname(self.default_load_launch))
                    self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                        'Loading %s' % os.path.basename(lfile),
                                                        self._load_launchfile,
                                                        {'launchfile': lfile,
                                                         'args_forced': {}
                                                        })
                    self._start_queue(self._progress_queue)
        if self.restarting_daemon:
            self.restarting_daemon = False
            for lf, lc in self.reloading_files.items():
                    self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                        'Loading %s' % os.path.basename(lf),
                                                        self._load_launchfile,
                                                        {'launchfile': lf,
                                                         'args_forced': lc.args
                                                        })
            self._start_queue(self._progress_queue)

        masteruri = self.masteruri
        host = get_hostname(masteruri)
        host_addr = host
        if host_addr is None:
            host_addr = host
        new_configs = []
        for ld in launch_descriptions:
            # TODO: check masteruri and host
            if ld.masteruri != masteruri:
                # rospy.logdebug("skip apply config %s from %s to %s with configs %s ", ld.path, ld.masteruri, masteruri, self.__configs)
                continue
            # add the new config
            if ld.path not in self.__configs:
                args = {}
                if ld.path in self._loaded_args:
                    args = self._loaded_args[ld.path]
                self.__configs[ld.path] = LaunchConfig(ld.path, args=args)
                nm.nmd().launch.get_mtimes_threaded(ld.path)
            new_configs.append(ld.path)
            self.__configs[ld.path].nodes = ld.nodes
            alredy_added_nodes = set()
            # update node configuration
            node_cfgs = dict()
            for n in ld.nodes:
                # if n not in alredy_added_nodes:
                node_cfgs[n] = ld.path
            self.node_tree_model.append_config(masteruri, host_addr, node_cfgs)
            # update capabilities
            for rd in ld.robot_descriptions:
                # add capabilities
                caps = dict()
                rd_node_cfgs = dict()
                for c in rd.capabilities:
                    if c.namespace not in caps:
                        caps[c.namespace] = dict()
                    caps[c.namespace][utf8(c.name)] = {'type': c.type, 'images': [interpret_path(i) for i in c.images], 'description': interpret_path(utf8(c.description.replace("\\n ", "\n"))), 'nodes': list(c.nodes)}
                    for n in c.nodes:
                        rd_node_cfgs[n] = ld.path
                        alredy_added_nodes.add(n)
                robot_addr = host_addr
                valid_machine = False
                if rd.machine and rd.machine != host:
                    robot_addr = rd.machine
                    valid_machine = True
                # print('append', masteruri, robot_addr, rd_node_cfgs)
                if robot_addr != host_addr:
                    self.node_tree_model.append_config(masteruri, robot_addr, rd_node_cfgs)
                if valid_machine or not rd.robot_name or utf8(rd.robot_name) == self.mastername:
                    self.node_tree_model.add_capabilities(masteruri, robot_addr, ld.path, caps)
                    # set host description
                    tooltip = self.node_tree_model.update_host_description(masteruri, robot_addr, rd.robot_type, utf8(rd.robot_name), interpret_path(utf8(rd.robot_descr)))
                    self.capabilities_update_signal.emit(masteruri, robot_addr, ld.path, [rd])
                    self.host_description_updated.emit(masteruri, robot_addr, tooltip)
             # set the robot_icon
            if ld.path in self.__robot_icons:
                self.__robot_icons.remove(ld.path)
            self.__robot_icons.insert(0, ld.path)
            self.set_duplicate_nodes(self.__running_nodes)
            # expand items to restore old view
            if ld.path in self.__expanded_items:
                self._expand_groups(self.__expanded_items[ld.path])
                del self.__expanded_items[ld.path]
#             # update nodelets TODO: get it from nmd
#             nodelets = {}
#             for n in launchConfig.Roscfg.nodes:
#                 if n.package == 'nodelet' and n.type == 'nodelet':
#                     args = n.args.split(' ')
#                     if len(args) == 3 and args[0] == 'load':
#                         nodelet_mngr = roslib.names.ns_join(n.namespace, args[2])
#                         if nodelet_mngr not in nodelets:
#                             nodelets[nodelet_mngr] = []
#                         nodelets[nodelet_mngr].append(roslib.names.ns_join(n.namespace, n.name))
            for mngr, nlist in ld.nodelets.items():
                mngr_nodes = self.node_tree_model.get_tree_node(mngr, self.masteruri)
                for mn in mngr_nodes:
                    mn.nodelets = nlist
                for nlet in nlist:
                    nlet_nodes = self.node_tree_model.get_tree_node(nlet, self.masteruri)
                    for nn in nlet_nodes:
                        nn.nodelet_mngr = mngr
            self._nodelets[ld.path] = ld.nodelets
            with self._associations_lock:
                self._associations[ld.path] = ld.associations
            if ld.path in self._start_nodes_after_load_cfg:
                self.start_nodes_by_name(self._start_nodes_after_load_cfg[ld.path], ld.path, True)
                del self._start_nodes_after_load_cfg[ld.path]
        removed_configs = set(self.__configs.keys()) - set(new_configs)
        for cfg in removed_configs:
            if isinstance(cfg, tuple):
                rospy.logwarn("CFG: unsupported config type: %s" % str(cfg))
                continue
            if cfg.startswith(url):
                self.remove_cfg_from_model(cfg)
                del self.__configs[cfg]
            else:
                pass
        self.updateButtons()
        for cfg in new_configs:
            if cfg in self._cfg_changed_nodes:
                changed_nodes = self._cfg_changed_nodes[cfg]
                del self._cfg_changed_nodes[cfg]
                node_count = ''
                if len(changed_nodes) > 1:
                    node_count = 's [%d]' % len(changed_nodes)
                nodes_text = '<br>'
                for chn in changed_nodes:
                    nodes_text += "%s   " % HTMLDelegate.toHTML(chn)
                self.message_frame.show_question(MessageFrame.TYPE_NODE_CFG, 'Configuration changed for node%s:%s<br>restart?' % (node_count, nodes_text), MessageData((changed_nodes, cfg)))

    def on_nmd_version_retrieved(self, nmd_url, version, date):
        if not nmdurl.equal_uri(nmdurl.masteruri(nmd_url), self.masteruri):
            return
        self._diag_nmd_version = version
        self._check_diag_state_nmd()

    def on_log_dir_retrieved(self, nmd_url, log_dir_size):
        if not nmdurl.equal_uri(nmdurl.masteruri(nmd_url), self.masteruri):
            return
        self._diag_log_dir_size = log_dir_size
        self._check_diag_state_nmd()

    def _check_diag_state_nmd(self):
        state_ok = True
        if self._diag_nmd_version is not None:
            if self._diag_nmd_version != self._nmd_version:
                state_ok = False
                res = self.set_diagnostic_warn('/node_manager_daemon', "node_manager_daemon has on<br>%s different version<br>'%s', own:<br>'%s'.<br>Please update and restart!" % (self.masteruri, self._diag_nmd_version, self._nmd_version))
                if not res:
                    self.message_frame.show_question(MessageFrame.TYPE_NMD, "node_manager_daemon has on %s different version '%s', own '%s'.\nShould it be started?" % (self.masteruri, self._diag_nmd_version, self._nmd_version), MessageData(self.masteruri))
        if self._diag_log_dir_size is not None:
            if self._diag_log_dir_size > 1073741824:
                state_ok = False
                hostname = get_hostname(self.masteruri)
                clean_cmd = '<a href="rosclean://%s" title="calls `rosclean purge` at `%s`">rosclean purge</a>' % (self.masteruri.replace('http://', ''), hostname)
                res = self.set_diagnostic_warn('/node_manager_daemon', "disk usage in log directory @%s is %s. %s" % (get_hostname(self.masteruri), sizeof_fmt(self._diag_log_dir_size), clean_cmd))
        if state_ok:
            self.set_diagnostic_ok('/node_manager_daemon')

    def set_diagnostic_warn(self, node_name, msg):
        if DIAGNOSTICS_AVAILABLE:
            diagnostic_status = DiagnosticStatus()
            diagnostic_status.name = node_name
            diagnostic_status.level = DiagnosticStatus.WARN
            diagnostic_status.message = msg
            self.append_diagnostic(diagnostic_status)
            return True
        return False

    def set_diagnostic_ok(self, node_name):
        if DIAGNOSTICS_AVAILABLE:
            diagnostic_status = DiagnosticStatus()
            diagnostic_status.name = node_name
            diagnostic_status.level = DiagnosticStatus.OK
            diagnostic_status.message = ''
            self.append_diagnostic(diagnostic_status)
            return True
        return False

    def update_system_diagnostics(self, diagnostics):
        self.node_tree_model.update_system_diagnostics(self.masteruri, diagnostics)
        selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
        selectedNodes = self.hostsFromIndexes(selections)
        if len(selectedNodes) == 1:
            if selectedNodes[0].local:
                self.on_node_selection_changed(None, None)

    def append_diagnostic(self, diagnostic_status, isnew=True):
        result = False
        if (diagnostic_status.name == '/master_sync'):
            if get_hostname(self.masteruri) != diagnostic_status.hardware_id:
                return False
        if diagnostic_status.name not in ['/node_manager_daemon']:
            self._has_diagnostics = True
        nodes = self.getNode(diagnostic_status.name)
        for node in nodes:
            node.append_diagnostic_status(diagnostic_status)
            result = True
        if nodes:
            selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                if node.name == diagnostic_status.name:
                    self.on_node_selection_changed(None, None)
        elif isnew:
            # store to have messages received before node was detected
            self._stored_diagnostic_messages[time.time()] = diagnostic_status
        return result

    def sysmon_active_update(self):
        if self._sysmon_timer is None:
            self._sysmon_timer = QTimer()
            self._sysmon_timer.timeout.connect(self._sysmon_update_callback)
            self._sysmon_timer.start(1000)
            self.node_tree_model.sysmon_set_state(self.masteruri, True)
        else:
            self._sysmon_timer.stop()
            self._sysmon_timer = None
            self.node_tree_model.sysmon_set_state(self.masteruri, False)
            # update host description
            selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.hostsFromIndexes(selections)
            if len(selectedNodes) == 1:
                if selectedNodes[0].local:
                    self.on_node_selection_changed(None, None)

    def _sysmon_update_callback(self):
        if self._has_nmd and self.__online:
            nm.nmd().monitor.get_system_diagnostics_threaded(nmdurl.nmduri(self.masteruri))
            if not nm.is_local(self.mastername):
                nm.nmd().monitor.get_diagnostics_threaded(nmdurl.nmduri(self.masteruri))

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

        :param str serveruri: the URI of the roslaunch server
        :param str pid: the process id of the roslaunch server
        :param list(str) nodes: list with nodes handled by the roslaunch server
        '''
        self.__launch_servers[serveruri] = (pid, nodes)

    def on_launch_server_err(self, serveruri, msg):
        '''
        Handles the error messages from launch server hanlder.

        :param str serveruri: the URI of the launch server
        :param str msg: the error message
        '''
        try:
            del self.__launch_servers[serveruri]
        except Exception:
            pass

    def on_remove_all_launch_server(self):
        '''
        Kill all running launch server. The coresponding URIS are removed by master_monitor.
        '''
        for lsuri, (pid, nodes) in self.__launch_servers.items():
            try:
                if not self._is_master_launch_server(nodes):
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   'kill roslaunch %s (%s)' % (lsuri, utf8(pid)),
                                                   nm.starter().kill,
                                                   {'host': get_hostname(lsuri),
                                                    'pid': pid,
                                                    'auto_pw_request': False,
                                                    'user': self.current_user
                                                   })
                    self.launch_server_handler.updateLaunchServerInfo(lsuri, delayed_exec=3.0)
            except Exception as e:
                rospy.logwarn("Error while kill roslaunch %s: %s", utf8(lsuri), utf8(e))
                raise DetailedError("Kill error",
                                    ''.join(['Error while kill roslaunch ', lsuri]),
                                    utf8(e))
        self._start_queue(self._progress_queue)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Handling of the view activities                  %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def on_node_activated(self, index):
        '''
        Depending of the state of the node, it will be run or the screen output will
        be open.

        :param index: The index of the activated node
        :type index: :class:`QtCore.QModelIndex` <https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>
        '''
        self.__last_node_activation = time.time()
        selectedNodes = []
        if index.column() == 0:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes(), False)
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
        elif has_running:
            self.on_io_clicked(activated=True)

    def on_node_clicked(self, index):
        if time.time() - self.__last_node_activation > 1.:
            self.message_frame.hide_question([MessageFrame.TYPE_NODELET])
            self.info_frame.hide_question([MessageFrame.TYPE_NOSCREEN])
        if time.time() - self.__last_selection > 1.:
            self.on_node_selection_changed(None, None, True)

    def on_topic_activated(self, index):
        '''
        :param index: The index of the activated topic
        :type index: :class:`QtCore.QModelIndex` <https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>
        '''
        model_index = self.topic_proxyModel.mapToSource(index)
        item = self.topic_model.itemFromIndex(model_index)
        if isinstance(item, TopicItem):
            self.on_topic_echo_clicked([item.topic])

    def on_topic_clicked(self, index):
        if time.time() - self.__last_selection > 1.:
            self.on_topic_selection_changed(None, None, True)

    def on_service_activated(self, index):
        '''
        :param index: The index of the activated service
        :type index: :class:`QtCore.QModelIndex` <https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>
        '''
        model_index = self.service_proxyModel.mapToSource(index)
        item = self.service_model.itemFromIndex(model_index)
        if isinstance(item, ServiceItem):
            self.on_service_call_clicked([item.service])

    def on_service_clicked(self, index):
        if time.time() - self.__last_selection > 1.:
            self.on_service_selection_changed(None, None, True)

    def on_host_inserted(self, item):
        if item == (self.masteruri, get_hostname(self.masteruri)):
            index = self.node_tree_model.indexFromItem(item)
            model_index = self.node_proxy_model.mapFromSource(index)
            if model_index.isValid():
                self.ui.nodeTreeView.expand(model_index)
#        self.ui.nodeTreeView.expandAll()

    def on_node_collapsed(self, index):
        if not index.parent().isValid():
            self.ui.nodeTreeView.selectionModel().clear()

    def on_node_expanded(self, index):
        pass

    def _create_html_list(self, title, items, list_type=None, name=''):
        '''
        :param list_type: LAUNCH, TOPIC, NODE, SERVICE, LOG
        :type list_type: str
        '''
        result = ''
        if items:
            result = '<b><u>%s</u></b>' % (title)
            if len(items) > 1:
                result = '%s <span style="color:gray;">[%d]</span>' % (result, len(items))
            result = '%s<table style="display: inline-table">' % result
            items.sort()
            for i in items:
                item = i
                # reduce the displayed name
                item_name = i
                if list_type in ['LOG']:
                    item = i.name
                    item_name = i.name
                if name:
                    if item_name.startswith(name):
                        item_name = item_name.replace('%s%s' % (name, roslib.names.SEP), '~', 1)
                    ns = roslib.names.namespace(name)
                    if item_name.startswith(ns) and ns != roslib.names.SEP:
                        item_name = item_name.replace(ns, '', 1)
                if list_type in ['NODE']:
                    item = '<tr>'
                    item += '<td><a href="node://%s%s">%s</a></td>' % (self.mastername, i, item_name)
                    item += '</tr>'
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
                    except Exception:
                        pass
                    # add the count
                    if count is not None:
                        item = '<tr>'
                        item += '<td><span style="color:gray;">%d</span></td>' % (count)
                        item += '<td><a href="topicecho://%s%s"><span style="color:gray;"><i>echo</i></span></a></td>' % (self.mastername, i)
                        item += '<td><a href="topic://%s">%s</a></td>' % (i, item_name)
                        #sekkyumu_topic_echo_24 = nm.settings().icon_path('sekkyumu_topic_echo_24.png')
                        #item += '<td><a href="topicecho://%s%s" title="Show the content of the topic"><img src="%s" alt="echo"></a></td>' % (self.mastername, i, sekkyumu_topic_echo_24)
                        item += '</tr>'
                    else:
                        item = '<tr>'
                        item += '<td colspan="3" style="float:left"><span style="color:red;">!sync </span><a>%s</a></td>' % (item_name)
                        item += '</tr>'
                elif list_type == 'SERVICE':
                    try:
                        srv = self.__master_info.getService(i)
                        if srv is not None and name in srv.serviceProvider:
                            item = '<tr>'
                            item += '<td><a href="servicecall://%s%s"><span style="color:gray;"><i>call</i></span></a></td>' % (self.mastername, i)
                            item += '<td><a href="service://%s%s">%s</a></td>' % (self.mastername, i, item_name)
                            item += '</tr>'
                        else:
                            item = '<tr>'
                            item += '<td colspan="2" style="float:left"><span style="color:red;">!sync </span>%s</td>' % (item_name)
                            item += '</tr>'
                    except Exception:
                        item = '<tr>'
                        item += '<td colspan="2" style="float:left"><span style="color:red;">?sync </span>%s</td>' % (item_name)
                        item += '</tr>'
                elif list_type == 'LAUNCH':
                    if i in self.__configs and self.__configs[i].global_param_done:
                        item = '<tr>'
                        item_ref = '<a href="%s">%s</a>' % (i.replace('grpc://', 'open-edit://'), os.path.basename(item_name))
                        item += '<td>%s</td>' % (item_ref)
                        pkg, _path = nm.nmd().file.package_name(i)
                        item += '<td><i>%s</i></td>' % (os.path.dirname(item_name) if pkg is None else pkg)
                        item += '</tr>'
                elif list_type == 'LOG':
                    node_host = self.getHostFromNode(i)
                    if nm.is_local(node_host, wait=False):
                        roslogfile = screen.get_ros_logfile(node=i.name)
                        logfile = screen.get_logfile(node=i.name)
                    else:
                        roslogfile = '%s@%s' % (i.name, node_host)
                        logfile = '%s@%s' % (i.name, node_host)
                    item = '<tr>'
                    item += '<td colspan="2" style="float:left"><span style="color:grey;">roslog: </span></td>'
                    item_ref = '<a href="%s">%s</a>' % ('show-roslog://%s@%s' % (i.name, node_host), roslogfile)
                    item += '<td>%s</td>' % (item_ref)
                    item += '</tr>'
                    item += '<tr>'
                    item += '<td colspan="2" style="float:left"><span style="color:grey;">screen: </span></td>'
                    item_ref = '<a href="%s">%s</a>' % ('show-log://%s@%s' % (i.name, node_host), logfile)
                    item += '<td>%s</td>' % (item_ref)
                    item += '</tr>'
                result += item
            result += '</table>\n<br>'
        return result

    def on_tab_current_changed(self, index):
        tab_name = self.ui.tabWidget.currentWidget().objectName()
        if tab_name == 'tabTopics':
            # select the topics of the selected node in the "Topic" view
            selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                selected_topics = self.topic_model.index_from_names(node.published, node.subscribed)
                for s in selected_topics:
                    self.ui.topicsView.selectionModel().select(self.topic_proxyModel.mapFromSource(s), QItemSelectionModel.Select)
        elif tab_name == 'tabServices':
            # select the services of the selected node in the "Services" view
            selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
            selectedNodes = self.nodesFromIndexes(selections)
            if len(selectedNodes) == 1:
                node = selectedNodes[0]
                selected_services = self.service_model.index_from_names(node.services)
                for s in selected_services:
                    self.ui.servicesView.selectionModel().select(self.service_proxyModel.mapFromSource(s), QItemSelectionModel.Select)

    def _is_current_tab_name(self, tab_name):
        return (self.ui.tabWidget.currentWidget().objectName() == tab_name)

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
            selections = self.ui.nodeTreeView.selectionModel().selectedIndexes()
            selectedHosts = self.hostsFromIndexes(selections)
            selectedNodes = self.nodesFromIndexes(selections)
            selectedGroups = self.groupsFromIndexes(selections)
        self.ui.topicsView.selectionModel().clear()
        self.ui.servicesView.selectionModel().clear()
        name = ''
        text = ''
        # add control buttons for more then one selected node
        if len(selectedNodes) > 1 or len(selectedGroups) > 0:
            restartable_nodes = [sn for sn in selectedNodes if len(sn.cfgs) > 0 and not self._is_in_ignore_list(sn.name)]
            restartable_nodes_with_launchfiles = [sn for sn in selectedNodes if sn.has_launch_cfgs(sn.cfgs) > 0 and not self._is_in_ignore_list(sn.name)]
            killable_nodes = [sn for sn in selectedNodes if not self._is_in_ignore_list(sn.name)]
            unregisterble_nodes = [sn for sn in selectedNodes if sn.node_info.pid is None and sn.node_info.uri is not None and sn.node_info.isLocal and not self._is_in_ignore_list(sn.name)]
            # add description for multiple selected nodes
            if restartable_nodes or killable_nodes or unregisterble_nodes:
                text += '<b>Selected nodes:</b><br>'
            restart_icon_path = nm.settings().icon_path('sekkyumu_restart_24.png')
            restart_g_icon_path = nm.settings().icon_path('sekkyumu_restart_g_24.png')
            sekkyumu_kill_screen_24 = nm.settings().icon_path('sekkyumu_kill_screen_24.png')
            play_alt_icon_path = nm.settings().icon_path('sekkyumu_play_alt_24.png')
            if restartable_nodes:
                text += '<a href="restart-node://all_selected_nodes" title="Restart %s selected nodes Ctrl+Shift+R"><img src="%s" alt="restart">[%d]</a>' % (len(restartable_nodes), restart_icon_path, len(restartable_nodes))
                text += '&nbsp;<a href="restart-node-g://all_selected_nodes" title="Reload global parameter and restart %s selected nodes Ctrl+Shift+Alt+R"><img src="%s" alt="restart">[%d]</a>' % (len(restartable_nodes), restart_g_icon_path, len(restartable_nodes))
            if killable_nodes:
                text += '&nbsp;<a href="kill-screen://all_selected_nodes" title="Kill %s screens of selected nodes"><img src="%s" alt="killscreen">[%d]</a>' % (len(killable_nodes), sekkyumu_kill_screen_24, len(killable_nodes))
            if restartable_nodes_with_launchfiles:
                text += '&nbsp;<a href="start-node-adv://all_selected_nodes" title="Start %s nodes with additional options, e.g. loglevel"><img src="%s" alt="play alt">[%d]</a>' % (len(restartable_nodes_with_launchfiles), play_alt_icon_path, len(restartable_nodes_with_launchfiles))
            if unregisterble_nodes:
                text += '<br><a href="unregister-node://all_selected_nodes">unregister [%d]</a>' % len(unregisterble_nodes)
        # add host description, if only the one host is selected
        if len(selectedHosts) == 1:  # and len(selections) / 2 == 1:
            host = selectedHosts[0]
            name = '%s - Robot' % host.name
            text += host.generate_description()
            text += '<br>'
        else:
            # add group description, if only the one group is selected
            if len(selectedGroups) == 1 and len(selections) / 2 == 1:
                group = selectedGroups[0]
                name = '%s - Group' % group.name
                text += group.generate_description()
                text += '<br>'
        # add node description for one selected node
        if len(selectedHosts) != 1 and len(selectedNodes) == 1 and len(selectedGroups) == 0:
            node = selectedNodes[0]
            text = '<div>%s</div>' % self.get_node_description(node_name, node)
            # name = node.name
            name = 'Node - Info'
        if (self._is_current_tab_name('tabNodes') and self.__last_info_text != text) or force_emit:
            self.__last_info_text = text
            self.description_signal.emit(name, text, True if selected or deselected or force_emit else False)
        self.updateButtons(selectedNodes)

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
            launches = [c for c in node.cfgs if not isinstance(c, tuple)]
            crystal_clear_settings_24 = nm.settings().icon_path('crystal_clear_settings_24.png')
            if name == 'node_manager_daemon':
                text += '<a href="nmd-cfg://%s" title="Configure Daemon"><img src="%s" alt="configure"></a>' % (utf8(self.masteruri).replace('http://', ''), crystal_clear_settings_24)
            elif name == 'node_manager' and nm.is_local(self.mastername):
                text += '<a href="nm-cfg://%s" title="Configure Node Manager"><img src="%s" alt="configure"></a>' % (utf8(self.masteruri).replace('http://', ''), crystal_clear_settings_24)
            if launches:
                sekkyumu_restart_24 = nm.settings().icon_path('sekkyumu_restart_24.png')
                sekkyumu_restart_g_24 = nm.settings().icon_path('sekkyumu_restart_g_24.png')
                text += '<a href="restart-node://%s" title="Restart node Ctrl+Shift+R"><img src="%s" alt="restart"></a>' % (node.name, sekkyumu_restart_24)
                text += '&nbsp;<a href="restart-node-g://%s" title="Reload global parameter and restart node Ctrl+Shift+Alt+R"><img src="%s" alt="restart"></a>' % (node.name, sekkyumu_restart_g_24)
            sekkyumu_kill_screen_24 = nm.settings().icon_path('sekkyumu_kill_screen_24.png')
            text += '&nbsp; <a href="kill-screen://%s" title="Kill screen of the node"><img src="%s" alt="killscreen"></a>' % (node.name, sekkyumu_kill_screen_24)
            if launches:
                sekkyumu_play_alt_24 = nm.settings().icon_path('sekkyumu_play_alt_24.png')
                text += '&nbsp; <a href="start-node-adv://%s" title="Start node with additional options, e.g. loglevel"><img src="%s" alt="play alt"></a>' % (node.name, sekkyumu_play_alt_24)
            crystal_clear_copy_log_path_24 = nm.settings().icon_path('crystal_clear_copy_log_path_24.png')
            text += '&nbsp; <a href="copy-log-path://%s" title="copy log path to clipboard"><img src="%s" alt="copy_log_path"></a>' % (node.name, crystal_clear_copy_log_path_24)
            text += '<br><font size="+2"><b>%s</b></font>' % (name)
            text += '&nbsp;<font size="-1"><a href="copy://%s" style="color:gray;">copy</a></font>' % (node.name)
            text += '<br><span style="color:gray;">ns: </span><b>%s%s</b>' % (ns, sep)
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
                    text += '<dt><font color="#FF9900"><b>remote nodes will not be ping, so they are always marked running.<br>Do all nodes have the same ROS_MASTER_URI or node uri?</b></font>'
                else:
                    text += '<dt><font color="#CC0000"><b>the node does not respond: </b></font>'
                text += ' <a href="unregister-node://%s">unregister</a></dt>' % node.name
            added_diags = []
            if node.diagnostic_array:
                diag_status = node.diagnostic_array[-1]
                if node.diagnostic_array:
                    level_str = 'Unknown'
                    if diag_status.level in self.DIAGNOSTIC_LEVELS:
                        level_str = self.DIAGNOSTIC_LEVELS[diag_status.level]
                    diag_color = '#008000'  # green
                    if diag_status.level == 1:
                        diag_color = '#FF6600'
                    elif diag_status.level == 2:
                        diag_color = '#CC0000'
                    elif diag_status.level == 3:
                        diag_color = '#FFCC00'
                    elif diag_status.level > 3:
                        diag_color = '#0000CC'
                    if diag_status.message:
                        diag_msg = '<dt><font color="%s"><b>%s: %s</b></font></dt>' % (diag_color, level_str, diag_status.message)
                    else:
                        diag_msg = '<dt><font color="%s"><b>%s</b></font></dt>' % (diag_color, level_str)
                    if diag_msg not in added_diags:
                        text += diag_msg
                        added_diags.append(diag_msg)
#                if len(node.diagnostic_array) > 1:
                text += '<dt><a href="show-all-diagnostics://%s">show all diagnostic msgs (%s)</a></dt>' % (node.name, len(node.diagnostic_array))
#        if len(node.diagnostic_array) > 1:
#          text += '<dt><font color="#FF6600"><a href="view_diagnostics://%s">view recent %d items</a></font></dt>'%(node.name, len(node.diagnostic_array))
            if node.nodelet_mngr:
                text += '<dt><b>Nodelet manager</b>: %s</dt>' % self._create_html_list('', [node.nodelet_mngr], 'NODE')
            if node.nodelets:
                text += '<dt>Manager for <b>%d</b> nodelets</dt>' % len(node.nodelets)
            text += '</dl>'
            if nm.settings().transpose_pub_sub_descr:
                text += self._create_html_list('<br>Subscribed Topics:', node.subscribed, 'TOPIC_SUB', node.name)
                text += self._create_html_list('<br>Published Topics:', node.published, 'TOPIC_PUB', node.name)
            else:
                text += self._create_html_list('<br>Published Topics:', node.published, 'TOPIC_PUB', node.name)
                text += self._create_html_list('<br>Subscribed Topics:', node.subscribed, 'TOPIC_SUB', node.name)
            text += self._create_html_list('<br>Services:', node.services, 'SERVICE', node.name)
            # set loaunch file paths
            text += self._create_html_list('<br>Launch Files:', launches, 'LAUNCH')
            # text += self._create_html_list('Default Configurations:', default_cfgs, 'NODE')
            text += self._create_html_list('<br>Logs:', [node], 'LOG')
#      text += '<dt><a href="copy-log-path://%s">copy log path to clipboard</a></dt>'%node.name
        return text

    def show_diagnostic_messages(self, node):
        found_nodes = []
        if self.master_info is not None:
            found_nodes = self.node_tree_model.get_node_items_by_name([node])
            if found_nodes:
                node_item = found_nodes[0]
                text = ''
                for diag in reversed(node_item.diagnostic_array):
                    msg = EchoDialog.strify_message(diag)
                    if isinstance(msg, tuple):
                        msg = msg[0]
                    msg = msg.replace('<', '&lt;').replace('>', '&gt;')
                    msg = '<pre style="background-color:#FFFCCC; color:#000000;font-family:Fixedsys,Courier; padding:10px;">---------- %s --------------------\n%s</pre>' % (diag.values[-1].value, msg)
                    text += msg
                if text:
                    self.description_signal.emit('%s diagnostic' % node, text, True)

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
            selectedTopics = [self.master_info.getTopic('%s' % topic_name)]
            if len(selectedTopics) == 0:
                return
        else:
            if not self._is_current_tab_name('tabTopics'):
                return
            selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
            topics_selected = (len(selectedTopics) > 0)
            self.ui.echoTopicButton.setEnabled(topics_selected)
            self.ui.hzTopicButton.setEnabled(topics_selected)
            self.ui.hzSshTopicButton.setEnabled(topics_selected)
            self.ui.pubStopTopicButton.setEnabled(topics_selected)
        if len(selectedTopics) == 1:
            try:
                topic = selectedTopics[0]
                text = self.get_topic_description(topic_name, topic)
                info_text = '<div>%s</div>' % text
                if (self._is_current_tab_name('tabTopics') and self.__last_info_text != info_text) or force_emit:
                    self.__last_info_text = info_text
                    self.description_signal.emit('Topic - Info', info_text, True if selected or deselected or force_emit else False)
            except Exception as _:
                pass

    def get_topic_description(self, topic_name, topic=None):
        text = ''
        if topic is None:
            selectedTopics = []
            if topic_name and self.master_info is not None:
                selectedTopics = [self.master_info.getTopic("%s" % topic_name)]
            else:
                selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
            if len(selectedTopics) == 1:
                topic = selectedTopics[0]
        if topic is not None:
            sekkyumu_topic_echo_24 = nm.settings().icon_path('sekkyumu_topic_echo_24.png')
            sekkyumu_topic_hz_24 = nm.settings().icon_path('sekkyumu_topic_hz_24.png')
            sekkyumu_topic_echo_ssh_hz_24 = nm.settings().icon_path('sekkyumu_topic_echo_ssh_hz_24.png')
            sekkyumu_topic_pub_24 = nm.settings().icon_path('sekkyumu_topic_pub_24.png')
            sekkyumu_topic_repub_24 = nm.settings().icon_path('sekkyumu_topic_repub_24.png')
            ns, sep, name = topic.name.rpartition(rospy.names.SEP)
            # text = '<font size="+1"><b><span style="color:gray;">%s%s</span><b>%s</b></font><br>' % (ns, sep, name)
            text += '&nbsp;<a href="topicecho://%s%s" title="Show the content of the topic"><img src="%s" alt="echo"></a>' % (self.mastername, topic.name, sekkyumu_topic_echo_24)
            text += '&nbsp;<a href="topichz://%s%s" title="Show only the receive rate of the topic.<br>All data is sent through the network"><img src="%s" alt="hz"></a>' % (self.mastername, topic.name, sekkyumu_topic_hz_24)
            text += '&nbsp;<a href="topichzssh://%s%s" title="Show only the receive rate of the topic.<br>Uses an SSH connection to execute `rostopic hz` on remote host."><img src="%s" alt="sshhz"></a>' % (self.mastername, topic.name, sekkyumu_topic_echo_ssh_hz_24)
            text += '&nbsp;<a href="topicpub://%s%s" title="Start a publisher for selected topic"><img src="%s" alt="pub"></a>' % (self.mastername, topic.name, sekkyumu_topic_pub_24)
            if topic.name in self.__republish_params:
                text += '&nbsp;<a href="topicrepub://%s%s" title="Start a publisher with last parameters"><img src="%s" alt="repub"></a>' % (self.mastername, topic.name, sekkyumu_topic_repub_24)
            topic_publisher = []
            topic_prefix = '/rostopic_pub%s_' % topic.name
            node_names = self.master_info.node_names
            for n in node_names:
                if n.startswith(topic_prefix):
                    topic_publisher.append(n)
            if topic_publisher:
                sekkyumu_topic_pub_stop_24 = nm.settings().icon_path('sekkyumu_topic_pub_stop_24.png')
                text += '&nbsp;<a href="topicstop://%s%s"><img src="%s" alt="stop"> [%d]</a>' % (self.mastername, topic.name, sekkyumu_topic_pub_stop_24, len(topic_publisher))
            text += '<br><font size="+2"><b>%s</b></font>' % (name)
            text += '&nbsp;<font size="-1"><a href="copy://%s" style="color:gray;">copy</a></font>' % (topic.name)
            text += '<br><span style="color:gray;">ns: </span><b>%s%s</b>' % (ns, sep)
            text += '<br>'
            if nm.settings().transpose_pub_sub_descr:
                text += self._create_html_list('<br>Subscriber:', topic.subscriberNodes, 'NODE')
                text += self._create_html_list('<br>Publisher:', topic.publisherNodes, 'NODE')
            else:
                text += self._create_html_list('<br>Publisher:', topic.publisherNodes, 'NODE')
                text += self._create_html_list('<br>Subscriber:', topic.subscriberNodes, 'NODE')
            text += '<br>'
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
                        for n in sorted(constants.keys()):
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
            selectedServices = self.servicesFromIndexes(self.ui.servicesView.selectionModel().selectedIndexes())
            self.ui.callServiceButton.setEnabled(len(selectedServices) > 0)
            if not self._is_current_tab_name('tabServices'):
                return
        if len(selectedServices) == 1:
            text = ''
            service = selectedServices[0]
            ns, sep, name = service.name.rpartition(rospy.names.SEP)
            # text = '<font size="+1"><b><span style="color:gray;">%s%s</span><b>%s</b></font><br>' % (ns, sep, name)
            sekkyumu_call_service_24 = nm.settings().icon_path('sekkyumu_call_service_24.png')
            text += '<a href="servicecall://%s%s" title="call service"><img src="%s" alt="call"></a>' % (self.mastername, service.name, sekkyumu_call_service_24)
            text += '<br><font size="+2"><b>%s</b></font>' % (name)
            text += '&nbsp;<font size="-1"><a href="copy://%s" style="color:gray;">copy</a></font>' % (service.name)
            text += '<br><span style="color:gray;">ns: </span><b>%s%s</b>' % (ns, sep)
            text += '<dl>'
            text += '<dt><b>URI</b>: %s</dt>' % service.uri
            text += '<dt><b>ORG.MASTERURI</b>: %s</dt>' % service.masteruri
            text += self._create_html_list('<br>Provider:', service.serviceProvider, 'NODE')
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
            except Exception:
                text += '<h4><font color=red>Unknown service type</font></h4>'
                if service.isLocal:
                    text += '<font color=red>Unable to communicate with service, is provider node running?</font>'
                else:
                    text += '<font color=red>Try to refresh <b>all</b> hosts. Is provider node running?</font>'
            info_text = '<div>%s</div>' % text
            self._is_current_tab_name('tabServices')
            if (self._is_current_tab_name('tabServices') and self.__last_info_text != info_text) or force_emit:
                self.__last_info_text = info_text
                self.description_signal.emit('Service - Info', info_text, True if selected or deselected or force_emit else False)

    def _href_from_svrtype(self, srv_type):
        result = srv_type
        if srv_type:
            result = '<a href="http://ros.org/doc/api/%s.html">%s</a>' % (srv_type.replace('/', '/html/srv/'), srv_type)
        return result

    def on_parameter_selection_changed(self, selected, deselected):
        selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
        self.ui.deleteParameterButton.setEnabled(len(selectedParameter) > 0)
        self.ui.saveParameterButton.setEnabled(len(selectedParameter) > 0)
        self.ui.transferParameterButton.setEnabled(len(selectedParameter) > 0)

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
        regex = QRegExp(self.ui.nodeFilterInput.text())
        for index in indexes:
            if index.column() == 0:
                model_index = self.node_proxy_model.mapToSource(index)
                item = self.node_tree_model.itemFromIndex(model_index)
                res = self._nodesFromItems(item, recursive)
                for r in res:
                    if r not in result and (not regex.pattern() or regex.indexIn(r.name) != -1):
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

    def topicsFromIndexes(self, indexes, recursive=True):
        result = []
        for index in indexes:
            model_index = self.topic_proxyModel.mapToSource(index)
            item = self.topic_model.itemFromIndex(model_index)
            if item is not None:
                if isinstance(item, TopicItem):
                    result.append(item.topic)
                elif recursive and isinstance(item, TopicGroupItem):
                    for titem in item.get_topic_items():
                        result.append(titem.topic)
        return result

    def servicesFromIndexes(self, indexes):
        result = []
        for index in indexes:
            model_index = self.service_proxyModel.mapToSource(index)
            item = self.service_model.itemFromIndex(model_index)
            if item is not None:
                if isinstance(item, ServiceItem):
                    result.append(item.service)
                elif isinstance(item, ServiceGroupItem):
                    for sitem in item.get_service_items():
                        result.append(sitem.service)
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
        self.ui.startButton.setEnabled(False)
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            self.start_nodes(selectedNodes)
        finally:
            self.setCursor(cursor)
            self.ui.startButton.setEnabled(True)

    def on_start_alt_clicked(self):
        '''
        Starts the selected nodes with additional options.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            self.start_nodes(selectedNodes, force=True, use_adv_cfg=True)
        finally:
            self.setCursor(cursor)

    def start_node(self, node, force, config, force_host=None, logging=None, cmd_prefix='', path=''):

        if node is None:
            raise DetailedError("Start error", 'None is not valid node name!')
        if node.pid is None or force:
            # start the node using launch configuration
            if config is None:
                raise DetailedError("Start error",
                                    'Error while start %s:\nNo configuration found!' % node.name)
            try:
                reload_global_param = False
                if not self.__configs[config].global_param_done:
                    reload_global_param = True
                    self.__configs[config].global_param_done = True
                loglevel = ''
                logformat = ''
                if logging is not None:
                    if not logging.is_default('console_format'):
                        logformat = logging.console_format
                    if not logging.is_default('loglevel'):
                        loglevel = logging.loglevel
                if self._has_nmd:
                    _result = nm.nmd().launch.start_node(node.name, config, self.masteruri, reload_global_param=reload_global_param,
                                                         loglevel=loglevel, logformat=logformat, path=path, cmd_prefix=cmd_prefix)
                else:
                    rospy.logwarn("no running daemon found, start '%s' via SSH" % node.name)
                    nm.starter().bc_run_node(node.name, config, self.masteruri, reload_global_param=reload_global_param,
                                             loglevel=loglevel, logformat=logformat)
            except socket.error as se:
                rospy.logwarn("Error while start '%s': %s\n\n Start canceled!", node.name, utf8(se))
                raise DetailedError("Start error",
                                    'Error while start %s\n\nStart canceled!' % node.name,
                                    '%s' % utf8(se))
            except nm.InteractionNeededError as _:
                raise
            except nm.BinarySelectionRequest as bsr:
                raise nm.InteractionNeededError(bsr, self.start_node, {'node': node, 'force': force, 'config': config, 'force_host': force_host, 'logging': logging, 'cmd_prefix': cmd_prefix, 'path': path})
            except (exceptions.StartException, nm.StartException) as e:
                rospy.logwarn("Error while start '%s': %s" % (node.name, utf8(e)))
                lines = utf8(e).splitlines()
                last_line = lines[-1]
                for line in lines:
                    if line:
                        last_line = line
                raise DetailedError("Start error", 'Error while start %s:\n%s' % (node.name, last_line), '%s' % utf8(e))
            except Exception as e:
                print(type(e))
                print(traceback.format_exc(3))
                rospy.logwarn("Error while start '%s': %s" % (node.name, utf8(e)))
                raise DetailedError("Start error", 'Error while start %s' % node.name, '%s' % utf8(e))

    def start_nodes(self, nodes, force=False, force_host=None, use_adv_cfg=False, check_nodelets=True):
        '''
        Internal method to start a list with nodes

        :param nodes: the list with nodes to start
        :type nodes: list(:class:`fkie_node_manager.node_tree_model.NodeItem`)
        :param bool force: force the start of the node, also if it is already started.
        :param str force_host: force the start of the node at specified host.
        '''
        cfg_choices = dict()
        cfg_nodes = dict()
#        has_launch_files = False
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
                    cfg_nodes[node.name] = lcfg
                    node.launched_cfg = lcfg
                    node.next_start_cfg = None
                else:
                    choices = self._get_cfg_choises(node)
                    ch_keys = list(choices.keys())
                    if ch_keys:
                        ch_keys.sort()
                        choises_str = utf8(ch_keys)
                        if choises_str not in list(cfg_choices.keys()):
                            choice, ok = self._get_cfg_userchoice(choices, node.name)
                            if choice is not None:
                                cfg_choices[choises_str] = choices[choice]
                                cfg_nodes[node.name] = choices[choice]
                                node.launched_cfg = choices[choice]
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
        cmd_prefix = ''
        if use_adv_cfg:
            log_params = {'Level': {':type': 'string', ':value': nm.settings().logging.get_alternatives('loglevel')},
                          # 'Level (roscpp)': ('string', nm.settings().logging.get_alternatives('loglevel_roscpp')),
                          # 'Level (super)': ('string', nm.settings().logging.get_alternatives('loglevel_superdebug')),
                          'Format': {':type': 'string', ':value': nm.settings().logging.get_alternatives('console_format')}
                          }
            params = {'Prefix': {':type': 'string',
                                 ':value': ['', 'gdb -ex run --args', 'valgrind', 'python -m pdb'],
                                 ':hint': 'Custom command prefix. It will be prepended before launch prefix.'
                                 },
                      'Logging': log_params}
            dia = ParameterDialog(params, store_geometry="adv_cfg_dialog")
            dia.setFilterVisible(False)
            dia.setWindowTitle('Start with parameters')
            dia.setFocusField('Level')
            diag_canceled = not dia.exec_()
            if not diag_canceled:
                try:
                    params = dia.getKeywords()
                    nm.settings().logging.loglevel = params['Logging']['Level']
                    # nm.settings().logging.loglevel_roscpp = params['Logging']['Level (roscpp)']
                    # nm.settings().logging.loglevel_superdebug = params['Logging']['Level (super)']
                    nm.settings().logging.console_format = params['Logging']['Format']
                    nm.settings().store_logging()
                    logging = nm.settings().logging
                    cmd_prefix = params['Prefix']
                except Exception as e:
                    diag_canceled = True
                    MessageBox.warning(self, "Get advanced start parameter",
                                       'Error while parse parameter',
                                       utf8(e))
        if not diag_canceled:
            # check for nodelets
            if check_nodelets:
                pass
                # self._check_for_nodelets(nodes)
            all2start = set()
            # put into the queue and start
            for node in nodes:
                if node.name in cfg_nodes and not node.name in all2start:
                    # remove node from question
                    self.message_frame.hide_question([MessageFrame.TYPE_BINARY], MessageData(node))
                    # add associated nodes to start
                    associated2start = self._get_associated_nodes([node.name], ignore=all2start)
                    all2start |= associated2start
                    found_nodes = self._get_nodes_by_name(list(associated2start))
                    for anode in found_nodes:
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'start %s' % anode.name,
                                                       self.start_node,
                                                       {'node': anode.node_info,
                                                        'force': force,
                                                        'config': cfg_nodes[node.node_info.name],
                                                        'force_host': force_host
                                                       })
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   'start %s' % node.node_info.name,
                                                   self.start_node,
                                                   {'node': node.node_info,
                                                    'force': force,
                                                    'config': cfg_nodes[node.node_info.name],
                                                    'force_host': force_host,
                                                    'logging': logging,
                                                    'cmd_prefix': cmd_prefix
                                                   })
        self._start_queue(self._progress_queue)

    def _check_for_nodelets(self, nodes):
        self._restart_nodelets = {}
        nodenames = [n.name for n in nodes]
        nodelet_mngr = ''
        nlmngr = ''
        for node in nodes:
            try:
                cfg_name = node.launched_cfg
                if isinstance(node.launched_cfg, LaunchConfig):
                    cfg_name = node.launched_cfg.Filename
                nodelets = self._get_nodelets(node.name, cfg_name)
                if nodelets:
                    nodelets = self._get_nodelets(node.name, cfg_name)
                    r_nn = []
                    for nn in nodelets:
                        if nn not in nodenames:
                            r_nn.append(nn)
                            if cfg_name not in self._restart_nodelets:
                                self._restart_nodelets[cfg_name] = []
                            self._restart_nodelets[cfg_name].append(nn)
                    if self._restart_nodelets:
                        nlmngr = node.name
                else:
                    nodelet_mngr = self._get_nodelet_manager(node.name, cfg_name)
                    if nodelet_mngr:
                        if nodelet_mngr not in nodenames:
                            if cfg_name not in self._restart_nodelets:
                                self._restart_nodelets[cfg_name] = []
                            self._restart_nodelets[cfg_name].append(nodelet_mngr)
                            nodelets = self._get_nodelets(nodelet_mngr, cfg_name)
                            r_nn = []
                            for nn in nodelets:
                                if nn not in nodenames:
                                    r_nn.append(nn)
                                    self._restart_nodelets[cfg_name].append(nn)
                            nodelet_mngr = nodelet_mngr
            except Exception as err:
                rospy.logwarn("Error while test for nodelets: %s" % utf8(err))
        if nm.settings().check_for_nodelets_at_start:
            if nodelet_mngr and nodelet_mngr not in nodenames:
                self.message_frame.show_question(MessageFrame.TYPE_NODELET, "Nodelet manager '%s' not in current list. (Re)Start nodelet manager and all nodelets?" % nodelet_mngr, MessageData(self._restart_nodelets))
            elif self._restart_nodelets:
                self.message_frame.show_question(MessageFrame.TYPE_NODELET, "Not all nodelets of manager '%s' are in the start list. (Re)Start these?" % nlmngr, MessageData(self._restart_nodelets))

    def start_nodes_by_name(self, nodes, cfg, force=False, check_nodelets=True):
        '''
        Start nodes given in a list by their names.

        :param nodes: a list with full node names
        :type nodes: list(str)
        '''
        result = []
        config = cfg
        if isinstance(cfg, LaunchConfig):
            config = cfg.launchname
        if self.master_info is not None:
            for n in nodes:
                node_items = self.getNode(n)
                if node_items:
                    node_item = node_items[0]
                    node_item.next_start_cfg = config
                elif config:
                    node_info = NodeInfo(n, self.masteruri)
                    node_item = NodeItem(node_info)
                    node_item.next_start_cfg = config
                if node_item is not None:
                    result.append(node_item)
        self.start_nodes(result, force, check_nodelets=check_nodelets)

    def start_nodes_after_load_cfg(self, cfg_name, nodes, force=False):
        '''
        Start nodes after the given configuration is loaded and applied to the model.

        :param cfg_name: the name of the cnofiguration
        :type cfg_name: str
        :param nodes: the list of node names
        :type nodes: list(str)
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
        self.ui.startButton.setEnabled(False)
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
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
            self.ui.startButton.setEnabled(True)

    def on_start_nodes_at_host(self):
        '''
        Starts the selected nodes on an another host.
        :TODO: remove this method or adapt to new ParameterDailaog
        '''
        cursor = self.cursor()
        self.ui.startButton.setEnabled(False)
        params = {'Host': {':type': 'string', ':value': 'localhost'}}
        dia = ParameterDialog(params, store_geometry="start_node_at_host_dialog")
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start node on...')
        dia.setFocusField('host')
        if dia.exec_():
            try:
                params = dia.getKeywords()
                host = params['Host']
                self.setCursor(Qt.WaitCursor)
                try:
                    selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
                    self.start_nodes(selectedNodes, True, host)
                finally:
                    self.setCursor(cursor)
            except Exception as e:
                MessageBox.warning(self, "Start error",
                                   'Error while parse parameter',
                                   utf8(e))
        self.ui.startButton.setEnabled(True)

    def _get_cfg_choises(self, node, ignore_defaults=False):
        result = {}
        for c in node.cfgs:
            if c and not isinstance(c, tuple):
                # TODO: create name
                result[c] = c
#                 if not isinstance(c, tuple):
#                     launch = self.launchfiles[c]
#                     result[''.join([utf8(launch.LaunchName), ' [', utf8(launch.PackageName), ']'])] = self.launchfiles[c]
#                 elif not ignore_defaults:
#                     result[' '.join(['[default]', c[0]])] = roslib.names.ns_join(c[0], 'run')
        return result

    def _get_cfg_userchoice(self, choices, nodename):
        value = None
        ok = False
        # Open selection
        if len(choices) == 1:
            value = list(choices.keys())[0]
            ok = True
        elif len(choices) > 0:
            items, ok = SelectDialog.getValue('Configuration selection', 'Select configuration to launch <b>%s</b>' % nodename, list(choices.keys()), True, store_geometry='cfg_select')
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
            self.ui.stopButton.showMenu()
        else:
            cursor = self.cursor()
            self.setCursor(Qt.WaitCursor)
            try:
                selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
                self.stop_nodes(selectedNodes)
            finally:
                self.setCursor(cursor)

    def stop_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            success = False
            try:
                rospy.loginfo("Stop node '%s'[%s]", utf8(node.name), utf8(node.uri))
                socket.setdefaulttimeout(10)
                p = xmlrpcclient.ServerProxy(node.uri)
                (code, statusMessage, ignore) = p.shutdown(rospy.get_name(), '[node manager] request from %s' % self.mastername)
                if code == 1:
                    success = True
                else:
                    rospy.logwarn("Error while shutdown node '%s': %s", utf8(node.name), utf8(statusMessage))
            except Exception as e:
                err_msg = utf8(e)
                if ' 111' in err_msg:
                    # this error occurs sometimes after shutdown node
                    rospy.logdebug("Error while stop node '%s': %s", utf8(node.name), utf8(e))
                else:
                    rospy.logwarn("Error while stop node '%s': %s", utf8(node.name), utf8(e))
            finally:
                socket.setdefaulttimeout(None)
            # wait kill_on_stop is an integer
            if node.pid is not None:
                if hasattr(node, 'kill_on_stop') and type(node.kill_on_stop) in [int, float]:
                    time.sleep(float(node.kill_on_stop) / 1000.0)
                    nm.nmd().monitor.kill_process(node.pid, nmdurl.nmduri(node.masteruri))
                elif not success:
                    if node.name != '/node_manager_daemon':
                        rospy.loginfo("Try to kill process %d of the node: %s", node.pid, utf8(node.name))
                        nm.nmd().monitor.kill_process(node.pid, nmdurl.nmduri(node.masteruri))
        elif isinstance(node, NodeItem) and node.is_ghost:
            # since for ghost nodes no info is available, emit a signal to handle the
            # stop message in other master_view_proxy
            self.stop_nodes_signal.emit(node.masteruri, [node.name])
        return True

    def stop_nodes(self, nodes, force=False):
        '''
        Internal method to stop a list with nodes

        :param nodes: the list with nodes to stop
        :type nodes: list(:class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>)
        '''
        # put into the queue and start the queue handling
        for node in nodes:
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'stop %s' % node.name,
                                           self.stop_node,
                                           {'node': node, 'force': (len(nodes) == 1) or force})
        self._start_queue(self._progress_queue)
        # add associated nodes to stop
        associated2stop = self._get_associated_nodes([node.name for node in nodes])
        found_nodes = self._get_nodes_by_name(list(associated2stop))
        for node in found_nodes:
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'stop %s' % node.name,
                                           self.stop_node,
                                           {'node': node, 'force': (len(nodes) == 1) or force})
        self._start_queue(self._progress_queue)

    def stop_nodes_by_name(self, nodes, force=False, ignore=[], only_local=True):
        '''
        Stop nodes given in a list by their names.

        :param nodes: a list with full node names
        :type nodes: list(str)
        '''
        found_nodes = self._get_nodes_by_name(nodes, ignore, only_local)
        self.stop_nodes(found_nodes, force)

    def _get_nodes_by_name(self, nodes, ignore=[], only_local=True):
        found_nodes = []
        if self.master_info is not None:
            req_nodes = []
            md_node = False
            for n in nodes:
                if n not in ignore:
                    if '/master_dsicovery' == n:
                        md_node = True
                    else:
                        req_nodes.append(n)
            if md_node:
                req_nodes.append('/master_dsicovery')
            found_nodes = self.node_tree_model.get_node_items_by_name(req_nodes, only_local)
        return found_nodes

    def _get_associated_nodes(self, nodenames, config='', ignore=set()):
        result = set()
        ignore_all = set(nodenames) | ignore
        with self._associations_lock:
            for nodename in nodenames:
                if config:
                    if config in self._associations:
                        associations = self._associations[config]
                        associated_nodes = set(associations[nodename])
                        result |= associated_nodes - ignore_all
                        result |= self._get_associated_nodes(list(result), config, ignore=ignore_all)
                else:
                    for _cfg, associations in self._associations.items():
                        if nodename in associations:
                            associated_nodes = set(associations[nodename])
                            new_nodes = associated_nodes - ignore_all
                            ignore_all |= new_nodes
                            result |= new_nodes
                            result |= self._get_associated_nodes(list(new_nodes), config, ignore_all)
        return result

    def kill_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            pid = node.pid
            if pid is None:
                # try to get the process id of the node
                try:
                    socket.setdefaulttimeout(10)
                    rpc_node = xmlrpcclient.ServerProxy(node.uri)
                    _, _, pid = rpc_node.getPid(rospy.get_name())  # _:=code, msg
                except Exception:
                    pass
                finally:
                    socket.setdefaulttimeout(None)
            # kill the node
            if pid is not None:
                try:
                    if self._has_nmd:
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'kill %s (%s)' % (node.name, utf8(pid)),
                                                       nm.nmd().monitor.kill_process,
                                                       {'pid': pid, 'grpc_url': self._grpc_from_node(node)})
                    else:
                        self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                       'kill %s (%s)' % (node.name, utf8(pid)),
                                                       nm.starter().kill,
                                                       {'host': self.getHostFromNode(node),
                                                        'pid': pid,
                                                        'auto_pw_request': False,
                                                        'user': self.current_user
                                                       })
                    self._start_queue(self._progress_queue)
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
                                                   {'host': host, 'user': self.current_user})
                    self._start_queue(self._progress_queue)
                else:
                    nm.starter().killall_roscore(host, self.current_user)
            except Exception as e:
                rospy.logwarn("Error while killall roscore on %s: %s" % (host, utf8(e)))
                raise DetailedError("Killall roscore error",
                                    'Error while killall roscore',
                                    '%s' % utf8(e))
        return True

    def on_kill_pid(self, pid):
        ret = MessageBox.question(self, "Kill Process", "You are going to kill process with ID %d\nContinue?" % pid, buttons=MessageBox.Ok | MessageBox.Cancel)
        ret = (ret == MessageBox.Ok)
        if ret:
            host = get_hostname(self.masteruri)
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'kill %d' % pid,
                                           nm.starter().kill,
                                           {'host': host,
                                            'pid': pid
                                           })                                           
            self._start_queue(self._progress_queue)

    def on_kill_nodes(self):
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())

        # put into the queue and start the que handling
        for node in selectedNodes:
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                           'kill %s' % node.name,
                                           self.kill_node,
                                           {'node': node, 'force': (len(selectedNodes) == 1)})
        self._start_queue(self._progress_queue)

    def unregister_node(self, node, force=False):
        if node is not None and node.uri is not None and (not self._is_in_ignore_list(node.name) or force):
            # stop the node?
            # try:
            #   p = xmlrpclib.ServerProxy(node.uri)
            #   p.shutdown(rospy.get_name(), ''.join(['[node manager] request from ', self.hostname]))
            # except Exception, e:
            #   rospy.logwarn("Error while stop node '%s': %s", utf8(node.name), utf8(e))
            #   self.ui.stopButton.setEnabled(False)
            # unregister all entries of the node from ROS master
            try:
                socket.setdefaulttimeout(10)
                master = xmlrpcclient.ServerProxy(node.masteruri)
                master_multi = xmlrpcclient.MultiCall(master)
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
            except Exception as e:
                rospy.logwarn("Error while unregister node %s: %s", utf8(node.name), utf8(e))
                raise DetailedError("Unregister error",
                                    ''.join(['Error while Unregister node ', node.name]),
                                    utf8(e))
            finally:
                socket.setdefaulttimeout(None)
        return True

    def on_unregister_nodes(self):
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
        # put into the queue and start the que handling
        for node in selectedNodes:
            if node.pid is None or len(selectedNodes) == 1:
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               'unregister node %s' % node.name,
                                               self.unregister_node,
                                               {'node': node, 'force': (len(selectedNodes) == 1)})
        self._start_queue(self._progress_queue)

    def on_stop_context_toggled(self, state):
        menu = QMenu(self)
        self.killAct = QAction("&Kill Node", self, shortcut=QKeySequence.New, statusTip="Kill selected node", triggered=self.kill_nodes)
        self.unregAct = QAction("&Unregister Nodes...", self, shortcut=QKeySequence.Open, statusTip="Open an existing file", triggered=self.unreg_nodes)
        menu.addAction(self.killAct)
        menu.addAction(self.unregAct)
        menu.exec_(self.ui.stopContextButton.pos())

    def getHostFromNode(self, node):
        '''
        If the node is running the host the node URI will be returned. Otherwise
        tries to get the host from the launch configuration. If the configuration
        contains no machine assignment for this node the host of the ROS master URI
        will be used.

        :param node:
        :type node: :class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>
        '''
        if node.uri is not None:
            return get_hostname(node.uri)
        # get hostname from host item where the node is located
        host = node.host
        if host:
            return host
        # try to get it from the configuration,
        # TODO: get it from node manager daemon?
        for c in node.cfgs:
            if isstring(c):
                launch_config = self.__configs[c]
                if node.name in launch_config.nodes:
                    url, _path = nmdurl.split(c, with_scheme=True)
                    return get_hostname(url)
#                 if item is not None and item.machine_name and not item.machine_name == 'localhost':
#                     return launch_config.Roscfg.machines[item.machine_name].address
        # return the host of the assigned ROS master
        return get_hostname(node.masteruri)

    def _grpc_from_node(self, node):
        '''
        If the node is running the grpc url of the masteruri of the node will be returned. Otherwise
        tries to get the grpc from the launch configuration. Or the ROS master URI
        will be used.

        :param node:
        :type node: :class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>
        '''
        # get hostname from host item where the node is located
        host = node.host
        if host:
            return nmdurl.nmduri('http://%s:%d' % (host, get_port(self.masteruri)))
        if node.masteruri is not None:
            return nmdurl.nmduri(node.masteruri)
        # try to get it from the configuration,
        # TODO: get it from node manager daemon?
        for c in node.cfgs:
            if isstring(c):
                launch_config = self.__configs[c]
                if node.name in launch_config.nodes:
                    url, _path = nmdurl.split(c, with_scheme=True)
                    return url
        return nmdurl.nmduri(self.masteruri)

    def on_io_clicked(self, activated=False):
        '''
        Shows IO of the selected nodes.
        '''
        # key_mod = QApplication.keyboardModifiers()
        # use_mod = key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier
        # if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
        #     self.ui.ioButton.showMenu()
        # else:
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            if selectedNodes:
                ret = True
                if len(selectedNodes) > 5:
                    ret = MessageBox.question(self, "Show IO", "You are going to open the IO of " + utf8(len(selectedNodes)) + " nodes at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
                    ret = (ret == MessageBox.Ok)
                if ret:
                    queue = self._progress_queue_prio
                    # we use normal queue, if there are not a lot of processes
                    if self._progress_queue.count() < 5:
                        queue = self._progress_queue
                    key_mod = QApplication.keyboardModifiers()
                    use_log_widget = nm.settings().use_internal_log_widget
                    if activated and (key_mod & Qt.ShiftModifier):
                        # show ROS log if shift or control was pressed while activating
                        if use_log_widget or key_mod & Qt.ControlModifier:
                            for node in selectedNodes:
                                self.main_window.open_screen_dock(self.masteruri, screen_name='', nodename=node.name, user=self.current_user)
                        else:
                            for node in selectedNodes:
                                queue.add2queue(utf8(uuid.uuid4()),
                                                'show log of %s' % node.name,
                                                nm.starter().openLog,
                                                {'nodename' : node.name,
                                                'host': self.getHostFromNode(node),
                                                'user': self.current_user,
                                                'only_screen': False
                                                })
                    else:
                        for node in selectedNodes:
                            queue.add2queue(utf8(uuid.uuid4()),
                                            'show IO of %s' % node.name,
                                            nm.screen().open_screen,
                                            {'node': node.name,
                                            'grpc_url': self._grpc_from_node(node),
                                            'auto_item_request': False,
                                            'use_log_widget': activated and (use_log_widget or key_mod & Qt.ControlModifier),
                                            'user': self.current_user,
                                            'pw': None,
                                            'items': [],
                                            'use_nmd': self._has_nmd
                                            })
                    self._start_queue(queue)
            else:
                self.on_show_all_screens()
        finally:
            self.setCursor(cursor)

    def _on_no_screen_error(self, nodename, host):
        msg = nm.NoScreenOpenLogRequest(nodename, host).msg()
        rospy.loginfo('%s' % msg)
        muris = nm.nameres().masterurisbyaddr(host)
        for muri in muris:
            if muri == self.masteruri:
                nodes = self.node_tree_model.get_tree_node(nodename, muri)
                for node in nodes:
                    node.has_screen = False
        if nm.settings().show_noscreen_error:
            self.info_frame.show_info(MessageFrame.TYPE_NOSCREEN, 'No screens found for:', MessageData('', [nodename]))

    def on_kill_screens(self):
        '''
        Kills selected screens, if some available.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                if not self._is_in_ignore_list(node.name):
                    self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                                   "kill screen of %s" % node.name,
                                                   nm.screen().kill_screens,
                                                   {'node': node.name,
                                                    'grpc_url': self._grpc_from_node(node),
                                                    'auto_ok_request': False,
                                                    'user': self.current_user
                                                   })
            self._start_queue(self._progress_queue)
        finally:
            self.setCursor(cursor)

    def on_show_all_screens(self):
        '''
        Shows all available screens.
        '''
        cursor = self.cursor()
        self.setCursor(Qt.WaitCursor)
        try:
            grpc_url = nmdurl.nmduri(self.masteruri)
            sel_screen = []
            try:
                screens = nm.nmd().screen.get_all_screens(grpc_url)
                sel_screen, _ok = SelectDialog.getValue('Open screen', '', list(screens.keys()), False, False, self, store_geometry='open_screen')
            except Exception as e:
                rospy.logwarn("Error while get screen list: %s", utf8(e))
                MessageBox.warning(self, "Screen list error",
                                   "Error while get screen list from '%s'" % grpc_url,
                                   utf8(e))
            host = get_hostname(self.masteruri)
            for screen in sel_screen:
                try:
                    if not nm.screen().open_screen_terminal(self.masteruri, screen, screen, False, self.current_user):
                        pass
                except Exception as e:
                    rospy.logwarn("Error while show IO for %s: %s", utf8(screen), utf8(e))
                    MessageBox.warning(self, "Show IO error",
                                       "Error while show IO '%s' on '%s'" % (screen, host),
                                       utf8(e))
        finally:
            self.setCursor(cursor)

    def on_multiple_screens(self, grpc_url, screens):
        muri = nmdurl.masteruri(grpc_url)
        self.node_tree_model.clear_multiple_screens(muri)
        for node, screens in screens.items():
            nodes = self.node_tree_model.get_tree_node(node, muri)
            for node in nodes:
                node.has_multiple_screens = True

    def on_log_clicked(self):
        '''
        Shows log files of the selected nodes.
        '''
        try:
            only_screen = True
            key_mod = QApplication.keyboardModifiers()
            if (key_mod & Qt.ShiftModifier or key_mod & Qt.ControlModifier):
                only_screen = False
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            ret = True
            if len(selectedNodes) > 5:
                ret = MessageBox.question(self, "Show Log", "You are going to open the logs of " + utf8(len(selectedNodes)) + " nodes at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
                ret = (ret == MessageBox.Ok)
            if ret:
                for node in selectedNodes:
                    self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                        'show log of %s' % node.name,
                                                        nm.starter().openLog,
                                                        {'nodename' : node.name,
                                                         'host': self.getHostFromNode(node),
                                                         'user': self.current_user,
                                                         'only_screen': only_screen
                                                        })
                self._start_queue(self._progress_queue_prio)
        except Exception as e:
            print(traceback.format_exc(3))
            rospy.logwarn("Error while show log: %s", utf8(e))
            MessageBox.warning(self, "Show log error",
                               'Error while show Log',
                               utf8(e))

    def show_log(self, nodename, host, roslog=True):
        self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                            'show log of %s' % nodename,
                                            nm.starter().openLog,
                                            {'nodename' : nodename,
                                             'host': host,
                                             'user': self.current_user,
                                             'only_screen': not roslog,
                                             'only_roslog': roslog
                                            })
        self._start_queue(self._progress_queue_prio)

    def on_log_path_copy(self):
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
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

    def on_log_delete_clicked(self):
        '''
        Deletes log files of the selected nodes.
        '''
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            self._progress_queue_prio.add2queue(utf8(uuid.uuid4()),
                                                "delete Log of '%s'" % node.name,
                                                nm.starter().delete_log,
                                                {'nodename': node.name,
                                                 'grpc_uri': self._grpc_from_node(node),
                                                 'auto_pw_request': False,
                                                 'user': self.current_user
                                                })
        self._start_queue(self._progress_queue_prio)

    def on_dynamic_config_clicked(self):
        '''
        Opens the dynamic configuration dialogs for selected nodes.
        '''
        if self.master_info is not None:
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            for n in selectedNodes:
                try:
                    nodes = sorted([srv_name[:-len('/set_parameters')] for srv_name, srv in self.master_info.services.items() if (srv_name.endswith('/set_parameters') and n.name in srv.serviceProvider)])
                    items = []
                    if len(nodes) == 1:
                        items = nodes
                    elif len(nodes) > 1:
                        items, _ = SelectDialog.getValue('Dynamic configuration selection', '', [i for i in nodes], store_geometry='dynamic_cfg')
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
                        _ = SupervisedPopen(['rosrun', 'fkie_node_manager', 'dynamic_reconfigure', node, '__ns:=dynamic_reconfigure'], env=env, object_id=node, description='Start dynamic reconfiguration for %s failed' % node)
                except Exception as e:
                    rospy.logwarn("Start dynamic reconfiguration for '%s' failed: %s" % (n.name, utf8(e)))
                    MessageBox.warning(self, "Start dynamic reconfiguration error",
                                       'Start dynamic reconfiguration for %s failed!' % n.name,
                                       utf8(e))

    def on_edit_config_clicked(self):
        '''
        '''
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            choices = self._get_cfg_choises(node, True)
            choice, ok = self._get_cfg_userchoice(choices, node.name)
            config = choices[choice] if choices and choice else ''
            if ok and config:
                self.request_xml_editor.emit(config, 'name="%s"' % os.path.basename(node.name))

    def on_edit_rosparam_clicked(self):
        '''
        '''
        selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
        for node in selectedNodes:
            # set the parameter in the ROS parameter server
            try:
                inputDia = MasterParameterDialog(node.masteruri if node.masteruri is not None else self.masteruri, ''.join([node.name, roslib.names.SEP]), parent=self, store_geometry="edit_param_dialog")
                inputDia.setWindowTitle('%s - %s' % (os.path.basename(node.name), "parameter"))
                if node.has_launch_cfgs(node.cfgs):
                    inputDia.add_warning("The changes may not have any effect, because the launch file was also loaded and the parameter in the launch file will be reloaded on restart of the ROS node.")
                inputDia.show()
            except Exception:
                rospy.logwarn("Error on retrieve parameter for %s: %s", utf8(node.name), traceback.format_exc(1))

    def on_close_clicked(self):
        '''
        Opens a dialog to select configurations to close or stop all nodes
        (with roscore) or shutdown the host.
        '''
        choices = dict()

        for grpc_path, _ in self.__configs.items():
            try:
                package = utf8(package_name(grpc_path)[0])
                choices['%s [%s]' % (os.path.basename(grpc_path), package)] = grpc_path
            except ValueError as val_err:
                rospy.logwarn(val_err)
        cfg_items = list(choices.keys())
        cfg_items.sort()
        res = SelectDialog.getValue('Close/Stop/Shutdown', '',
                                    cfg_items, False, False,
                                    self, checkitem1='stop ROS',
                                    checkitem2='shutdown host',
                                    store_geometry='close_cfg')
        cfgs, _, stop_nodes, shutdown = res[0], res[1], res[2], res[3]
        # close configurations
        for config in cfgs:
            self._close_cfg(choices[config])
        if stop_nodes:
            self._on_stop_kill_roscore = True
            # stop all nodes, system nodes at the end
            ignore_nodes = [rospy.get_name(), '/master_discovery', '/rosout']
            self.stop_nodes_by_name(self.get_nodes_runningIfLocal(), True, ignore_nodes)
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
                                           {'host': '%s' % self.mastername})
            self._start_queue(self._progress_queue)
        except (Exception, nm.StartException) as emsg:
            rospy.logwarn("Error while poweroff %s: %s", self.mastername, utf8(emsg))
            MessageBox.warning(self, "Run error",
                               'Error while poweroff %s' % self.mastername,
                               '%s' % utf8(emsg))

    def _close_cfg(self, cfg):
        try:
            self.remove_cfg_from_model(cfg)
            nm.nmd().launch.unload_launch(cfg, self.masteruri)
            del self.__configs[cfg]
            nm.nmd().launch.get_nodes_threaded(cfg)
        except exceptions.ResourceNotFound:
            del self.__configs[cfg]
            nm.nmd().launch.get_nodes_threaded(cfg)
        except Exception:
            rospy.logwarn(traceback.format_exc())

    def on_topic_echo_clicked(self, topics=[]):
        '''
        Shows the output of the topic in a terminal.
        '''
        self._show_topic_output(False, use_ssh=False, topics=topics)

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
        selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
        if len(selectedTopics) > 0:
            for topic in selectedTopics:
                if not self._start_publisher(topic.name, topic.type):
                    break
        else:  # create a new topic
            # fill the input fields
            # determine the list all available message types
            msg_types = []
            for ppath, pname in nm.nmd().file.get_packages(nmdurl.nmduri(self.masteruri)).items():
                #:TODO: get message types from remote nmduri
                _guri, lpath = nmdurl.split(ppath, with_scheme=False)
                import rosmsg
                for f in rosmsg._list_types('%s/msg' % lpath, 'msg', rosmsg.MODE_MSG):
                    msg_types.append("%s/%s" % (pname, f))
            msg_types.sort()
            fields = {'Type': {':type': 'string', ':value': msg_types}, 'Name': {':type': 'string', ':value': ['']}}
            # create a dialog
            dia = ParameterDialog(fields, parent=self, store_geometry="topic_pub_dialog")
            dia.setWindowTitle('Publish to topic')
            dia.setFilterVisible(False)
            dia.setFocusField('Name')
            if dia.exec_():
                params = dia.getKeywords()
                try:
                    if params['Name'] and params['Type']:
                        try:
                            self._start_publisher(params['Name'], params['Type'])
                        except Exception as e:
                            print(traceback.format_exc(1))
                            rospy.logwarn("Publish topic '%s' failed: %s", utf8(params['Name']), utf8(e))
                            MessageBox.warning(self, "Publish topic error",
                                               ''.join(['Publish topic ', params['Name'], ' failed!']),
                                               utf8(e))
                    else:
                        MessageBox.warning(self, "Invalid name or type",
                                           "Can't publish to topic '%s' with type '%s'!" % (params['Name'], params['Type']))
                except (KeyError, ValueError) as e:
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
                                   'invalid message type: %s\nIf this is a valid message type, perhaps you need to run "rosmake"' % topic_type)
                return
            slots = mclass.__slots__
            types = mclass._slot_types
            default_topic_values = {}
            rate_values = ['once', 'latch', '1']
            if republish and topic_name in self.__republish_params:
                default_topic_values = self.__republish_params[topic_name][topic_type]
                rate_values = self.__republish_params[topic_name]['! Publish rate']
            args = ServiceDialog._params_from_slots(slots, types, default_topic_values)
            p = {'! Publish rate': {':type': 'string', ':value': rate_values}, topic_type: args}
            dia = ParameterDialog(p, store_geometry="start_publisher_dialog")
            dia.setWindowTitle('Publish to %s' % topic_name)
            dia.showLoadSaveButtons()
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
                        except Exception:
                            i = float(rate)
                        if i > 0:
                            opt_str = ''.join(['-r ', rate])
                            opt_name_suf = '__%sHz_' % (utf8(rate).replace('.', '_'))
                    except Exception:
                        pass
                # remove empty lists
                topic_params = dict()
                if topic_type in params:
                    topic_params = self._rem_empty_lists(params[topic_type])
                pub_cmd = 'pub %s %s "%s" %s' % (topic_name, topic_type, str(topic_params), opt_str)
                rospy.logdebug("rostopic parameter: %s" % pub_cmd)
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               'start publisher for %s' % topic_name,
                                               nm.starter().runNodeWithoutConfig,
                                               {'host': nm.nameres().address(self.masteruri),
                                                'package': 'rostopic',
                                                'binary': 'rostopic',
                                                'name': 'rostopic_pub%s%s%s' % (topic_name, opt_name_suf, str(rospy.Time.now())),
                                                'args': [pub_cmd],
                                                'masteruri': self.masteruri,
                                                'use_nmd': True,
                                                'auto_pw_request': False,
                                                'user': self.current_user
                                               })
                self._start_queue(self._progress_queue)
                return True
            else:
                return False
        except Exception as e:
            rospy.logwarn("Publish topic '%s' failed: %s", utf8(topic_name), utf8(e))
            MessageBox.warning(self, "Publish topic error",
                               ''.join(['Publish topic ', topic_name, ' failed!']),
                               utf8(e))
            print(utf8(traceback.format_exc(1)))
            return False

    def _rem_empty_lists(self, param_dict):
        result = dict()
        for key, value in param_dict.items():
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
            selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
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

    def _show_topic_output(self, show_hz_only, use_ssh=False, topics=[]):
        '''
        Shows the output of the topic in a terminal.
        '''
        selected_topics = topics
        if not selected_topics:
            selected_topics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
        ret = True
        if len(selected_topics) > 5:
            ret = MessageBox.question(self, "Show echo", "You are going to open the echo of " + utf8(len(selected_topics)) + " topics at once\nContinue?", buttons=MessageBox.Ok | MessageBox.Cancel)
            ret = (ret == MessageBox.Ok)
        if ret:
            for topic in selected_topics:
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
            namespace = rospy.names.namespace(topic.name)
            nodename = os.path.basename(topic.name)
            namespace = '/echo_%s%s%s%s' % ('hz_' if show_hz_only else '', 'ssh_' if use_ssh else '', utf8(get_hostname(self.masteruri)), namespace)
            args = []
            # subscription parameter
            args.append("--echo %s %s %s %s" % (topic.name, topic.type, '--hz' if show_hz_only else '', '--ssh' if use_ssh else ''))
            args.append("__ns:=%s" % namespace)
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                            'start subcriber for %s' % topic.name,
                                            nm.starter().runNodeWithoutConfig,
                                            {'host': 'localhost',
                                             'package': 'fkie_node_manager',
                                             'binary': 'node_manager',
                                             'name': nodename,
                                             'args': args,
                                             'masteruri': self.masteruri,
                                             'use_nmd': False,
                                             'auto_pw_request': False,
                                             'user': self.current_user
                                            })
            self._start_queue(self._progress_queue)
            self.__echo_topics_dialogs.add(rospy.names.ns_join(namespace, nodename))
        except Exception as e:
            rospy.logwarn("Echo topic '%s' failed: %s" % (topic.name, utf8(e)))
            MessageBox.warning(self, "Echo of topic error",
                               'Echo of topic %s failed!' % topic.name,
                               '%s' % utf8(e))

    def on_service_call_clicked(self, services=[]):
        '''
        calls a service.
        '''
        selected_services = services
        if not selected_services:
            selected_services = self.servicesFromIndexes(self.ui.servicesView.selectionModel().selectedIndexes())
        try:
            for service in selected_services:
                param = ServiceDialog(service, self)
                param.show()
        except Exception as e:
            rospy.logwarn("Call service '%s' failed: %s" % (service.name, utf8(e)))
            MessageBox.warning(self, "Call service error",
                               'Call service %s failed!' % service.name,
                               '%s' % utf8(e))

    def service_call(self, service_name):
        service = self.master_info.getService(utf8(service_name))
        if service is not None:
            try:
                param = ServiceDialog(service, self)
                param.show()
            except Exception as e:
                rospy.logwarn("Call service '%s' failed: %s" % (service.name, utf8(e)))
                MessageBox.warning(self, "Call service error",
                                   'Call service %s failed!' % service.name,
                                   '%s' % utf8(e))

    def _restore_expand_state(self, tree_view, proxy_model):
        '''
        Expand the first item and all selected items.
        '''
        tree_view.collapseAll()
        for selected in tree_view.selectionModel().selectedIndexes():
            index = selected
            while index is not None and index.isValid():
                item = proxy_model.sourceModel().itemFromIndex(index)
                tree_view.setExpanded(index, True)
                index = index.parent()
        # expand the root item. NodesView has on sync also other hosts. In this case only local host will expanded.
        for root_idx in range(proxy_model.sourceModel().rowCount()):
            source_index = proxy_model.sourceModel().index(root_idx, 0)
            item = proxy_model.sourceModel().itemFromIndex(source_index)
            if type(item) in [HostItem] and not item._local:
                continue
            mapped_index = proxy_model.mapFromSource(source_index)
            tree_view.setExpanded(mapped_index, True)

    def on_node_filter_changed(self, text):
        '''
        Filter the displayed nodes
        '''
        self.node_proxy_model.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))
        if text:
            self.ui.nodeTreeView.expandAll()
        else:
            self._restore_expand_state(self.ui.nodeTreeView, self.node_proxy_model)

    def on_topic_filter_changed(self, text):
        '''
        Filter the displayed topics
        '''
        self.topic_proxyModel.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))
        if text:
            self.ui.topicsView.expandAll()
        else:
            self._restore_expand_state(self.ui.topicsView, self.topic_proxyModel)

    def on_service_filter_changed(self, text):
        '''
        Filter the displayed services
        '''
        self.service_proxyModel.setFilterRegExp(QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard))
        if text:
            self.ui.servicesView.expandAll()
        else:
            self._restore_expand_state(self.ui.servicesView, self.service_proxyModel)

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
        selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
        ns = '/'
        if selectedParameter:
            ns = roslib.names.namespace(selectedParameter[0][0])
        fields = {'name': {':value': ns}, 'type': {':type': 'string', ':value': ['string', 'int', 'float', 'bool', 'list']}, 'value': {':value': ''}}
        newparamDia = ParameterDialog(fields, parent=self, store_geometry="add_parameter_dialog")
        newparamDia.setWindowTitle('Add new parameter')
        newparamDia.setFilterVisible(False)
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
                        value = [ruamel.yaml.load(params['value'], Loader=ruamel.yaml.Loader)]
                        # if there is no YAML, load() will return an
                        # empty string.  We want an empty dictionary instead
                        # for our representation of empty.
                        if value is None:
                            value = []
                    except ruamel.yaml.MarkedYAMLError as e:
                        MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e), buttons=MessageBox.Ok)
                        return
                else:
                    value = params['value']
                self.parameterHandler.deliverParameter(self.masteruri, {params['name']: value})
                self.parameterHandler.requestParameterList(self.masteruri)
                self.sender().close()
            except (KeyError, ValueError) as e:
                MessageBox.warning(self, "Warning",
                                   'Error while add a parameter to the ROS parameter server',
                                   utf8(e))

    def on_delete_parameter_clicked(self):
        '''
        Deletes the parameter from the ROS parameter server.
        '''
        selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
        try:
            socket.setdefaulttimeout(15)
            name = rospy.get_name()
            master = xmlrpcclient.ServerProxy(self.masteruri)
            master_multi = xmlrpcclient.MultiCall(master)
            for (key, _) in selectedParameter:  # _ := value
                master_multi.deleteParam(name, key)
            r = master_multi()
            for code, msg, parameter in r:
                if code != 1:
                    rospy.logwarn("Error on delete parameter '%s': %s", parameter, msg)
        except Exception:
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
        selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
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
                        buf = ruamel.yaml.compat.StringIO()
                        ruamel.yaml.dump(values, buf, Dumper=ruamel.yaml.RoundTripDumper)
                        f.write(buf.getvalue())
                except Exception as e:
                    print(utf8(traceback.format_exc(1)))
                    MessageBox.warning(self, "Save parameter Error",
                                       'Error while save parameter',
                                       utf8(e))

    def on_transfer_parameter_clicked(self):
        '''
        Copy selected parameter to local ROS-Master.
        '''
        selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
        if selectedParameter:
            try:
                params = {}
                for (key, value) in selectedParameter:
                    params[key] = value
                if params:
                    dia_params = {'master': {':value': masteruri_from_ros()}}
                    dia = ParameterDialog(dia_params, store_geometry="transfer_param_dialog")
                    dia.setFilterVisible(False)
                    dia.setWindowTitle('Copy parameter')
                    dia.setFocusField('master')
                    if dia.exec_():
                        dia_params = dia.getKeywords()
                        url = dia_params['master']
                        rospy.loginfo("Copy %d parameter to %s" % (len(params), url))
                        self.parameterHandler.deliverParameter(url, params)
            except Exception as e:
                MessageBox.warning(self, "Copy parameter Error",
                                   'Error while transfer parameter',
                                   utf8(e))

    def _replaceDoubleSlash(self, liste):
        '''
        used to avoid the adding of \\ for each \ in a string of a list
        '''
        if liste and isinstance(liste, list):
            result = []
            for l in liste:
                val = l
                if isstring(l):
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
                        value = ruamel.yaml.load(item.text(), Loader=ruamel.yaml.Loader)
                        # if there is no YAML, load() will return an
                        # empty string.  We want an empty dictionary instead
                        # for our representation of empty.
                        if value is None:
                            value = []
                        value = self._replaceDoubleSlash(value)
                    except ruamel.yaml.MarkedYAMLError as e:
                        MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e), buttons=MessageBox.Ok)
                        item.setText(utf8(item.value))
                        return
                else:
                    value = item.text()
                self.parameterHandler.deliverParameter(self.masteruri, {item.name: value})
                item.value = value
            except ValueError as e:
                MessageBox.warning(self, "Warning",
                                   'Error while add changes to the ROS parameter server',
                                   utf8(e))
                item.setText(item.value)

    def _on_param_list(self, masteruri, code, msg, params):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The list the parameter names.
        :type params: list(str)
        '''
        if code == 1:
            params.sort()
            self.parameterHandler.requestParameterValues(masteruri, params)

    def _on_param_values(self, masteruri, code, msg, params):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary the parameter names and request result.
        :type params: dict(paramName : (code, statusMessage, parameterValue))
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
            self.parameter_model.update_model_data(result)
        else:
            rospy.logwarn("Error on retrieve parameter from %s: %s", utf8(masteruri), utf8(msg))

    def _on_delivered_values(self, masteruri, code, msg, params):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary the parameter names and request result.
        :type params: dict(paramName : (code, statusMessage, parameterValue))
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
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary the parameter names and request result.
        :type params: dict(paramName : (code, statusMessage, parameterValue))
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
#                         # TODO: you have to launch global parameter
#                         for _, launch_cfg in self.__configs.items():
#                             try:
#                                 launch_cfg.global_param_done.remove(masteruri)
#                             except ValueError:
#                                 pass
        else:
            rospy.logwarn("Error on retrieve sim parameter value from %s: %s", utf8(masteruri), utf8(msg))
        if not robot_icon_found:
            self.__current_parameter_robot_icon = ''
            self.update_robot_icon()

    def _get_nm_masteruri(self):
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it. The 'materuri' attribute will be set to the requested value.

        :return: ROS master URI
        :rtype: str or None
        '''
        if not hasattr(self, '_nm_materuri') or self._nm_materuri is None:
            masteruri = masteruri_from_ros()
            master = xmlrpcclient.ServerProxy(masteruri)
            _, _, self._nm_materuri = master.getUri(rospy.get_name())  # reuslt: code, message, self._nm_materuri
        return self._nm_materuri

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Nodelet handling                                 %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _on_question_cancel(self, questionid, data):
        pass

    def _on_question_ok(self, questionid, data):
        if questionid == MessageFrame.TYPE_NODELET:
            try:
                for cfgs, nodes in data.data.items():
                    self.stop_nodes_by_name(nodes)
                    self.start_nodes_by_name(nodes, cfgs, force=True, check_nodelets=False)
            except Exception as err:
                rospy.logwarn("Error while start nodelets: %s" % utf8(err))
        elif questionid == MessageFrame.TYPE_LAUNCH_FILE:
            try:
                self.launchfiles = data.data
            except Exception as err:
                rospy.logwarn("Error while reload launch file %s: %s" % (data.data, utf8(err)))
                MessageBox.warning(self, "Loading launch file", data.data, '%s' % utf8(err))
        elif questionid == MessageFrame.TYPE_TRANSFER:
            try:
                nmd_uri = nmdurl.nmduri(self.masteruri)
                username = self.current_user
                self.main_window.launch_dock.progress_queue.add2queue(utf8(uuid.uuid4()),
                                                                      'transfer %s to %s' % (data.data, nmd_uri),
                                                                      nm.starter().transfer_file_nmd,
                                                                      {'grpc_url': nmd_uri,
                                                                       'path': data.data,
                                                                       'auto_pw_request': True,
                                                                       'user': username
                                                                      })
                self.main_window.launch_dock.progress_queue.start()
            except Exception as err:
                rospy.logwarn("Error while transfer changed files %s: %s" % (data.data, utf8(err)))
                MessageBox.warning(self, "Loading launch file", data.data, '%s' % utf8(err))
        elif questionid == MessageFrame.TYPE_NMD:
            self.start_daemon()
        elif questionid == MessageFrame.TYPE_NMD_RESTART:
            # start node manager daemon if not already running
            rospy.loginfo("stop node manager daemon for %s", self.masteruri)
            self.stop_nodes_by_name(['node_manager_daemon'])
            self.start_daemon()
        elif questionid == MessageFrame.TYPE_BINARY:
            try:
                self.stop_nodes_by_name([node.name for node in data.data_list])
                for node in data.data_list:
                    if node.next_start_cfg:
                        self.start_node(node, force=True, config=node.next_start_cfg)
                    else:
                        self.start_nodes([node], force=True)
                    try:
                        del self._changed_binaries[node.name]
                    except KeyError:
                        pass
            except Exception as err:
                rospy.logwarn("Error while restart nodes %s: %s" % (data.data, utf8(err)))
                MessageBox.warning(self, "Restart nodes", data.data, '%s' % utf8(err))

        elif questionid == MessageFrame.TYPE_NODE_CFG:
            try:
                nodes, cfg = data.data
                self.stop_nodes_by_name(nodes)
                self.start_nodes_by_name(nodes, cfg, force=True)
            except Exception as err:
                rospy.logwarn("Error while restart nodes %s: %s" % (str(nodes), utf8(err)))
                MessageBox.warning(self, "Restart nodes", str(nodes), '%s' % utf8(err))

    def _on_info_ok(self, questionid, data):
        pass

    def start_daemon(self):
        try:
            # start node manager daemon if not already running
            host_addr = nm.nameres().address(self.masteruri)
            rospy.loginfo("start node manager daemon for %s", self.masteruri)
            self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                            'start node_manager_daemon for %s' % host_addr,
                                            nm.starter().runNodeWithoutConfig,
                                            {'host': host_addr,
                                            'package': 'fkie_node_manager_daemon',
                                            'binary': 'node_manager_daemon',
                                            'name': 'node_manager_daemon',
                                            'args': [],
                                            'masteruri': self.masteruri,
                                            'use_nmd': False,
                                            'auto_pw_request': False,
                                            'user': self.current_user
                                            })
            self._start_queue(self._progress_queue)
        except Exception as err:
            rospy.logwarn("Error while start node manager daemon on %s: %s" % (self.masteruri, utf8(err)))
            MessageBox.warning(self, "Start node manager daemon", self.masteruri, '%s' % utf8(err))


    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%   Shortcuts handling                               %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def focus_filter_line(self):
        if self._is_current_tab_name('tabNodes'):
            self.ui.nodeFilterInput.setFocus(Qt.ActiveWindowFocusReason)
        elif self._is_current_tab_name('tabTopics'):
            self.ui.topicFilterInput.setFocus(Qt.ActiveWindowFocusReason)
        elif self._is_current_tab_name('tabServices'):
            self.ui.serviceFilterInput.setFocus(Qt.ActiveWindowFocusReason)
        elif self._is_current_tab_name('tabParameter'):
            self.ui.parameterFilterInput.setFocus(Qt.ActiveWindowFocusReason)


    def select_host_block(self, index):
        '''
        Selects all nodes of a host with given index

        :param int index: the index of the host in the tree model
        '''
        root = self.ui.nodeTreeView.model().index(index, 0)
        if not root.isValid():
            return
        self.ui.nodeTreeView.expand(root)
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
        self.ui.nodeTreeView.selectionModel().select(selection, QItemSelectionModel.ClearAndSelect)

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
        self.ui.nodeTreeView.selectionModel().clearSelection()
        self.ui.nodeTreeView.collapseAll()

    def on_copy_c_pressed(self):
        result = ''
        if self.ui.nodeTreeView.hasFocus():
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                try:
                    result = '%s %s' % (result, node.name)
                except Exception:
                    pass
        elif self.ui.topicsView.hasFocus():
            selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
            for topic in selectedTopics:
                try:
                    result = '%s %s' % (result, topic.name)
                except Exception:
                    pass
        elif self.ui.servicesView.hasFocus():
            selectedServices = self.servicesFromIndexes(self.ui.servicesView.selectionModel().selectedIndexes())
            for service in selectedServices:
                try:
                    result = '%s %s' % (result, service.name)
                except Exception:
                    pass
        elif self.ui.parameterView.hasFocus():
            selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
            for (name, _value) in selectedParameter:
                try:
                    result = '%s %s' % (result, name)
                except Exception:
                    pass
        QApplication.clipboard().setText(result.strip())

    def on_copy_x_pressed(self):
        result = ''
        if self.ui.nodeTreeView.hasFocus():
            selectedNodes = self.nodesFromIndexes(self.ui.nodeTreeView.selectionModel().selectedIndexes())
            for node in selectedNodes:
                try:
                    result = '%s %s' % (result, node.pid)
                except Exception:
                    pass
        elif self.ui.topicsView.hasFocus():
            selectedTopics = self.topicsFromIndexes(self.ui.topicsView.selectionModel().selectedIndexes())
            for topic in selectedTopics:
                try:
                    result = '%s %s' % (result, topic.type)
                except Exception:
                    pass
        elif self.ui.servicesView.hasFocus():
            selectedServices = self.servicesFromIndexes(self.ui.servicesView.selectionModel().selectedIndexes())
            for service in selectedServices:
                try:
                    result = '%s %s' % (result, service.type)
                except Exception:
                    pass
        elif self.ui.parameterView.hasFocus():
            selectedParameter = self.parameterFromIndexes(self.ui.parameterView.selectionModel().selectedIndexes())
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
        result = True
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
        regex = self.filterRegExp()
        item = self.sourceModel().itemFromIndex(index0)
        if type(item) == TopicItem:
            result = (regex.indexIn(item.topic.name) != -1 or regex.indexIn(item.topic_type_str) != -1)
        elif type(item) == TopicGroupItem:
            result = True
            if regex.indexIn(item.name) != -1:
                result = True
            else:
                sitems = item.get_topic_items()
                for sitem in sitems:
                    result = (regex.indexIn(sitem.topic.name) != -1 or regex.indexIn(sitem.topic_type_str) != -1)
                    if result:
                        break
        return result


class ServicesSortFilterProxyModel(QSortFilterProxyModel):

    def filterAcceptsRow(self, sourceRow, sourceParent):
        '''
        Perform filtering on columns 0 and 1 (Name, Type)
        '''
        index0 = self.sourceModel().index(sourceRow, 0, sourceParent)
        regex = self.filterRegExp()
        item = self.sourceModel().itemFromIndex(index0)
        if type(item) == ServiceItem:
            return (regex.indexIn(item.service.name) != -1 or regex.indexIn(item.service_type_str) != -1)
        elif type(item) == ServiceGroupItem:
            if regex.indexIn(item.name) != -1:
                return True
            grp_res = True
            sitems = item.get_service_items()
            for sitem in sitems:
                res = (regex.indexIn(sitem.service.name) != -1 or regex.indexIn(sitem.service_type_str) != -1)
                if res:
                    return True
                grp_res = res
            return grp_res
        return True


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

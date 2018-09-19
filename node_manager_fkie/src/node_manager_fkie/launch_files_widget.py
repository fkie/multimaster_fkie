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
from python_qt_binding.QtGui import QColor, QKeySequence, QPalette

try:
    from python_qt_binding.QtGui import QSortFilterProxyModel
except:
    from python_qt_binding.QtCore import QSortFilterProxyModel
try:
    from python_qt_binding.QtGui import QAbstractItemView, QAction, QApplication, QDockWidget, QFileDialog, QLineEdit, QMenu, QWidget
except:
    from python_qt_binding.QtWidgets import QAbstractItemView, QAction, QApplication, QDockWidget, QFileDialog, QLineEdit, QMenu, QWidget

import os

import rospy

from node_manager_daemon_fkie.common import get_nmd_url, get_masteruri_from_nmd
from master_discovery_fkie.common import get_hostname
import node_manager_fkie as nm
from .common import package_name, utf8, grpc_join  # , masteruri_from_ros
from .detailed_msg_box import MessageBox
from .html_delegate import HTMLDelegate
from .launch_list_model import LaunchListModel, PathItem
#from .launch_tree_model import LaunchTreeModel, PathItem
from .parameter_dialog import ParameterDialog
from .progress_queue import ProgressQueue  # , ProgressThread


class LaunchFilesWidget(QDockWidget):
    '''
    Launch file browser.
    '''

    load_signal = Signal(str, dict, str)
    ''' load the launch file with given arguments (launchfile, args, masteruri)'''
    load_profile_signal = Signal(str)
    ''' load the profile file '''
    load_as_default_signal = Signal(str, str)
    ''' load the launch file as default (path, host) '''
    edit_signal = Signal(str)
    ''' list of paths to open in an editor '''
    transfer_signal = Signal(list)
    ''' list of tuples of (url, path) selected for transfer '''

    def __init__(self, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QDockWidget.__init__(self, parent)
        # initialize parameter
        self.__current_path = os.path.expanduser('~')
        # load the UI file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'LaunchFilesDockWidget.ui')
        loadUi(ui_file, self)
        self._current_search = ''
        pal = self.palette()
        self._default_color = pal.color(QPalette.Window)

        # initialize the progress queue
        self.progress_queue = ProgressQueue(self.ui_frame_progress_cfg, self.ui_bar_progress_cfg, self.ui_button_progress_cancel_cfg, 'Launch File')
        # initialize the view for the launch files
#         self.launchlist_model = LaunchTreeModel(self, progress_queue=self.progress_queue)
#         self.launchlist_proxy_model = FilesSortFilterProxyModel(self)
#         self.launchlist_proxy_model.setSourceModel(self.launchlist_model)
#         self.ui_file_view.setModel(self.launchlist_model)
#         for i, (_, width) in enumerate(LaunchTreeModel.header):  # _:=name
#             self.ui_file_view.setColumnWidth(i, width)
#         self.name_delegate = HTMLDelegate()
#         self.ui_file_view.setItemDelegateForColumn(0, self.name_delegate)
# #        self.node_delegate = IconsDelegate()
# #        self.masterTab.nodeTreeView.setItemDelegateForColumn(1, self.node_delegate)
#         #self.masterTab.nodeTreeView.collapsed.connect(self.on_node_collapsed)
#         #self.masterTab.nodeTreeView.expanded.connect(self.on_node_expanded)
#         sm = self.ui_file_view.selectionModel()
#         sm.selectionChanged.connect(self.on_launch_selection_changed)
#         self.ui_file_view.activated.connect(self.on_launch_selection_activated)
        #self.ui_file_view.clicked.connect(self.on_node_clicked)
        #####
        self.launchlist_model = LaunchListModel(progress_queue=self.progress_queue)
        self.launchlist_proxy_model = QSortFilterProxyModel(self)
        self.launchlist_proxy_model.setSourceModel(self.launchlist_model)
        self.name_delegate = HTMLDelegate(check_for_ros_names=False)
        self.ui_file_view.setItemDelegateForColumn(0, self.name_delegate)
        self.ui_file_view.setModel(self.launchlist_proxy_model)
        self.ui_file_view.setAlternatingRowColors(True)
        self.ui_file_view.activated.connect(self.on_launch_selection_activated)
        self.ui_file_view.setDragDropMode(QAbstractItemView.DragOnly)
        self.ui_file_view.setDragEnabled(True)
        sm = self.ui_file_view.selectionModel()
        sm.selectionChanged.connect(self.on_ui_file_view_selection_changed)
        self.launchlist_model.pathlist_handled.connect(self.on_pathlist_handled)
        self.launchlist_model.error_on_path.connect(self.on_error_on_path)

#    self.ui_search_line.setVisible(False)
        # self.ui_search_line.textChanged.connect(self.set_package_filter)
        self.ui_search_line.refresh_signal.connect(self.set_package_filter)
        self.ui_search_line.stop_signal.connect(self.stop)
        # self.ui_search_line.focusInEvent = self._searchline_focusInEvent
        # connect to the button signals
        # self.ui_button_refresh_path.clicked.connect(self.on_refresh_xml_clicked)
        self.ui_button_edit.clicked.connect(self.on_edit_xml_clicked)
        self.ui_button_new.clicked.connect(self.on_new_xml_clicked)
        self.ui_button_open_path.clicked.connect(self.on_open_xml_clicked)
        self.ui_button_transfer.clicked.connect(self.on_transfer_file_clicked)
        self.ui_button_load.clicked.connect(self.on_load_xml_clicked)
        self._masteruri2name = {}

    def stop(self):
        '''
        Cancel the executing queued actions. This method must be
        called at the exit!
        '''
        self.progress_queue.stop()
        self.ui_search_line.set_process_active(False)

    def set_current_master(self, masteruri, mastername):
        self.launchlist_model.set_current_master(masteruri, mastername)
        self._masteruri2name[masteruri.rstrip(os.path.sep)] = mastername

    def on_launch_selection_activated(self, activated):
        '''
        Tries to load the launch file, if one was activated.
        '''
        print "activated"
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        for item in selected:
            try:
                self.ui_search_line.set_process_active(True)
                lfile = self.launchlist_model.expand_item(item.path, item.id)
                print "after expand", lfile
                # self.ui_search_line.setText('')
                if lfile is not None:
                    self.ui_search_line.set_process_active(False)
                    if item.is_launch_file():
                        nm.settings().launch_history_add(item.path)
                        key_mod = QApplication.keyboardModifiers()
                        if key_mod & Qt.ShiftModifier:
                            self.load_as_default_signal.emit(item.path, None)
                        elif key_mod & Qt.ControlModifier:
                            self.launchlist_model.setPath(os.path.dirname(item.path))
                        else:
                            print "send load signal"
                            self.load_signal.emit(item.path, {}, None)
                    elif item.is_profile_file():
                        nm.settings().launch_history_add(item.path)
                        key_mod = QApplication.keyboardModifiers()
                        if key_mod & Qt.ControlModifier:
                            self.launchlist_model.setPath(os.path.dirname(item.path))
                        else:
                            self.load_profile_signal.emit(item.path)
                    elif item.is_config_file():
                        self.edit_signal.emit(lfile)
                if self.launchlist_model.current_path:
                    self.setWindowTitle('Launch @%s' % get_hostname(self.launchlist_model.current_grpc))
                else:
                    self.setWindowTitle('Launch files')
            except Exception as e:
                import traceback
                print traceback.format_exc()
                rospy.logwarn("Error while load launch file %s: %s" % (item, utf8(e)))
                MessageBox.warning(self, "Load error",
                                   'Error while load launch file:\n%s' % item.name,
                                   "%s" % utf8(e))
        try:
            print "CCCCCC", nm.nameres().masteruri2name(get_masteruri_from_nmd(self.launchlist_model.current_path)), get_masteruri_from_nmd(self.launchlist_model.current_path)
            color = QColor.fromRgb(nm.settings().host_color(self._masteruri2name[get_masteruri_from_nmd(self.launchlist_model.current_path)], self._default_color.rgb()))
            self._new_color(color)
        except Exception as err:
            import traceback
            print traceback.format_exc()
            rospy.logwarn("Error while set color in launch dock: %s" % utf8(err))

#        self.launchlist_model.reloadCurrentPath()
    def _new_color(self, color):
        bg_style_launch_dock = "QWidget#ui_dock_widget_contents { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 %s, stop: 0.7 %s);}" % (color.name(), self._default_color.name())
        self.setStyleSheet("%s" % (bg_style_launch_dock))

    def on_pathlist_handled(self, gpath):
        self.ui_search_line.set_process_active(False)

    def on_error_on_path(self, gpath):
        print "ERROR on_error_on_path", gpath, "c:", self.launchlist_model.current_path
        if gpath == self._current_search or gpath == self.launchlist_model.current_path:
            self.ui_search_line.set_process_active(False)

    def on_launch_selection_changed(self, selected, deselected):
        print "selection launch changed"

    def load_file(self, path, args={}, masteruri=None):
        '''
        Tries to load the launch file, if one was activated.
        '''
        if path is not None:
            if os.path.isfile(path):
                if path.endswith('.launch'):
                    self.load_signal.emit(path, args, masteruri)
                elif path.endswith('.nmprofile'):
                    self.load_profile_signal.emit(path)

    def on_ui_file_view_selection_changed(self, selected, deselected):
        '''
        On selection of a launch file, the buttons are enabled otherwise disabled.
        '''
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        for item in selected:
            islaunch = item.is_launch_file()
            isconfig = item.is_config_file()
            isprofile = item.is_profile_file()
            self.ui_button_edit.setEnabled(islaunch or isconfig or isprofile)
            self.ui_button_load.setEnabled(islaunch or isprofile)
            self.ui_button_transfer.setEnabled(islaunch or isconfig)

    def set_package_filter(self, text):
        if text.startswith('s '):
            if len(text) > 2:
                search_text = text[2:]
                self.launchlist_proxy_model.setFilterRegExp(QRegExp(search_text, Qt.CaseInsensitive, QRegExp.Wildcard))
                import glob
                print glob.glob1(self.launchlist_model.current_path, text)
            else:
                self.ui_search_line.set_process_active(False)
        else:
            print "new path:", text
            if text:
                if text.startswith(os.path.sep):
                    self._current_search = grpc_join(self.launchlist_model.current_grpc, text)
                    print "SET PATH", self._current_search
                    self.launchlist_model.set_path(text)
                else:
                    # search for a package
                    self.launchlist_model.show_packages(text)
                    self.ui_search_line.set_process_active(False)
            else:
                self.launchlist_model.reload_current_path(clear_cache=True)

    def on_refresh_xml_clicked(self):
        '''
        Reload the current path.
        '''
        self.launchlist_model.reloadCurrentPath()
        print "UPDATE PACKAGES LOCAL", get_nmd_url()
        nm.nmd().list_packages_threaded(get_nmd_url(), True)
        # TODO: self.launchlist_model.reload_packages()
        self.ui_button_edit.setEnabled(False)
        self.ui_button_load.setEnabled(False)
        self.ui_button_transfer.setEnabled(False)

    def on_edit_xml_clicked(self):
        '''
        Opens an XML editor to edit the launch file.
        '''
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        for item in selected:
            path = self.launchlist_model.expand_item(item.path, item.id)
            if path is not None:
                self.edit_signal.emit(path)

    def on_new_xml_clicked(self):
        '''
        Creates a new launch file.
        '''
        # get new file from open dialog, use last path if one exists
        open_path = self.__current_path
        if self.launchlist_model.currentPath is not None:
            open_path = self.launchlist_model.currentPath
        (fileName, _) = QFileDialog.getSaveFileName(self,
                                                    "New launch file",
                                                    open_path,
                                                    "Config files (*.launch *.yaml);;All files (*)")
        if fileName:
            try:
                (pkg, _) = package_name(os.path.dirname(fileName))  # _:=pkg_path
                if pkg is None:
                    MessageBox.warning(self, "New File Error", 'The new file is not in a ROS package')
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
                self.__current_path = os.path.dirname(fileName)
                self.launchlist_model.setPath(self.__current_path)
                self.edit_signal.emit(fileName)
            except EnvironmentError as e:
                MessageBox.warning(self, "New File Error",
                                   'Error while create a new file',
                                   '%s' % utf8(e))

    def on_open_xml_clicked(self):
        (fileName, _) = QFileDialog.getOpenFileName(self,
                                                    "Load launch file",
                                                    self.__current_path,
                                                    "Config files (*.launch);;All files (*)")
        if fileName:
            self.__current_path = os.path.dirname(fileName)
            nm.settings().launch_history_add(fileName)
            self.load_signal.emit(fileName, {}, None)

    def on_transfer_file_clicked(self):
        '''
        Emit the signal to copy the selected file to a remote host.
        '''
        # TODO
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        paths = list()
        for item in selected:
            path = self.launchlist_model.expand_item(item.path, item.id)
            if path is not None:
                paths.append(path)
        if paths:
            self.transfer_signal.emit(paths)

    def on_load_xml_clicked(self):
        '''
        Tries to load the selected launch file. The button is only enabled and this
        method is called, if the button was enabled by on_launch_selection_clicked()
        '''
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        for item in selected:
            path = self.launchlist_model.expand_item(item.path, item.id)
            if path is not None:
                nm.settings().launch_history_add(item.path)
                self.load_signal.emit(path, {}, None)

    def on_load_as_default_at_host(self):
        '''
        Tries to load the selected launch file as default configuration. The button
        is only enabled and this method is called, if the button was enabled by
        on_launch_selection_clicked()
        '''
        selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
        for item in selected:
            path = self.launchlist_model.expandItem(item.name, item.path, item.id)
            if path is not None:
                params = {'Host': ('string', 'localhost')}
                dia = ParameterDialog(params)
                dia.setFilterVisible(False)
                dia.setWindowTitle('Start node on...')
                dia.resize(350, 120)
                dia.setFocusField('Host')
                if dia.exec_():
                    try:
                        params = dia.getKeywords()
                        host = params['Host']
                        rospy.loginfo("LOAD the launch file on host %s as default: %s" % (host, path))
                        nm.settings().launch_history_add(path)
                        self.load_as_default_signal.emit(path, host)
                    except Exception, e:
                        MessageBox.warning(self, "Load default config error",
                                           'Error while parse parameter',
                                           '%s' % utf8(e))

#     def on_load_as_default(self):
#         '''
#         Tries to load the selected launch file as default configuration. The button
#         is only enabled and this method is called, if the button was enabled by
#         on_launch_selection_clicked()
#         '''
#         key_mod = QApplication.keyboardModifiers()
#         if key_mod & Qt.ShiftModifier:
#             self.loadXmlAsDefaultButton.showMenu()
#         else:
#             selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
#             for item in selected:
#                 path = self.launchlist_model.expandItem(item.name, item.path, item.id)
#                 if path is not None:
#                     rospy.loginfo("LOAD the launch file as default: %s", path)
#                     nm.settings().launch_history_add(path)
#                     self.load_as_default_signal.emit(path, None)

    def _pathItemsFromIndexes(self, indexes, recursive=True):
        result = []
        for index in indexes:
            if index.column() == 0:
                model_index = self.launchlist_proxy_model.mapToSource(index)
                item = self.launchlist_model.itemFromIndex(model_index)
                if item is not None and isinstance(item, PathItem):
                    result.append(item)
        return result

    def keyPressEvent(self, event):
        '''
        Defines some of shortcuts for navigation/management in launch
        list view or topics view.
        '''
        key_mod = QApplication.keyboardModifiers()
        if not self.ui_file_view.state() == QAbstractItemView.EditingState:
            # remove history file from list by pressing DEL
            if event == QKeySequence.Delete:
                selected = self._pathItemsFromIndexes(self.ui_file_view.selectionModel().selectedIndexes(), False)
                for item in selected:
                    nm.settings().launch_history_remove(item.path)
                    self.launchlist_model.reloadCurrentPath()
            elif not key_mod and event.key() == Qt.Key_F4 and self.ui_button_edit.isEnabled():
                # open selected launch file in xml editor by F4
                self.on_edit_xml_clicked()
            elif event == QKeySequence.Find:
                # set focus to filter box for packages
                self.ui_search_line.setFocus(Qt.ActiveWindowFocusReason)
            elif event == QKeySequence.Paste:
                # paste files from clipboard
                self.launchlist_model.paste_from_clipboard()
            elif event == QKeySequence.Copy:
                # copy the selected items as file paths into clipboard
                selected = self.ui_file_view.selectionModel().selectedIndexes()
                indexes = []
                for s in selected:
                    indexes.append(self.launchlist_proxy_model.mapToSource(s))
                self.launchlist_model.copy_to_clipboard(indexes)
        if self.ui_search_line.hasFocus() and event.key() == Qt.Key_Escape:
            # cancel package filtering on pressing ESC
            self.launchlist_model.reload_current_path()
            self.ui_search_line.setText('')
            self.ui_file_view.setFocus(Qt.ActiveWindowFocusReason)
        QDockWidget.keyReleaseEvent(self, event)

    def _searchline_focusInEvent(self, event):
        self.launchlist_model.show_packages(True)
        QLineEdit.focusInEvent(self.ui_search_line, event)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%   Filter handling                               %%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


class FilesSortFilterProxyModel(QSortFilterProxyModel):

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

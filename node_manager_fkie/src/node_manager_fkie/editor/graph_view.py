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
from python_qt_binding.QtCore import QObject, Signal, Qt
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
import os
import threading
import rospy

from node_manager_fkie.common import package_name
from node_manager_fkie.html_delegate import HTMLDelegate
from node_manager_fkie.launch_config import LaunchConfig

try:
    from python_qt_binding.QtGui import QCheckBox, QFrame, QLabel, QTreeWidget, QTreeWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtGui import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy, QAbstractItemView, QItemSelectionModel
except:
    from python_qt_binding.QtWidgets import QCheckBox, QFrame, QLabel, QTreeWidget, QTreeWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy, QAbstractItemView
    from python_qt_binding.QtCore import QItemSelectionModel


GRAPH_CACHE = {}
CHACHE_MUTEX = threading.RLock()


class GraphViewWidget(QDockWidget):
    '''
    A frame to find text in the Editor.
    '''
    load_signal = Signal(str, bool)
    ''' @ivar: filename of file to load, True if insert after the current open tab'''
    goto_signal = Signal(str, int)
    ''' @ivar: filename, line to go'''
    DATA_FILE = Qt.UserRole + 1
    DATA_LINE = Qt.UserRole + 2
    DATA_INC_FILE = Qt.UserRole + 3
    DATA_LEVEL = Qt.UserRole + 4

    def __init__(self, tabwidget, parent=None):
        QDockWidget.__init__(self, "LaunchGraph", parent)
        graph_ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'GraphDockWidget.ui')
        loadUi(graph_ui_file, self)
        self.setObjectName('LaunchGraph')
        self.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self._tabwidget = tabwidget
        self._current_path = None
        self._root_path = None
        self._current_deep = 0
        self.graphTreeView.setSelectionBehavior(QAbstractItemView.SelectRows)
        model = QStandardItemModel()
        self.graphTreeView.setModel(model)
        self.graphTreeView.setUniformRowHeights(True)
        self.graphTreeView.header().hide()
        self.htmlDelegate = HTMLDelegate()
        self.graphTreeView.setItemDelegateForColumn(0, self.htmlDelegate)
        self.graphTreeView.activated.connect(self.on_activated)
        self.graphTreeView.clicked.connect(self.on_clicked)
        self._created_tree = False
        self._refill_tree([], [], False)

    def clear_cache(self):
        with CHACHE_MUTEX:
            GRAPH_CACHE.clear()
            self._created_tree = False
            self.graphTreeView.model().clear()
            crp = self._current_path
            self._current_path = None
            self.set_file(crp, self._root_path)

    def set_file(self, current_path, root_path):
        self._root_path = root_path
        if self._current_path != current_path:
            self._current_path = current_path
            # TODO: run analyzer/path parser in a new thread
            self.setWindowTitle("Include Graph - loading...")
            self._fill_graph_thread = GraphThread(current_path, root_path)
            self._fill_graph_thread.graph.connect(self._refill_tree)
            self._fill_graph_thread.start()

    def find_parent_file(self):
        selected = self.graphTreeView.selectionModel().selectedIndexes()
        for index in selected:
            item = self.graphTreeView.model().itemFromIndex(index.parent())
            self.load_signal.emit(item.data(self.DATA_INC_FILE), self._current_deep < item.data(self.DATA_LEVEL))

    def on_activated(self, index):
        item = self.graphTreeView.model().itemFromIndex(index)
        self.load_signal.emit(item.data(self.DATA_INC_FILE), self._current_deep < item.data(self.DATA_LEVEL))

    def on_clicked(self, index):
        item = self.graphTreeView.model().itemFromIndex(index)
        self.goto_signal.emit(item.data(self.DATA_FILE), item.data(self.DATA_LINE))

    def enable(self):
        self.setVisible(True)
        self.raise_()
        self.activateWindow()
        self.graphTreeView.setFocus()

    def _refill_tree(self, included_from, includes, create_tree=True):
        deep = 0
        file_dsrc = self._root_path
        try:
            file_dsrc = os.path.basename(self._root_path)
        except Exception:
            pass
        self.setWindowTitle("Include Graph - %s" % file_dsrc)
        if not self._created_tree and create_tree:
            with CHACHE_MUTEX:
                if self._root_path in GRAPH_CACHE:
                    pkg, _ = package_name(os.path.dirname(self._root_path))
                    itemstr = '%s [%s]' % (os.path.basename(self._root_path), pkg)
                    inc_item = QStandardItem('%s' % itemstr)
                    inc_item.setData(self._root_path, self.DATA_FILE)
                    inc_item.setData(-1, self.DATA_LINE)
                    inc_item.setData(self._root_path, self.DATA_INC_FILE)
                    inc_item.setData(deep, self.DATA_LEVEL)
                    self._append_items(inc_item, deep + 1)
                    self.graphTreeView.model().appendRow(inc_item)
                    # self.graphTreeView.expand(self.graphTreeView.model().indexFromItem(inc_item))
                self._created_tree = True
        items = self.graphTreeView.model().match(self.graphTreeView.model().index(0, 0), self.DATA_INC_FILE, self._current_path, 10, Qt.MatchRecursive)
        first = True
        self.graphTreeView.selectionModel().clearSelection()
        for item in items:
            if first:
                self._current_deep = item.data(self.DATA_LEVEL)
                self.graphTreeView.selectionModel().select(item, QItemSelectionModel.SelectCurrent)
                first = False
            else:
                self.graphTreeView.selectionModel().select(item, QItemSelectionModel.Select)
        self.graphTreeView.expandAll()

    def _append_items(self, item, deep):
        path = item.data(self.DATA_INC_FILE)
        if not path:
            path = item.data(self.DATA_FILE)
        if path in GRAPH_CACHE:
            for inc_lnr, inc_path in GRAPH_CACHE[path]:
                pkg, _ = package_name(os.path.dirname(inc_path))
                itemstr = '%s [%s]' % (os.path.basename(inc_path), pkg)
                inc_item = QStandardItem('%d: %s' % (inc_lnr + 1, itemstr))
                inc_item.setData(path, self.DATA_FILE)
                inc_item.setData(inc_lnr + 1, self.DATA_LINE)
                inc_item.setData(inc_path, self.DATA_INC_FILE)
                inc_item.setData(deep, self.DATA_LEVEL)
                self._append_items(inc_item, deep + 1)
                item.appendRow(inc_item)


class GraphThread(QObject, threading.Thread):
    '''
    A thread to parse file for includes
    '''
    graph = Signal(list, list)
    '''
    @ivar: graph is a signal, which emit two list for files include the current path and a list with included files.
    Each entry is a tuple of the line number and path.
    '''

    def __init__(self, current_path, root_path):
        '''
        :param root_path: the open root file
        :type root_path: str
        :param current_path: current shown file
        :type current_path: str
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.current_path = current_path
        self.root_path = root_path

    def run(self):
        '''
        '''
        try:
            includes = self._get_includes(self.current_path)
            included_from = []
            incs = self._get_includes(self.root_path)
            included_from = self._find_inc_file(self.current_path, incs, self.root_path)
            self.graph.emit(included_from, includes)
        except Exception:
            import traceback
            formatted_lines = traceback.format_exc(1).splitlines()
            print "Error while parse launch file for includes:\n\t%s" % traceback.format_exc()
            try:
                rospy.logwarn("Error while parse launch file for includes:\n\t%s", formatted_lines[-1])
            except Exception:
                pass

    def _get_includes(self, path):
        result = []
        with CHACHE_MUTEX:
            if path:
                if path in GRAPH_CACHE:
                    result = GRAPH_CACHE[path]
                else:
                    result = LaunchConfig.included_files(path, recursive=False, unique=False)
                    GRAPH_CACHE[path] = result
        return result

    def _find_inc_file(self, filename, files, root_path):
        result = []
        for f in files:
            if filename == f[1]:
                result.append((f[0], root_path))
            else:
                inc_files = self._get_includes(f[1])
                result += self._find_inc_file(filename, inc_files, f[1])
        return result

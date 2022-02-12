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



from python_qt_binding.QtCore import Signal, Qt
import os
import rospy

from fkie_node_manager.common import package_name

from .line_edit import EnhancedLineEdit
from .text_search_thread import TextSearchThread


try:
    from python_qt_binding.QtGui import QCheckBox, QFrame, QLabel, QTreeWidget, QTreeWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtGui import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy
except:
    from python_qt_binding.QtWidgets import QCheckBox, QFrame, QLabel, QTreeWidget, QTreeWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy


class TextSearchFrame(QDockWidget):
    '''
    A frame to find text in the Editor.
    '''
    found_signal = Signal(str, bool, str, int, int, int, str)
    ''' @ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, first position in text, last position in text, linenr, line_text)
        for each result a signal will be emitted.
    '''
    replace_signal = Signal(str, str, int, str)
    ''' @ivar: A signal emitted to replace string at given position.
        (search text, file, position in text, replaced by text)
    '''

    def __init__(self, tabwidget, parent=None):
        QDockWidget.__init__(self, "Find", parent)
        self.setObjectName('SearchFrame')
        self.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self._dockwidget = QFrame(self)
        self.vbox_layout = QVBoxLayout(self._dockwidget)
        self.layout().setContentsMargins(0, 0, 0, 0)
        self.layout().setSpacing(1)
        # frame with two rows for find and replace
        find_replace_frame = QFrame(self)
        find_replace_vbox_layout = QVBoxLayout(find_replace_frame)
        find_replace_vbox_layout.setContentsMargins(0, 0, 0, 0)
        find_replace_vbox_layout.setSpacing(1)
#        find_replace_vbox_layout.addSpacerItem(QSpacerItem(1, 1, QSizePolicy.Expanding, QSizePolicy.Expanding))
        # create frame with find row
        self.find_frame = self._create_find_frame()
        find_replace_vbox_layout.addWidget(self.find_frame)
        rplc_frame = self._create_replace_frame()
        find_replace_vbox_layout.addWidget(rplc_frame)
        # frame for find&replace and search results
        self.vbox_layout.addWidget(find_replace_frame)
        self.vbox_layout.addWidget(self._create_found_frame())
#        self.vbox_layout.addStretch(2024)
        self.setWidget(self._dockwidget)
        # intern search parameters
        self._tabwidget = tabwidget
        self.current_search_text = ''
        self.search_results = []
        self.search_results_fileset = set()
        self._search_result_index = -1
        self._search_recursive = False
        self._search_thread = None

    def _create_find_frame(self):
        find_frame = QFrame(self)
        find_hbox_layout = QHBoxLayout(find_frame)
        find_hbox_layout.setContentsMargins(0, 0, 0, 0)
        find_hbox_layout.setSpacing(1)
        self.search_field = EnhancedLineEdit(find_frame)
        self.search_field.setPlaceholderText('search text')
        self.search_field.textChanged.connect(self.on_search_text_changed)
        self.search_field.returnPressed.connect(self.on_search)
        find_hbox_layout.addWidget(self.search_field)
        self.search_result_label = QLabel(find_frame)
        self.search_result_label.setText(' ')
        find_hbox_layout.addWidget(self.search_result_label)
        self.find_button_back = QPushButton("<")
        self.find_button_back.setFixedWidth(44)
        self.find_button_back.clicked.connect(self.on_search_back)
        find_hbox_layout.addWidget(self.find_button_back)
        self.find_button = QPushButton(">")
        self.find_button.setDefault(True)
        # self.find_button.setFlat(True)
        self.find_button.setFixedWidth(44)
        self.find_button.clicked.connect(self.on_search)
        find_hbox_layout.addWidget(self.find_button)
        return find_frame

    def _create_replace_frame(self):
        # create frame with replace row
        self.rplc_frame = rplc_frame = QFrame(self)
        rplc_hbox_layout = QHBoxLayout(rplc_frame)
        rplc_hbox_layout.setContentsMargins(0, 0, 0, 0)
        rplc_hbox_layout.setSpacing(1)
        self.replace_field = EnhancedLineEdit(rplc_frame)
        self.replace_field.setPlaceholderText('replace text')
        self.replace_field.returnPressed.connect(self.on_replace)
        rplc_hbox_layout.addWidget(self.replace_field)
        self.replace_result_label = QLabel(rplc_frame)
        self.replace_result_label.setText(' ')
        rplc_hbox_layout.addWidget(self.replace_result_label)
        self.replace_button = replace_button = QPushButton("> &Replace >")
        replace_button.setFixedWidth(90)
        replace_button.clicked.connect(self.on_replace_click)
        rplc_hbox_layout.addWidget(replace_button)
        rplc_frame.setVisible(False)
        return rplc_frame

    def _create_found_frame(self):
        ff_frame = QFrame(self)
        self.found_files_vbox_layout = QVBoxLayout(ff_frame)
        self.found_files_vbox_layout.setContentsMargins(0, 0, 0, 0)
        self.recursive_search_box = QCheckBox("recursive search")
        self.found_files_vbox_layout.addWidget(self.recursive_search_box)
        self.found_files_list = QTreeWidget(ff_frame)
        self.found_files_list.setColumnCount(1)
        self.found_files_list.setFrameStyle(QFrame.StyledPanel)
        self.found_files_list.setHeaderHidden(True)
        self.found_files_list.itemActivated.connect(self.on_itemActivated)
        self.found_files_list.itemClicked.connect(self.on_itemActivated)
        self.found_files_list.setStyleSheet(
            "QTreeWidget {"
            "background-color:transparent;"
            "}"
            "QTreeWidget::item {"
            "background-color:transparent;"
            "}"
            "QTreeWidget::item:selected {"
            "background-color: darkgray;"
            "}")
        self.found_files_vbox_layout.addWidget(self.found_files_list)
        self.recursive_search_box.setChecked(False)
        return ff_frame

    def keyPressEvent(self, event):
        '''
        Enable the shortcats for search and replace
        '''
        self.parent().keyPressEvent(event)

    def on_search(self):
        '''
        Initiate the new search or request a next search result.
        '''
        if self.current_search_text != self.search_field.text() or self._search_recursive != self.recursive_search_box.isChecked():
            # clear current search results
            self._reset()
            self.current_search_text = self.search_field.text()
            if self.current_search_text:
                path_text = {}
                self._wait_for_result = True
                for i in range(self._tabwidget.count()):
                    path_text[self._tabwidget.widget(i).filename] = self._tabwidget.widget(i).document().toPlainText()
                self._search_recursive = self.recursive_search_box.isChecked()
                self._search_thread = TextSearchThread(self.current_search_text, self._tabwidget.currentWidget().filename, path_text=path_text, recursive=self._search_recursive)
                self._search_thread.search_result_signal.connect(self.on_search_result)
                self._search_thread.warning_signal.connect(self.on_warning_result)
                self._search_thread.start()
        elif self.search_results:
            self._check_position()
            if self.search_results:
                if self._search_result_index + 1 >= len(self.search_results):
                    self._search_result_index = -1
                self._search_result_index += 1
                (_id, search_text, found, path, startpos, endpos, linenr, line) = self.search_results[self._search_result_index]
                self.found_signal.emit(search_text, found, path, startpos, endpos, linenr, line)
                self.replace_button.setEnabled(True)
        self._update_label()

    def on_search_back(self):
        '''
        Slot to handle the search back function.
        '''
        self._check_position(False)
        if self.search_results:
            self._search_result_index -= 1
            if self._search_result_index < 0:
                self._search_result_index = len(self.search_results) - 1
            self._update_label()
            (_id, search_text, found, path, startpos, endpos, linenr, line) = self.search_results[self._search_result_index]
            self.found_signal.emit(search_text, found, path, startpos, endpos, linenr, line)
            self.replace_button.setEnabled(True)

    def _check_position(self, forward=True):
        try:
            # if the position of the textCursor was changed by the user, move the search index
            cur_pos = self._tabwidget.currentWidget().textCursor().position()
            _id, st, _f, pa, idx, _lnr, _ltxt = self.search_results[self._search_result_index]
            sear_pos = idx + len(st)
            if cur_pos != sear_pos:
                first_idx = self._get_current_index_for_current_file()
                if first_idx != -1:
                    _id, st, _f, pa, idx, _lnr, _ltxt = self.search_results[first_idx]
                    sear_pos = idx + len(st)
                    while cur_pos > sear_pos and self._tabwidget.currentWidget().filename == pa:
                        first_idx += 1
                        _id, st, _f, pa, idx, _lnr, _ltxt = self.search_results[first_idx]
                        sear_pos = idx + len(st)
                    self._search_result_index = first_idx
                    if forward:
                        self._search_result_index -= 1
                else:
                    self._reset(True)
        except Exception:
            pass

    def _get_current_index_for_current_file(self):
        for index in range(len(self.search_results)):
            _id, _st, _f, pa, _idx = self.search_results[index]
            if self._tabwidget.currentWidget().filename == pa:
                return index
        return -1

    def on_search_result(self, search_text, found, path, startpos, endpos, linenr, line):
        '''
        Slot to handle the signals for search result. This signals are forwarded used
        found_signal.
        '''
        if found and search_text == self.current_search_text:
            id_path = "%d|%s" % (startpos, path)
            self.search_results_fileset.add(path)
            item = (search_text, found, path, startpos)
            if item not in self.search_results:
                self.search_results.append((id_path, search_text, found, path, startpos, endpos, linenr, line))
            if hasattr(self, '_wait_for_result') and self._wait_for_result:
                self._search_result_index += 1
                if startpos >= self._tabwidget.currentWidget().textCursor().position() or self._tabwidget.currentWidget().filename != path:
                    self._wait_for_result = False
                    self.found_signal.emit(search_text, found, path, startpos, endpos, linenr, line)
                    self.replace_button.setEnabled(True)
            pkg, _rpath = package_name(os.path.dirname(path))
            itemstr = '%s [%s]' % (os.path.basename(path), pkg)
            if not self.found_files_list.findItems(itemstr, Qt.MatchExactly):
                list_item = QTreeWidgetItem(self.found_files_list)
                list_item.setText(0, itemstr)
                list_item.setToolTip(0, path)
                self.found_files_list.insertTopLevelItem(0, list_item)
                self.found_files_list.expandAll()
            for i in range(self.found_files_list.topLevelItemCount()):
                top_item = self.found_files_list.topLevelItem(i)
                if top_item.text(0) == itemstr:
                    sub_item_str = "%d: %s" % (linenr, line)
                    list_item2 = QTreeWidgetItem()
                    list_item2.setText(0, sub_item_str)
                    list_item2.setWhatsThis(0, id_path)
                    top_item.addChild(list_item2)
        self._update_label()

    def on_warning_result(self, text):
        rospy.logwarn(text)

    def on_replace_click(self):
        self.on_replace()
        self.on_search()

    def on_replace(self):
        '''
        Emits the replace signal, but only if currently selected text is equal to the searched one.
        '''
        if self.search_results:
            try:
                _id, search_text, _found, path, index, _endidx, _linenr, _line_text = self.search_results[self._search_result_index]
                cursor = self._tabwidget.currentWidget().textCursor()
                if cursor.selectedText() == search_text:
                    rptxt = self.replace_field.text()
                    for rindex in range(self._search_result_index + 1, len(self.search_results)):
                        iid, st, _f, pa, idx, endidx, lnr, ltxt = self.search_results[rindex]
                        if path == pa:
                            self.search_results.pop(rindex)
                            shift = len(rptxt) - len(st)
                            self.search_results.insert(rindex, (iid, st, _f, pa, idx + shift, endidx + shift, lnr, ltxt))
                        else:
                            break
                    self._remove_search_result(self._search_result_index)
                    self._search_result_index -= 1
                    self.replace_signal.emit(search_text, path, index, rptxt)
                else:
                    self.replace_button.setEnabled(False)
            except Exception:
                import traceback
                print(traceback.format_exc())

    def on_itemActivated(self, item):
        '''
        Go to the results for the selected file entry in the list.
        '''
        splits = item.whatsThis(0).split('|')
        if len(splits) == 2:
            item_index = int(splits[0])
            item_path = splits[1]
            new_search_index = -1
            for _id, search_text, found, path, startpos, endpos, linenr, line_text in self.search_results:
                new_search_index += 1
                if item_path == path and item_index == startpos:
                    self._search_result_index = new_search_index
                    self.found_signal.emit(search_text, found, path, startpos, endpos, linenr, line_text)
                    self._update_label()

    def on_search_text_changed(self, _text):
        '''
        Clear search result if the text was changed.
        '''
        self._reset()

    def _update_label(self, clear_label=False):
        '''
        Updates the status label for search results. The info is created from search result lists.
        '''
        msg = ' '
        if self.search_results:
            count_files = len(self.search_results_fileset)
            msg = '%d/%d' % (self._search_result_index + 1, len(self.search_results))
            if count_files > 1:
                msg = '%s(%d)' % (msg, count_files)
        if self._search_thread is not None and self._search_thread.is_alive():
            msg = 'searching..%s' % msg
        elif not msg.strip() and self.current_search_text:
            msg = '0 found'
            self.current_search_text = ''
        if clear_label:
            msg = ' '
        self.search_result_label.setText(msg)
        self.find_button_back.setEnabled(len(self.search_results))
        self._select_current_item_in_box(self._search_result_index)

    def file_changed(self, path):
        '''
        Clears search results if for changed file are some search results are available
        :param path: changed file path
        :type path: str
        '''
        if path in self.search_results_fileset:
            self._reset()

    def set_replace_visible(self, value):
        self.rplc_frame.setVisible(value)
        self.raise_()
        self.activateWindow()
        if value:
            self.replace_field.setFocus()
            self.replace_field.selectAll()
            self.setWindowTitle("Find / Replace")
        else:
            self.setWindowTitle("Find")
            self.search_field.setFocus()

    def is_replace_visible(self):
        return self.rplc_frame.isVisible()

    def _reset(self, force_new_search=False):
        # clear current search results
        if self._search_thread is not None:
            self._search_thread.search_result_signal.disconnect()
            self._search_thread.stop()
            self._search_thread = None
        self.current_search_text = ''
        self.search_results = []
        self.search_results_fileset = set()
        self.found_files_list.clear()
#        self.found_files_list.setVisible(False)
        self._update_label(True)
        self._search_result_index = -1
        self.find_button_back.setEnabled(False)
        if force_new_search:
            self.on_search()

    def enable(self):
        self.setVisible(True)
#        self.show()
        self.raise_()
        self.activateWindow()
        self.search_field.setFocus()
        self.search_field.selectAll()

    def _select_current_item_in_box(self, index):
        try:
            (id_path, _search_text, _found, _path, index, _endidx, _linenr, _line) = self.search_results[index]
            for topidx in range(self.found_files_list.topLevelItemCount()):
                topitem = self.found_files_list.topLevelItem(topidx)
                for childdx in range(topitem.childCount()):
                    child = topitem.child(childdx)
                    if child.whatsThis(0) == id_path:
                        child.setSelected(True)
                    elif child.isSelected():
                        child.setSelected(False)
        except Exception:
            pass

    def _remove_search_result(self, index):
        try:
            (id_path, _search_text, _found, path, index, _endidx, _linenr, _line) = self.search_results.pop(index)
            pkg, _rpath = package_name(os.path.dirname(path))
            itemstr = '%s [%s]' % (os.path.basename(path), pkg)
            found_items = self.found_files_list.findItems(itemstr, Qt.MatchExactly)
            for item in found_items:
                for chi in range(item.childCount()):
                    child = item.child(chi)
                    if child.whatsThis(0) == id_path:
                        item.removeChild(child)
                        break
            # delete top level item if it is now empty
            for topidx in range(self.found_files_list.topLevelItemCount()):
                if self.found_files_list.topLevelItem(topidx).childCount() == 0:
                    self.found_files_list.takeTopLevelItem(topidx)
                    break
            # create new set with files contain the search text
            new_path_set = set(path for _id, _st, _fd, path, _idx, _endidx, _lnr, _lntxt in self.search_results)
            self.search_results_fileset = new_path_set
#            self.found_files_list.setVisible(len(self.search_results_fileset) > 0)
        except Exception:
            import traceback
            print(traceback.format_exc())

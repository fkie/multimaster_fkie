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

from node_manager_fkie.common import package_name

from .line_edit import EnchancedLineEdit
from .text_search_thread import TextSearchThread

try:
    from python_qt_binding.QtGui import QFrame, QLabel, QListWidget, QListWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtGui import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy
except:
    from python_qt_binding.QtWidgets import QFrame, QLabel, QListWidget, QListWidgetItem, QPushButton, QGroupBox, QDockWidget
    from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy


class TextSearchFrame(QDockWidget):
    '''
    A frame to find text in the Editor.
    '''
    search_result_signal = Signal(str, bool, str, int)
    ''' @ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, position in text)
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
        find_replace_vbox_layout.addSpacerItem(QSpacerItem(1, 1, QSizePolicy.Expanding, QSizePolicy.Expanding))
        # create frame with find row
        find_frame = self._create_find_frame()
        find_replace_vbox_layout.addWidget(find_frame)
        rplc_frame = self._create_replace_frame()
        find_replace_vbox_layout.addWidget(rplc_frame)
        # frame for find&replace and search results
        self.vbox_layout.addWidget(find_replace_frame)
        self.vbox_layout.addWidget(self._create_found_frame())
        self.vbox_layout.addStretch(2024)
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
        self.search_field = EnchancedLineEdit(find_frame)
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
        self.replace_field = EnchancedLineEdit(rplc_frame)
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
        self.found_files_frame = ff_frame = QGroupBox("recursive search")
        ff_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.found_files_vbox_layout = QVBoxLayout(ff_frame)
        self.found_files_vbox_layout.setSpacing(0)
        self.found_files_vbox_layout.setContentsMargins(0, 0, 0, 0)
        self.found_files_list = QListWidget(ff_frame)
        self.found_files_list.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.found_files_list.setFrameStyle(QFrame.StyledPanel)
        self.found_files_list.itemActivated.connect(self.on_itemActivated)
        self.found_files_list.setStyleSheet(
            "QListWidget {"
            "background-color:transparent;"
            "}"
            "QListWidget::item {"
            "background-color:transparent;"
            "}"
            "QListWidget::item:selected {"
            "background-color: darkgray;"
            "}")
        self.found_files_vbox_layout.addWidget(self.found_files_list)
        ff_frame.setCheckable(True)
        ff_frame.setChecked(False)
        ff_frame.setFlat(True)
        self.found_files_list.setVisible(False)
        return self.found_files_frame

    def keyPressEvent(self, event):
        '''
        Enable the shortcats for search and replace
        '''
        self.parent().keyPressEvent(event)

    def on_search(self):
        '''
        Initiate the new search or request a next search result.
        '''
        if self.current_search_text != self.search_field.text() or self._search_recursive != self.found_files_frame.isChecked():
            # clear current search results
            self._reset()
            self.current_search_text = self.search_field.text()
            if self.current_search_text:
                path_text = {}
                self._wait_for_result = True
                for i in range(self._tabwidget.count()):
                    path_text[self._tabwidget.widget(i).filename] = self._tabwidget.widget(i).document().toPlainText()
                self._search_recursive = self.found_files_frame.isChecked()
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
                self.search_result_signal.emit(*self.search_results[self._search_result_index])
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
            self.search_result_signal.emit(*self.search_results[self._search_result_index])
            self.replace_button.setEnabled(True)

    def _check_position(self, forward=True):
        try:
            # if the position of the textCursor was changed by the user, move the search index
            cur_pos = self._tabwidget.currentWidget().textCursor().position()
            st, _f, pa, idx = self.search_results[self._search_result_index]
            sear_pos = idx + len(st)
            if cur_pos != sear_pos:
                first_idx = self._get_current_index_for_current_file()
                if first_idx != -1:
                    st, _f, pa, idx = self.search_results[first_idx]
                    sear_pos = idx + len(st)
                    while cur_pos > sear_pos and self._tabwidget.currentWidget().filename == pa:
                        first_idx += 1
                        st, _f, pa, idx = self.search_results[first_idx]
                        sear_pos = idx + len(st)
                    self._search_result_index = first_idx
                    if forward:
                        self._search_result_index -= 1
                else:
                    self._reset(True)
        except:
            pass

    def _get_current_index_for_current_file(self):
        for index in range(len(self.search_results)):
            _st, _f, pa, _idx = self.search_results[index]
            if self._tabwidget.currentWidget().filename == pa:
                return index
        return -1

    def on_search_result(self, search_text, found, path, index):
        '''
        Slot to handle the signals for search result. This signals are forwarded used
        search_result_signal.
        '''
        if found and search_text == self.current_search_text:
            self.search_results_fileset.add(path)
            item = (search_text, found, path, index)
            if item not in self.search_results:
                self.search_results.append((search_text, found, path, index))
            if self._wait_for_result:
                self._search_result_index += 1
                if index >= self._tabwidget.currentWidget().textCursor().position() or self._tabwidget.currentWidget().filename != path:
                    self._wait_for_result = False
                    self.search_result_signal.emit(*self.search_results[self._search_result_index])
                    self.replace_button.setEnabled(True)
        if self.search_results:
            if len(self.search_results_fileset) > 1:
                for item in self.search_results_fileset:
                    pkg, path = package_name(os.path.dirname(item))
                    itemstr = '%s [%s]' % (os.path.basename(item), pkg)
                    if not self.found_files_list.findItems(itemstr, Qt.MatchExactly):
                        list_item = QListWidgetItem(itemstr)
                        list_item.setToolTip(item)
                        self.found_files_list.addItem(list_item)
                        self.found_files_frame.setVisible(True)
                        self.found_files_list.setVisible(len(self.search_results_fileset) > 0)
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
                search_text, _found, path, index = self.search_results[self._search_result_index]
                cursor = self._tabwidget.currentWidget().textCursor()
                if cursor.selectedText() == search_text:
                    rptxt = self.replace_field.text()
                    for rindex in range(self._search_result_index + 1, len(self.search_results)):
                        st, _f, pa, idx = self.search_results[rindex]
                        if path == pa:
                            self.search_results.pop(rindex)
                            self.search_results.insert(rindex, (st, _f, pa, idx + len(rptxt) - len(st)))
                        else:
                            break
                    self._remove_search_result(self._search_result_index)
                    self.replace_signal.emit(search_text, path, index, rptxt)
                else:
                    self.replace_button.setEnabled(False)
            except:
                import traceback
                print traceback.format_exc()
                pass

    def on_itemActivated(self, item):
        '''
        Go to the results for the selected file entry in the list.
        '''
        item_path = item.toolTip()
        new_search_index = -1
        tmp_index = -1
        search_index = -1
        tmp_search_text = ''
        for search_text, found, path, index in self.search_results:
            new_search_index += 1
            if item_path == path:
                if tmp_index == -1:
                    tmp_index = new_search_index
                    tmp_search_text = search_text
                    search_index = index
                if new_search_index > self._search_result_index:
                    self.search_result_signal.emit(search_text, found, path, index)
                    self._search_result_index = new_search_index
                    self._update_label()
                    return
        if tmp_index != -1:
            self.search_result_signal.emit(tmp_search_text, True, item_path, search_index)
            self._search_result_index = tmp_index
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
        self.found_files_list.setVisible(False)
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

    def _remove_search_result(self, index):
        try:
            self.search_results.pop(index)
            # create new set with files contain the search text
            new_path_set = set(path for _st, _fd, path, _idx in self.search_results)
            # remove the file from the list widget
            for pp in self.search_results_fileset - new_path_set:
                for wi_idx in range(self.found_files_list.count()):
                    # we have to compare each tooltip of the item
                    if pp == self.found_files_list.item(wi_idx).toolTip():
                        self.found_files_list.takeItem(wi_idx)
                        break
            self.search_results_fileset = new_path_set
            self.found_files_list.setVisible(len(self.search_results_fileset) > 0)
        except:
            import traceback
            print traceback.format_exc()

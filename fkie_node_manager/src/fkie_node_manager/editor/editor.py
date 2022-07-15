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



from python_qt_binding.QtCore import QPoint, QSize, Qt, Signal
from python_qt_binding.QtGui import QColor, QIcon, QKeySequence, QPalette, QTextCursor
import os

import rospy


from fkie_master_discovery.common import masteruri_from_ros
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager.common import package_name
from fkie_node_manager.detailed_msg_box import MessageBox
from fkie_node_manager.run_dialog import PackageDialog
from fkie_node_manager.select_dialog import SelectDialog
import fkie_node_manager as nm

from .line_number_widget import LineNumberWidget
from .graph_view import GraphViewWidget
from .text_edit import TextEdit
from .text_search_frame import TextSearchFrame
from .text_search_thread import TextSearchThread

try:
    from python_qt_binding.QtGui import QApplication, QAction, QLineEdit, QDockWidget, QWidget, QMainWindow
    from python_qt_binding.QtGui import QDialog, QInputDialog, QLabel, QMenu, QPushButton, QTabWidget, QTextBrowser, QTextEdit
    from python_qt_binding.QtGui import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QAction, QLineEdit, QDockWidget, QWidget, QMainWindow
    from python_qt_binding.QtWidgets import QDialog, QInputDialog, QLabel, QMenu, QPushButton, QTabWidget, QTextBrowser, QTextEdit
    from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy


class EditorTabWidget(QTabWidget):
    '''
    This class was overloaded to close tabs on middle mouse click
    '''

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MidButton:
            close_index = self.tabBar().tabAt(event.pos())
            if close_index > -1:
                self.tabCloseRequested.emit(close_index)
                event.setAccepted(True)
        if not event.isAccepted():
            QTabWidget.mouseReleaseEvent(event)

    def currentWidget(self):
        '''
        This is an overloaded function to use with LineNumberWidget
        '''
        return QTabWidget.currentWidget(self).get_text_edit()

    def widget(self, index):
        '''
        This is an overloaded function to use with LineNumberWidget
        '''
        return QTabWidget.widget(self, index).get_text_edit()


class Editor(QMainWindow):
    '''
    Creates a dialog to edit a launch file.
    '''
    finished_signal = Signal(list)
    '''
    finished_signal has as parameter the filenames of the initialization and is emitted, if this
    dialog was closed.
    '''

    def __init__(self, filenames, search_text='', master_name='', parent=None):
        '''
        :param filenames: a list with filenames. The last one will be activated.
        :type filenames: [str]
        :param str search_text: if not empty, searches in new document for first occurrence of the given text
        '''
        QMainWindow.__init__(self, parent)
        self.setObjectName('Editor - %s' % utf8(filenames))
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowFlags(Qt.Window)
        self.mIcon = nm.settings().icon('crystal_clear_edit_launch.png')
        self._error_icon = nm.settings().icon('warning.png')
        self._info_icon = nm.settings().icon('info.png')
        self._empty_icon = QIcon()
        self.setWindowIcon(self.mIcon)
        window_title = "ROSLaunch Editor"
        if filenames:
            window_title = self.__getTabName(filenames[0])
        self.setWindowTitle('%s @%s' % (window_title, master_name))
        self.init_filenames = filenames
        self._search_node_count = 0
        self._search_thread = None
        self._last_search_request = None
        # list with all open files
        self.files = []
        # create tabs for files
        self.main_widget = QWidget(self)
        self.main_widget.setObjectName("editorMain")
        self.verticalLayout = QVBoxLayout(self.main_widget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(1)
        self.verticalLayout.setObjectName("verticalLayout")

        self.tabWidget = EditorTabWidget(self)
        self.tabWidget.setTabPosition(QTabWidget.North)
        self.tabWidget.setDocumentMode(True)
        self.tabWidget.setTabsClosable(True)
        self.tabWidget.setMovable(False)
        self.tabWidget.setObjectName("tabWidget")
        self.tabWidget.tabCloseRequested.connect(self.on_close_tab)
        self.tabWidget.currentChanged.connect(self.on_tab_changed)

        self.verticalLayout.addWidget(self.tabWidget)
        self.log_dock = self._create_log_bar()
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        # self.verticalLayout.addWidget(self.log_bar)
        self.buttons = self._create_buttons()
        self.verticalLayout.addWidget(self.buttons)
        self.setCentralWidget(self.main_widget)

        self.find_dialog = TextSearchFrame(self.tabWidget, self)
        self.find_dialog.found_signal.connect(self.on_search_result)
        self.find_dialog.replace_signal.connect(self.on_replace)
        self.addDockWidget(Qt.RightDockWidgetArea, self.find_dialog)

        self.graph_view = GraphViewWidget(self.tabWidget, self)
        self.graph_view.load_signal.connect(self.on_graph_load_file)
        self.graph_view.goto_signal.connect(self.on_graph_goto)
        self.graph_view.search_signal.connect(self.on_load_request)
        self.graph_view.finished_signal.connect(self.on_graph_finished)
        self.graph_view.info_signal.connect(self.on_graph_info)
        self.addDockWidget(Qt.RightDockWidgetArea, self.graph_view)
        self.readSettings()
        self.find_dialog.setVisible(False)
        self.graph_view.setVisible(False)
        nm.nmd().file.changed_file.connect(self.on_changed_file)
        nm.nmd().file.packages_available.connect(self._on_new_packages)
        # open the files
        for f in filenames:
            if f:
                self.on_load_request(f, search_text, only_launch=True)
        self.log_dock.setVisible(False)
        try:
            pal = self.tabWidget.palette()
            self._default_color = pal.color(QPalette.Window)
            color = QColor.fromRgb(nm.settings().host_color(master_name, self._default_color.rgb()))
            bg_style_launch_dock = "QWidget#editorMain { background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 %s, stop: 0.7 %s);}" % (color.name(), self._default_color.name())
            self.setStyleSheet('%s' % (bg_style_launch_dock))
        except Exception as _:
            pass
            # import traceback
            # print(traceback.format_exc())

#  def __del__(self):
#    print "******** destroy", self.objectName()

    def _create_buttons(self):
        # create the buttons line
        self.buttons = QWidget(self)
        self.horizontalLayout = QHBoxLayout(self.buttons)
        self.horizontalLayout.setContentsMargins(3, 0, 3, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        # add open upper launchfile button
        self.upperButton = QPushButton(self)
        self.upperButton.setObjectName("upperButton")
        self.upperButton.clicked.connect(self.on_upperButton_clicked)
        self.upperButton.setIcon(nm.settings().icon('up.png'))
        self.upperButton.setShortcut("Ctrl+U")
        self.upperButton.setToolTip('Open the file which include the current file (Ctrl+U)')
        self.upperButton.setFlat(True)
        self.horizontalLayout.addWidget(self.upperButton)

        # add the goto button
        self.gotoButton = QPushButton(self)
        self.gotoButton.setObjectName("gotoButton")
        self.gotoButton.clicked.connect(self.on_shortcut_goto)
        self.gotoButton.setText(self._translate("&Goto line"))
        self.gotoButton.setShortcut("Ctrl+G")
        self.gotoButton.setToolTip('Open a goto dialog (Ctrl+G)')
        self.gotoButton.setFlat(True)
        self.horizontalLayout.addWidget(self.gotoButton)
        # add a tag button
        self.tagButton = self._create_tag_button(self)
        self.horizontalLayout.addWidget(self.tagButton)
        # add save button
        self.saveButton = QPushButton(self)
        self.saveButton.setObjectName("saveButton")
        self.saveButton.setIcon(QIcon.fromTheme("document-save"))
        self.saveButton.clicked.connect(self.on_saveButton_clicked)
        self.saveButton.setText(self._translate("&Save"))
        self.saveButton.setShortcut("Ctrl+S")
        self.saveButton.setToolTip('Save the changes to the file (Ctrl+S)')
        self.saveButton.setFlat(True)
        self.horizontalLayout.addWidget(self.saveButton)
        # add spacer
        spacerItem = QSpacerItem(515, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        # add line number label
        self.pos_label = QLabel()
        self.horizontalLayout.addWidget(self.pos_label)
        # add spacer
        spacerItem = QSpacerItem(515, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        # add show log button
        self.show_log_button = QPushButton("Log>>", self)
        self.show_log_button.setObjectName("show_log_button")
        self.show_log_button.clicked.connect(self.on_toggled_log)
        self.show_log_button.setFlat(True)
        self.show_log_button.setCheckable(True)
        self.horizontalLayout.addWidget(self.show_log_button)
        # add graph button
        self.graphButton = QPushButton(self)
        self.graphButton.setObjectName("graphButton")
        self.graphButton.toggled.connect(self.on_toggled_graph)
        self.graphButton.setText("Includ&e Graph >>")
        self.graphButton.setCheckable(True)
        self.graphButton.setShortcut("Ctrl+E")
        self.graphButton.setToolTip('Shows include and include from files (Ctrl+E)')
        self.graphButton.setFlat(True)
        self.horizontalLayout.addWidget(self.graphButton)
        # add the search button
        self.searchButton = QPushButton(self)
        self.searchButton.setObjectName("searchButton")
#        self.searchButton.clicked.connect(self.on_shortcut_find)
        self.searchButton.toggled.connect(self.on_toggled_find)
        self.searchButton.setText(self._translate("&Find >>"))
        self.searchButton.setToolTip('Open a search dialog (Ctrl+F)')
        self.searchButton.setFlat(True)
        self.searchButton.setCheckable(True)
        self.horizontalLayout.addWidget(self.searchButton)
        # add the replace button
        self.replaceButton = QPushButton(self)
        self.replaceButton.setObjectName("replaceButton")
#        self.replaceButton.clicked.connect(self.on_shortcut_replace)
        self.replaceButton.toggled.connect(self.on_toggled_replace)
        self.replaceButton.setText(self._translate("&Replace >>"))
        self.replaceButton.setToolTip('Open a search&replace dialog (Ctrl+R)')
        self.replaceButton.setFlat(True)
        self.replaceButton.setCheckable(True)
        self.horizontalLayout.addWidget(self.replaceButton)
        return self.buttons

    def _create_log_bar(self):
        self.log_dock = QDockWidget(self)
        self.log_dock.setObjectName('LogFrame')
        self.log_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.log_bar = QWidget(self)
        self.horizontal_layout_log_bar = QHBoxLayout(self.log_bar)
        self.horizontal_layout_log_bar.setContentsMargins(2, 0, 2, 0)
        self.horizontal_layout_log_bar.setObjectName("horizontal_layout_log_bar")
        # add info label
        self._log_warning_count = 0
        self.log_browser = QTextEdit()
        self.log_browser.setObjectName("log_browser")
        self.log_browser.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.log_browser.setLineWrapMode(QTextEdit.NoWrap)
        # self.log_browser.setMaximumHeight(120)
        color = QColor(255, 255, 235)
        bg_style = "QTextEdit#log_browser { background-color: %s;}" % color.name()
        self.log_bar.setStyleSheet("%s" % (bg_style))
        self.horizontal_layout_log_bar.addWidget(self.log_browser)
        # add hide button
        self.clear_log_button = QPushButton("clear", self)
        self.clear_log_button.setObjectName("clear_log_button")
        self.clear_log_button.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
        self.clear_log_button.clicked.connect(self.on_clear_log_button_clicked)
        self.clear_log_button.setFlat(True)
        self.horizontal_layout_log_bar.addWidget(self.clear_log_button)
        self.log_dock.setWidget(self.log_bar)
        return self.log_dock

    def keyPressEvent(self, event):
        '''
        Enable the shortcats for search and replace
        '''
        if event.key() == Qt.Key_Escape:
            self.reject()
        elif event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_F:
            if self.tabWidget.currentWidget().hasFocus():
                if not self.searchButton.isChecked():
                    self.searchButton.setChecked(True)
                else:
                    self.on_toggled_find(True)
            else:
                self.searchButton.setChecked(not self.searchButton.isChecked())
        elif event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_R:
            if self.tabWidget.currentWidget().hasFocus():
                if not self.replaceButton.isChecked():
                    self.replaceButton.setChecked(True)
                else:
                    self.on_toggled_replace(True)
            else:
                self.replaceButton.setChecked(not self.replaceButton.isChecked())
        elif event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_E:
            if self.tabWidget.currentWidget().hasFocus():
                if not self.graphButton.isChecked():
                    self.graphButton.setChecked(True)
                else:
                    self.on_toggled_graph(True)
            else:
                self.graphButton.setChecked(not self.graphButton.isChecked())
        elif event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_W:
            self.on_close_tab(self.tabWidget.currentIndex())
        elif event.modifiers() in [Qt.ControlModifier, Qt.AltModifier] and event.key() == Qt.Key_Up:
            self.on_upperButton_clicked()
        elif event.modifiers() in [Qt.ControlModifier, Qt.AltModifier] and event.key() == Qt.Key_Down:
            self.on_downButton_clicked()
        else:
            event.accept()
            QMainWindow.keyPressEvent(self, event)

    def _translate(self, text):
        if hasattr(QApplication, "UnicodeUTF8"):
            return QApplication.translate("Editor", text, None, QApplication.UnicodeUTF8)
        else:
            return QApplication.translate("Editor", text, None)

    def readSettings(self):
        if nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup("editor")
            maximized = settings.value("maximized", 'false') == 'true'
            if maximized:
                self.showMaximized()
            else:
                self.resize(settings.value("size", QSize(800, 640)))
                self.move(settings.value("pos", QPoint(0, 0)))
            try:
                self.restoreState(settings.value("window_state"))
            except Exception:
                import traceback
                print(traceback.format_exc())
            settings.endGroup()

    def storeSetting(self):
        if nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup("editor")
            settings.setValue("size", self.size())
            settings.setValue("pos", self.pos())
            settings.setValue("maximized", self.isMaximized())
            settings.setValue("window_state", self.saveState())
            settings.endGroup()

    def on_load_request(self, filename, search_text='', insert_index=-1, goto_line=-1, only_launch=False, count_results=0):
        '''
        Loads a file in a new tab or focus the tab, if the file is already open.

        :param str filename: the path to file
        :param str search_text: if not empty, searches in new document for first occurrence of the given text
        '''
        if not filename:
            return
        self.tabWidget.setUpdatesEnabled(False)
        try:
            if filename not in self.files:
                tab_name = self.__getTabName(filename)
                editor = TextEdit(filename, parent=self)
                linenumber_editor = LineNumberWidget(editor)
                tab_index = 0
                if insert_index > -1:
                    tab_index = self.tabWidget.insertTab(insert_index, linenumber_editor, tab_name)
                else:
                    tab_index = self.tabWidget.addTab(linenumber_editor, tab_name)
                self.files.append(filename)
                editor.setCurrentPath(os.path.basename(filename))
                editor.load_request_signal.connect(self.on_load_request)
                editor.document().modificationChanged.connect(self.on_editor_modificationChanged)
                editor.cursorPositionChanged.connect(self.on_editor_positionChanged)
                editor.setFocus(Qt.OtherFocusReason)
#                editor.textChanged.connect(self.on_text_changed)
                editor.undoAvailable.connect(self.on_text_changed)
                self.tabWidget.setCurrentIndex(tab_index)
#                self.find_dialog.set_search_path(filename)
            else:
                for i in range(self.tabWidget.count()):
                    if self.tabWidget.widget(i).filename == filename:
                        self.tabWidget.setCurrentIndex(i)
                        break
            self.tabWidget.setUpdatesEnabled(True)
            if search_text:
                if only_launch:
                    self.find_dialog.found_files_list.clear()
                try:
                    self._search_thread.stop()
                    self._search_thread = None
                except Exception:
                    pass
                # TODO: put all text of all tabs into path_text
                rospy.logdebug("serach for '%s'" % search_text)
                self._search_node_count = 0
                self._search_thread = TextSearchThread(search_text, filename, recursive=True, only_launch=only_launch, count_results=count_results)
                self._search_thread.search_result_signal.connect(self.on_search_result_on_open)
                self._search_thread.warning_signal.connect(self.on_search_result_warning)
                self._last_search_request = (filename, search_text, insert_index, goto_line, only_launch)
                if not self.graph_view.is_loading():
                    self.on_graph_info("search thread: start search for '%s'" % self._search_thread._search_text)
                    self._search_thread.start()
            if goto_line != -1:
                self.tabWidget.currentWidget().goto(goto_line, True)
            self.upperButton.setEnabled(self.tabWidget.count() > 1)
        except Exception as err:
            self.tabWidget.setUpdatesEnabled(True)
            import traceback
            msg = "Error while open %s: %s" % (filename, traceback.format_exc())
            rospy.logwarn(msg)
            MessageBox.critical(self, "Error", utf8(err), msg)
            if self.tabWidget.count() == 0:
                self.close()

    def on_graph_load_file(self, path, insert_after=True):
        insert_index = self.tabWidget.currentIndex() + 1
        if not insert_after and insert_index > 1:
            insert_index = self.tabWidget.currentIndex()
        self.on_load_request(path, insert_index=insert_index)

    def on_graph_goto(self, path, linenr):
        if path == self.tabWidget.currentWidget().filename:
            if linenr != -1:
                self.tabWidget.currentWidget().goto(linenr, True)

    def on_graph_finished(self):
        self.on_graph_info("build tree: finished", False)
        if self.graph_view.has_warnings:
            self.graphButton.setIcon(self._info_icon)
        else:
            self.graphButton.setIcon(self._empty_icon)
        if self._search_thread:
            try:
                self._search_thread.find_args_not_set = True
                self._search_thread.start()
                self.on_graph_info("search thread: start search for '%s'" % self._search_thread._search_text)
            except Exception:
                pass

    def on_graph_info(self, msg, warning=False):
        text_color = "#000000"
        if warning:
            self._log_warning_count += 1
            if self._log_warning_count == 1:
                self.show_log_button.setIcon(self._error_icon)
            text_color = "#FE9A2E"
        text = ('<pre style="padding:10px;"><dt><font color="%s">'
                '%s</font></dt></pre>' % (text_color, msg))
        self.log_browser.append(text)

    def on_text_changed(self, value=""):
        if self.tabWidget.currentWidget().hasFocus():
            self.find_dialog.file_changed(self.tabWidget.currentWidget().filename)
            self._last_search_request = None

    def on_tab_changed(self, index):
        if index > -1:
            self.graph_view.set_file(self.tabWidget.widget(index).filename, self.tabWidget.widget(0).filename)
            self._last_search_request = None

    def on_close_tab(self, tab_index):
        '''
        Signal handling to close single tabs.
        :param int tab_index: tab index to close
        '''
        try:
            doremove = True
            w = self.tabWidget.widget(tab_index)
            if w.document().isModified():
                name = self.__getTabName(w.filename)
                result = MessageBox.question(self, "Unsaved Changes", '\n\n'.join(["Save the file before closing?", name]))
                if result == MessageBox.Yes:
                    self.tabWidget.currentWidget().save()
                elif result == MessageBox.No:
                    pass
                elif rospy.is_shutdown():
                    doremove = False
            if doremove:
                # remove the indexed files
                if w.filename in self.files:
                    self.files.remove(w.filename)
                # close tab
                self.tabWidget.removeTab(tab_index)
                # close editor, if no tabs are open
                if not self.tabWidget.count():
                    self.close()
            self._last_search_request = None
        except Exception:
            import traceback
            rospy.logwarn("Error while close tab %s: %s", str(tab_index), traceback.format_exc(1))
        self.upperButton.setEnabled(self.tabWidget.count() > 1)

    def reject(self):
        if self.find_dialog.isVisible():
            self.searchButton.setChecked(not self.searchButton.isChecked())
        else:
            self.close()

    def on_changed_file(self, grpc_path, mtime):
        if grpc_path in self.files:
            for i in range(self.tabWidget.count()):
                if self.tabWidget.widget(i).filename == grpc_path:
                    self.tabWidget.widget(i).file_changed(mtime)
                    break
        if self._last_search_request is not None:
            self.on_load_request(*self._last_search_request)

    def closeEvent(self, event):
        '''
        Test the open files for changes and save this if needed.
        '''
        changed = []
        # get the names of all changed files
        for i in range(self.tabWidget.count()):
            w = self.tabWidget.widget(i)
            if w.document().isModified():
                changed.append(self.__getTabName(w.filename))
        if changed:
            # ask the user for save changes
            if self.isHidden():
                buttons = MessageBox.Yes | MessageBox.No
            else:
                buttons = MessageBox.Yes | MessageBox.No | MessageBox.Cancel
            result = MessageBox.question(self, "Unsaved Changes", '\n\n'.join(["Save the file before closing?", '\n'.join(changed)]), buttons=buttons)
            if result == MessageBox.Yes:
                for i in range(self.tabWidget.count()):
                    w = self.tabWidget.widget(i).save()
                self.graph_view.clear_cache()
                event.accept()
            elif result == MessageBox.No:
                event.accept()
            # elif rospy.is_shutdown():
            else:
                event.ignore()
        else:
            event.accept()
        if event.isAccepted():
            self.storeSetting()
            nm.nmd().file.changed_file.disconnect(self.on_changed_file)
            nm.nmd().file.packages_available.connect(self._on_new_packages)
            self.finished_signal.emit(self.init_filenames)

    def on_editor_modificationChanged(self, value=None):
        '''
        If the content was changed, a '*' will be shown in the tab name.
        '''
        tab_name = self.__getTabName(self.tabWidget.currentWidget().filename)
        if (self.tabWidget.currentWidget().document().isModified()):
            tab_name = '*%s' % tab_name
        self.tabWidget.setTabText(self.tabWidget.currentIndex(), tab_name)

    def on_editor_positionChanged(self):
        '''
        Shows the number of the line and column in a label.
        '''
        cursor = self.tabWidget.currentWidget().textCursor()
        self.pos_label.setText(':%s:%s #%s' % (cursor.blockNumber() + 1, cursor.columnNumber(), cursor.position()))

    def __getTabName(self, lfile):
        base = os.path.basename(lfile).replace('.launch', '')
        (package, _) = package_name(os.path.dirname(lfile))
        return '%s [%s]' % (base, package)

    def _on_new_packages(self, url):
        try:
            if nmdurl.nmduri_from_path(url) == nmdurl.nmduri_from_path(self.tabWidget.currentWidget().filename):
                rospy.logdebug("packages updated, rebuild graph")
                if self.graph_view.has_none_packages:
                    self.graph_view.clear_cache()
        except Exception:
            import traceback
            print(traceback.format_exc())

    ##############################################################################
    # HANDLER for buttons
    ##############################################################################

    def on_clear_log_button_clicked(self):
        self._log_warning_count = 0
        self.show_log_button.setIcon(self._empty_icon)
        self.log_browser.clear()
        self.log_dock.setVisible(False)
        self.show_log_button.setChecked(False)
        self.tabWidget.currentWidget().setFocus()

    def on_upperButton_clicked(self):
        '''
        Opens the file which include the current open file
        '''
        if self.tabWidget.currentIndex() != 0:
            self.graph_view.find_parent_file()

    def on_downButton_clicked(self):
        '''
        Select editor right from current.
        '''
        if self.tabWidget.currentIndex() < self.tabWidget.count():
            self.tabWidget.setCurrentIndex(self.tabWidget.currentIndex() + 1)

    def on_saveButton_clicked(self):
        '''
        Saves the current document. This method is called if the C{save button}
        was clicked.
        '''
        saved, errors, msg = self.tabWidget.currentWidget().save()
        if errors:
            if msg:
                rospy.logwarn(msg)
                MessageBox.critical(self, "Error", "Error while save file: %s" % os.path.basename(self.tabWidget.currentWidget().filename), detailed_text=msg)
            self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._error_icon)
            self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), msg)
            self.on_graph_info("saved failed %s: %s" % (self.tabWidget.currentWidget().filename, msg), True)
        elif saved:
            self.on_graph_info("saved %s" % self.tabWidget.currentWidget().filename)
            self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._empty_icon)
            self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), '')
            self.graph_view.clear_cache()

    def on_shortcut_find(self):
        pass

    def on_toggled_log(self, value):
        '''
        Shows the log bar
        '''
        if value:
            self.log_dock.setVisible(True)
        else:
            self.log_dock.setVisible(False)
            self.tabWidget.currentWidget().setFocus()

    def on_toggled_graph(self, value):
        '''
        Shows the search frame
        '''
        if value:
            self.graph_view.enable()
        else:
            # self.replaceButton.setChecked(False)
            self.graph_view.setVisible(False)
            self.tabWidget.currentWidget().setFocus()

    def on_toggled_find(self, value, only_results=False):
        '''
        Shows the search frame
        '''
        if value:
            self.find_dialog.enable()
            self.find_dialog.find_frame.setVisible(not only_results)
            self.find_dialog.recursive_search_box.setVisible(not only_results)
            if not only_results:
                # clear results if not search text exists and we show not only search results
                if not self.find_dialog.search_field.text():
                    self.find_dialog.found_files_list.clear()
        else:
            self.replaceButton.setChecked(False)
            self.find_dialog.setVisible(False)
            self.tabWidget.currentWidget().setFocus()

    def on_toggled_replace(self, value):
        '''
        Shows the replace lineedit in the search frame
        '''
        if value:
            self.searchButton.setChecked(True)
        self.find_dialog.set_replace_visible(value)

    def on_shortcut_goto(self):
        '''
        Opens a C{goto} dialog.
        '''
        value = 1
        ok = False
        try:
            value, ok = QInputDialog.getInt(self, "Goto", self.tr("Line number:"),
                                                  QLineEdit.Normal, minValue=1, step=1)
        except Exception:
            value, ok = QInputDialog.getInt(self, "Goto", self.tr("Line number:"),
                                                  QLineEdit.Normal, min=1, step=1)
        if ok:
            self.tabWidget.currentWidget().goto(value)
        self.tabWidget.currentWidget().setFocus(Qt.ActiveWindowFocusReason)

    ##############################################################################
    # SLOTS for search dialog
    ##############################################################################

    def on_search_result(self, search_text, found, path, startpos, endpos, linenr=-1, line_text=''):
        '''
        A slot to handle a found text. It goes to the position in the text and select
        the searched text. On new file it will be open.
        :param search_text: the searched text
        :type search_text: str
        :param found: the text was found or not
        :type found: bool
        :param path: the path of the file the text was found
        :type path: str
        :param startpos: the position in the text
        :type startpos: int
        :param endpos: the end position in the text
        :type endpos: int
        '''
        if found:
            if self.tabWidget.currentWidget().filename != path:
                focus_widget = QApplication.focusWidget()
                self.on_load_request(path)
                focus_widget.setFocus()
            self.tabWidget.currentWidget().select(startpos, endpos, False)

    def on_search_result_on_open(self, search_text, found, path, startpos, endpos, linenr, line_text):
        '''
        Like on_search_result, but skips the text in comments.
        '''
        if found:
            self._search_node_count += 1
            if self._search_node_count > 1:
                self.on_toggled_find(True, only_results=True)
            self.find_dialog.current_search_text = search_text
            self.find_dialog.on_search_result(search_text, found, path, startpos, endpos, linenr, line_text)
            self.on_graph_info("search thread: found %s in '%s:%d'" % (search_text, path, linenr))
            if self.tabWidget.currentWidget().filename != path:
                focus_widget = QApplication.focusWidget()
                self.on_load_request(path)
                if focus_widget is not None:
                    focus_widget.setFocus()
            self.tabWidget.currentWidget().select(startpos, endpos, True)
        # self.on_search_result(search_text, found, path, startpos, endpos, linenr, line_text)

    def on_search_result_warning(self, msg):
        self.on_graph_info("search thread: %s" % (msg), True)

    def on_replace(self, search_text, path, index, replaced_text):
        '''
        A slot to handle a text replacement of the TextSearchFrame.
        :param search_text: the searched text
        :type search_text: str
        :param path: the path of the file the text was found
        :type path: str
        :param index: the position in the text
        :type index: int
        :param replaced_text: the new text
        :type replaced_text: str
        '''
        cursor = self.tabWidget.currentWidget().textCursor()
        if cursor.selectedText() == search_text:
            cursor.insertText(replaced_text)

    ##############################################################################
    # LAUNCH TAG insertion
    ##############################################################################

    def _show_custom_parameter_dialog(self):
        methods = {'nm/associations': self._on_add_cp_associations,
                   'capability_group': self._on_add_cp_capability_group,
                   'nm/kill_on_stop': self._on_add_cp_kill_on_stop,
                   'autostart/delay': self._on_add_cp_as_delay,
                   'autostart/exclude': self._on_add_cp_as_exclude,
                   'autostart/required_publisher': self._on_add_cp_as_req_publisher,
                   'respawn/max': self._on_add_cp_r_max,
                   'respawn/min_runtime': self._on_add_cp_r_min_runtime,
                   'respawn/delay': self._on_add_cp_r_delay,
                   }

        res = SelectDialog.getValue('Insert custom parameter', "Select parameter to insert:",
                                    sorted(methods.keys()), exclusive=True, parent=self,
                                    select_if_single=False,
                                    store_geometry='insert_param')
        tags2insert = res[0]
        for tag in tags2insert:
            methods[tag]()

    def _create_tag_button(self, parent=None):
        btn = QPushButton(parent)
        btn.setObjectName("tagButton")
        btn.setText(self._translate("Add &tag"))
        # btn.setShortcut("Ctrl+T")
        btn.setToolTip('Adds a ROS launch tag to launch file')
        btn.setMenu(self._create_tag_menu(btn))
        btn.setFlat(True)
        return btn

    def _create_tag_menu(self, parent=None):
        # creates a tag menu
        tag_menu = QMenu("ROS Tags", parent)
        # group tag
        add_group_tag_action = QAction("<group>", self, statusTip="", triggered=self._on_add_group_tag)
        add_group_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+g"))
        tag_menu.addAction(add_group_tag_action)
        # node tag
        add_node_tag_action = QAction("<node>", self, statusTip="", triggered=self._on_add_node_tag)
        add_node_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+n"))
        tag_menu.addAction(add_node_tag_action)
        # node tag with all attributes
        add_node_tag_all_action = QAction("<node all>", self, statusTip="", triggered=self._on_add_node_tag_all)
        tag_menu.addAction(add_node_tag_all_action)
        # include tag with all attributes
        add_include_tag_all_action = QAction("<include>", self, statusTip="", triggered=self._on_add_include_tag_all)
        add_include_tag_all_action.setShortcuts(QKeySequence("Ctrl+Shift+i"))
        tag_menu.addAction(add_include_tag_all_action)
        # remap
        add_remap_tag_action = QAction("<remap>", self, statusTip="", triggered=self._on_add_remap_tag)
        add_remap_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+r"))
        tag_menu.addAction(add_remap_tag_action)
        # env tag
        add_env_tag_action = QAction("<env>", self, statusTip="", triggered=self._on_add_env_tag)
        tag_menu.addAction(add_env_tag_action)
        # param tag
        add_param_clipboard_tag_action = QAction("<param value>", self, statusTip="add value from clipboard", triggered=self._on_add_param_clipboard_tag)
        add_param_clipboard_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+p"))
        tag_menu.addAction(add_param_clipboard_tag_action)
        add_param_tag_action = QAction("<param>", self, statusTip="", triggered=self._on_add_param_tag)
        add_param_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+Alt+p"))
        tag_menu.addAction(add_param_tag_action)
        # param tag with all attributes
        add_param_tag_all_action = QAction("<param all>", self, statusTip="", triggered=self._on_add_param_tag_all)
        tag_menu.addAction(add_param_tag_all_action)
        # rosparam tag with all attributes
        add_rosparam_tag_all_action = QAction("<rosparam>", self, statusTip="", triggered=self._on_add_rosparam_tag_all)
        tag_menu.addAction(add_rosparam_tag_all_action)
        # arg tag with default definition
        add_arg_tag_default_action = QAction("<arg default>", self, statusTip="", triggered=self._on_add_arg_tag_default)
        add_arg_tag_default_action.setShortcuts(QKeySequence("Ctrl+Shift+a"))
        tag_menu.addAction(add_arg_tag_default_action)
        # arg tag with value definition
        add_arg_tag_value_action = QAction("<arg value>", self, statusTip="", triggered=self._on_add_arg_tag_value)
        add_arg_tag_value_action.setShortcuts(QKeySequence("Ctrl+Alt+a"))
        tag_menu.addAction(add_arg_tag_value_action)

        # test tag
        add_test_tag_action = QAction("<test>", self, statusTip="", triggered=self._on_add_test_tag)
        add_test_tag_action.setShortcuts(QKeySequence("Ctrl+Alt+t"))
        tag_menu.addAction(add_test_tag_action)
        # test tag with all attributes
        add_test_tag_all_action = QAction("<test all>", self, statusTip="", triggered=self._on_add_test_tag_all)
        tag_menu.addAction(add_test_tag_all_action)
        sub_cp_menu = QMenu("Custom parameters", parent)

        show_cp_dialog_action = QAction("Show Dialog", self, statusTip="", triggered=self._show_custom_parameter_dialog)
        show_cp_dialog_action.setShortcuts(QKeySequence("Ctrl+Shift+d"))
        sub_cp_menu.addAction(show_cp_dialog_action)

        add_cp_associations_action = QAction("nm/associations", self, statusTip="", triggered=self._on_add_cp_associations)
        add_cp_associations_action.setShortcuts(QKeySequence("Ctrl+Alt+a"))
        sub_cp_menu.addAction(add_cp_associations_action)

        sub_cp_as_menu = QMenu("Autostart", parent)
        add_cp_as_delay_action = QAction("delay", self, statusTip="", triggered=self._on_add_cp_as_delay)
        sub_cp_as_menu.addAction(add_cp_as_delay_action)
        add_cp_as_exclude_action = QAction("exclude", self, statusTip="", triggered=self._on_add_cp_as_exclude)
        sub_cp_as_menu.addAction(add_cp_as_exclude_action)
        add_cp_as_req_publisher_action = QAction("required publisher", self, statusTip="", triggered=self._on_add_cp_as_req_publisher)
        sub_cp_as_menu.addAction(add_cp_as_req_publisher_action)
        sub_cp_menu.addMenu(sub_cp_as_menu)

        sub_cp_r_menu = QMenu("Respawn", parent)
        add_cp_r_max_action = QAction("max", self, statusTip="", triggered=self._on_add_cp_r_max)
        sub_cp_r_menu.addAction(add_cp_r_max_action)
        add_cp_r_min_runtime_action = QAction("min_runtime", self, statusTip="", triggered=self._on_add_cp_r_min_runtime)
        sub_cp_r_menu.addAction(add_cp_r_min_runtime_action)
        add_cp_r_delay_action = QAction("delay", self, statusTip="", triggered=self._on_add_cp_r_delay)
        sub_cp_r_menu.addAction(add_cp_r_delay_action)
        sub_cp_menu.addMenu(sub_cp_r_menu)

        add_cp_capability_group_action = QAction("capability_group", self, statusTip="", triggered=self._on_add_cp_capability_group)
        add_cp_capability_group_action.setShortcuts(QKeySequence("Ctrl+Alt+p"))
        sub_cp_menu.addAction(add_cp_capability_group_action)
        add_cp_kill_on_stop_action = QAction("nm/kill_on_stop", self, statusTip="True or time to wait in ms", triggered=self._on_add_cp_kill_on_stop)
        add_cp_kill_on_stop_action.setShortcuts(QKeySequence("Ctrl+Shift+k"))
        sub_cp_menu.addAction(add_cp_kill_on_stop_action)
        tag_menu.addMenu(sub_cp_menu)
        return tag_menu

    def _insert_text(self, text, cursor_pose=None, selection_len=None):
        if self.tabWidget.currentWidget().isReadOnly():
            return
        cursor = self.tabWidget.currentWidget().textCursor()
        if not cursor.isNull():
            cursor.beginEditBlock()
            col = cursor.columnNumber()
            spaces = ''.join([' ' for _ in range(col)])
            curr_cursor_pos = cursor.position()
            cursor.insertText(text.replace('\n', '\n%s' % spaces))
            if cursor_pose is not None:
                cursor.setPosition(curr_cursor_pos + cursor_pose, QTextCursor.MoveAnchor)
                if selection_len is not None:
                    cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, selection_len)
            cursor.endEditBlock()
            self.tabWidget.currentWidget().setTextCursor(cursor)
            self.tabWidget.currentWidget().setFocus(Qt.OtherFocusReason)

    def _on_add_group_tag(self):
        self._insert_text('<group ns="namespace" clear_params="true|false">\n'
                          '</group>', 11, 9)

    def _get_package_dialog(self):
        muri = masteruri_from_ros()
        if self.init_filenames:
            muri = nmdurl.masteruri(self.init_filenames[0])
        return PackageDialog(muri)

    def _on_add_node_tag(self):
        dia = self._get_package_dialog()
        if dia.exec_():
            self._insert_text('<node name="%s" pkg="%s" type="%s">\n'
                              '</node>' % (dia.binary, dia.package, dia.binary))

    def _on_add_node_tag_all(self):
        dia = self._get_package_dialog()
        if dia.exec_():
            self._insert_text('<node name="%s" pkg="%s" type="%s"\n'
                              '      args="arg1" machine="machine_name"\n'
                              '      respawn="true" required="true"\n'
                              '      ns="foo" clear_params="true|false"\n'
                              '      output="log|screen" cwd="ROS_HOME|node"\n'
                              '      launch-prefix="prefix arguments">\n'
                              '</node>' % (dia.binary, dia.package, dia.binary))

    def _on_add_include_tag_all(self):
        self._insert_text('<include file="$(find pkg-name)/path/filename.xml"\n'
                          '         ns="foo" clear_params="true|false">\n'
                          '</include>', 22, 27)

    def _on_add_remap_tag(self):
        self._insert_text('<remap from="original" to="new"/>', 13, 8)

    def _on_add_env_tag(self):
        self._insert_text('<env name="variable" value="value"/>', 11, 8)

    def _on_add_param_clipboard_tag(self):
        lines = QApplication.clipboard().mimeData().text().splitlines()
        name = ""
        if len(lines) == 1:
            name = lines[0]
        self._insert_text('<param name="%s" value="value" />' % name, 22 + len(name), 5)

    def _on_add_param_tag(self):
        self._insert_text('<param name="name" value="value" />', 13, 4)

    def _on_add_param_tag_all(self):
        self._insert_text('<param name="name" value="value"\n'
                          '       type="str|int|double|bool"\n'
                          '       textfile="$(find pkg-name)/path/file.txt"\n'
                          '       binfile="$(find pkg-name)/path/file"\n'
                          '       command="$(find pkg-name)/exe \'$(find pkg-name)/arg.txt\'">\n'
                          '</param>', 13, 4)

    def _on_add_rosparam_tag_all(self):
        self._insert_text('<rosparam param="name"\n'
                          '       file="$(find pkg-name)/path/foo.yaml"\n'
                          '       command="load|dump|delete"\n'
                          '       ns="namespace"\n'
                          '       subst_value="true|false">\n'
                          '</rosparam>', 17, 4)

    def _on_add_arg_tag_default(self):
        self._insert_text('<arg name="foo" default="1" />', 11, 3)

    def _on_add_arg_tag_value(self):
        self._insert_text('<arg name="foo" value="bar" />', 11, 3)

    def _on_add_test_tag(self):
        dia = self._get_package_dialog()
        if dia.exec_():
            self._insert_text('<test name="%s" pkg="%s" type="%s" test-name="test_%s">\n'
                              '</test>' % (dia.binary, dia.package, dia.binary, dia.binary))

    def _on_add_test_tag_all(self):
        dia = self._get_package_dialog()
        if dia.exec_():
            self._insert_text('<test name="%s" pkg="%s" type="%s" test-name="test_%s">\n'
                              '      args="arg1" time-limit="60.0"\n'
                              '      ns="foo" clear_params="true|false"\n'
                              '      cwd="ROS_HOME|node" retry="0"\n'
                              '      launch-prefix="prefix arguments">\n'
                              '</test>' % (dia.binary, dia.package, dia.binary, dia.binary))

    def _on_add_cp_capability_group(self):
        self._insert_text('<param name="capability_group" value="demo" />', 38, 4)

    def _on_add_cp_kill_on_stop(self):
        self._insert_text('<param name="nm/kill_on_stop" value="100" hint="[ms]" />', 34, 3)

    def _on_add_cp_associations(self):
        self._insert_text('<param name="nm/associations" value="node1,node2" hint="list of nodes" />', 34, 11)

    def _on_add_cp_as_delay(self):
        self._insert_text('<param name="autostart/delay" value="1" hint="[seconds]" />', 37, 1)

    def _on_add_cp_as_exclude(self):
        self._insert_text('<param name="autostart/exclude" value="True" />', 39, 4)

    def _on_add_cp_as_req_publisher(self):
        self._insert_text('<param name="autostart/required/publisher" value="topic" />', 50, 5)

    def _on_add_cp_r_max(self):
        self._insert_text('<param name="respawn/max" value="10" />', 33, 2)

    def _on_add_cp_r_min_runtime(self):
        self._insert_text('<param name="respawn/min_runtime" value="10" hint="[seconds]" />', 41, 2)

    def _on_add_cp_r_delay(self):
        self._insert_text('<param name="respawn/delay" value="5" hint="[seconds]" />', 31, 2)

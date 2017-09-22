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

from python_qt_binding.QtCore import QFileInfo, QPoint, QSize, Qt, Signal
from python_qt_binding.QtGui import QIcon, QKeySequence, QTextCursor, QTextDocument
import os

import rospy

from node_manager_fkie.common import package_name, utf8
from node_manager_fkie.run_dialog import PackageDialog
from node_manager_fkie.launch_config import LaunchConfig
import node_manager_fkie as nm

from .line_number_widget import LineNumberWidget
from .graph_view import GraphViewWidget
from .text_edit import TextEdit
from .text_search_frame import TextSearchFrame
from .text_search_thread import TextSearchThread
from node_manager_fkie.detailed_msg_box import MessageBox

try:
    from python_qt_binding.QtGui import QApplication, QAction, QLineEdit, QWidget, QMainWindow
    from python_qt_binding.QtGui import QDialog, QInputDialog, QLabel, QMenu, QPushButton, QTabWidget
    from python_qt_binding.QtGui import QHBoxLayout, QVBoxLayout, QSpacerItem, QSplitter, QSizePolicy
except:
    from python_qt_binding.QtWidgets import QApplication, QAction, QLineEdit, QWidget, QMainWindow
    from python_qt_binding.QtWidgets import QDialog, QInputDialog, QLabel, QMenu, QPushButton, QTabWidget
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

    def __init__(self, filenames, search_text='', parent=None):
        '''
        @param filenames: a list with filenames. The last one will be activated.
        @type filenames: C{[str, ...]}
        @param search_text: if not empty, searches in new document for first occurrence of the given text
        @type search_text: C{str} (Default: C{Empty String})
        '''
        QMainWindow.__init__(self, parent)
        self.setObjectName('Editor - %s' % utf8(filenames))
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowFlags(Qt.Window)
        self.mIcon = QIcon(":/icons/crystal_clear_edit_launch.png")
        self._error_icon = QIcon(":/icons/crystal_clear_warning.png")
        self._empty_icon = QIcon()
        self.setWindowIcon(self.mIcon)
        window_title = "ROSLaunch Editor"
        if filenames:
            window_title = self.__getTabName(filenames[0])
        self.setWindowTitle(window_title)
        self.init_filenames = list(filenames)
        self._search_thread = None
        # list with all open files
        self.files = []
        # create tabs for files
        self.main_widget = QWidget(self)
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
        self.buttons = self._create_buttons()
        self.verticalLayout.addWidget(self.buttons)
        self.setCentralWidget(self.main_widget)

        self.find_dialog = TextSearchFrame(self.tabWidget, self)
        self.find_dialog.search_result_signal.connect(self.on_search_result)
        self.find_dialog.replace_signal.connect(self.on_replace)
        self.addDockWidget(Qt.RightDockWidgetArea, self.find_dialog)

        self.graph_view = GraphViewWidget(self.tabWidget, self)
        self.graph_view.load_signal.connect(self.on_graph_load_file)
        self.graph_view.goto_signal.connect(self.on_graph_goto)
        self.addDockWidget(Qt.RightDockWidgetArea, self.graph_view)
        # open the files
        for f in filenames:
            if f:
                self.on_load_request(os.path.normpath(f), search_text)
        self.readSettings()
        self.find_dialog.setVisible(False)
        self.graph_view.setVisible(False)

#  def __del__(self):
#    print "******** destroy", self.objectName()

    def _create_buttons(self):
        # create the buttons line
        self.buttons = QWidget(self)
        self.horizontalLayout = QHBoxLayout(self.buttons)
        self.horizontalLayout.setContentsMargins(4, 0, 4, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        # add open upper launchfile button
        self.upperButton = QPushButton(self)
        self.upperButton.setObjectName("upperButton")
        self.upperButton.clicked.connect(self.on_upperButton_clicked)
        self.upperButton.setIcon(QIcon(":/icons/up.png"))
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
        # add graph button
        self.graphButton = QPushButton(self)
        self.graphButton.setObjectName("graphButton")
        self.graphButton.toggled.connect(self.on_toggled_graph)
        self.graphButton.setText("Includ&e Graph >>")
        self.graphButton.setCheckable(True)
        # self.graphButton.setIcon(QIcon(":/icons/button_graph.png"))
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
                print traceback.format_exc()
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

    def on_load_request(self, filename, search_text='', insert_index=-1, goto_line=-1):
        '''
        Loads a file in a new tab or focus the tab, if the file is already open.
        @param filename: the path to file
        @type filename: C{str}
        @param search_text: if not empty, searches in new document for first occurrence of the given text
        @type search_text: C{str} (Default: C{Empty String})
        '''
        if not filename:
            return
        self.tabWidget.setUpdatesEnabled(False)
        try:
            if filename not in self.files:
                tab_name = self.__getTabName(filename)
                editor = TextEdit(filename, self.tabWidget)
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
        except Exception:
            import traceback
            rospy.logwarn("Error while open %s: %s", filename, traceback.format_exc(1))
        self.tabWidget.setUpdatesEnabled(True)
        if search_text:
            try:
                self._search_thread.stop()
                self._search_thread = None
            except Exception:
                pass
            self._search_thread = TextSearchThread(search_text, filename, path_text=self.tabWidget.widget(0).document().toPlainText(), recursive=True)
            self._search_thread.search_result_signal.connect(self.on_search_result_on_open)
            self._search_thread.start()
        if goto_line != -1:
            self._goto(goto_line, True)
        self.upperButton.setEnabled(self.tabWidget.count() > 1)

    def on_graph_load_file(self, path, insert_after=True):
        insert_index = self.tabWidget.currentIndex() + 1
        if not insert_after:
            insert_index = self.tabWidget.currentIndex()
        self.on_load_request(path, insert_index=insert_index)

    def on_graph_goto(self, path, linenr):
        if path == self.tabWidget.currentWidget().filename:
            if linenr != -1:
                self._goto(linenr, True)

    def on_text_changed(self, value=""):
        if self.tabWidget.currentWidget().hasFocus():
            self.find_dialog.file_changed(self.tabWidget.currentWidget().filename)

    def on_tab_changed(self, index):
        if index > -1:
            self.graph_view.set_file(self.tabWidget.widget(index).filename, self.tabWidget.widget(0).filename)

    def on_close_tab(self, tab_index):
        '''
        Signal handling to close single tabs.
        @param tab_index: tab index to close
        @type tab_index: C{int}
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
                else:
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
        except Exception:
            import traceback
            rospy.logwarn("Error while close tab %s: %s", str(tab_index), traceback.format_exc(1))
        self.upperButton.setEnabled(self.tabWidget.count() > 1)

    def reject(self):
        if self.find_dialog.isVisible():
            self.searchButton.setChecked(not self.searchButton.isChecked())
        else:
            self.close()

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
            else:
                event.ignore()
        else:
            event.accept()
        if event.isAccepted():
            self.storeSetting()
            self.finished_signal.emit(self.init_filenames)

    def on_editor_modificationChanged(self, value=None):
        '''
        If the content was changed, a '*' will be shown in the tab name.
        '''
        tab_name = self.__getTabName(self.tabWidget.currentWidget().filename)
        if (self.tabWidget.currentWidget().document().isModified()) or not QFileInfo(self.tabWidget.currentWidget().filename).exists():
            tab_name = ''.join(['*', tab_name])
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

    ##############################################################################
    # HANDLER for buttons
    ##############################################################################

    def on_upperButton_clicked(self):
        '''
        Opens the file which include the current open file
        '''
        if self.tabWidget.currentIndex() != 0:
            self.graph_view.find_parent_file()

    def on_saveButton_clicked(self):
        '''
        Saves the current document. This method is called if the C{save button}
        was clicked.
        '''
        saved, errors, msg = self.tabWidget.currentWidget().save(True)
        if errors:
            MessageBox.critical(self, "Error", msg)
            self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._error_icon)
            self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), msg)
        elif saved:
            self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._empty_icon)
            self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), '')
            self.graph_view.clear_cache()

    def on_shortcut_find(self):
        pass

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

    def on_toggled_find(self, value):
        '''
        Shows the search frame
        '''
        if value:
            self.find_dialog.enable()
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
            self._goto(value)
        self.tabWidget.currentWidget().setFocus(Qt.ActiveWindowFocusReason)

    def _goto(self, linenr, select_line=True):
            if linenr > self.tabWidget.currentWidget().document().blockCount():
                linenr = self.tabWidget.currentWidget().document().blockCount()
            curpos = self.tabWidget.currentWidget().textCursor().blockNumber() + 1
            while curpos != linenr:
                mov = QTextCursor.NextBlock if curpos < linenr else QTextCursor.PreviousBlock
                self.tabWidget.currentWidget().moveCursor(mov)
                curpos = self.tabWidget.currentWidget().textCursor().blockNumber() + 1
            self.tabWidget.currentWidget().moveCursor(QTextCursor.EndOfBlock)
            self.tabWidget.currentWidget().moveCursor(QTextCursor.StartOfBlock, QTextCursor.KeepAnchor)

    ##############################################################################
    # SLOTS for search dialog
    ##############################################################################

    def on_search_result(self, search_text, found, path, index):
        '''
        A slot to handle a found text. It goes to the position in the text and select
        the searched text. On new file it will be open.
        :param search_text: the searched text
        :type search_text: str
        :param found: the text was found or not
        :type found: bool
        :param path: the path of the file the text was found
        :type path: str
        :param index: the position in the text
        :type index: int
        '''
        if found:
            if self.tabWidget.currentWidget().filename != path:
                focus_widget = QApplication.focusWidget()
                self.on_load_request(path)
                focus_widget.setFocus()
            cursor = self.tabWidget.currentWidget().textCursor()
            cursor.setPosition(index, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, len(search_text))
            self.tabWidget.currentWidget().setTextCursor(cursor)

    def on_search_result_on_open(self, search_text, found, path, index):
        '''
        Like on_search_result, but skips the text in comments.
        '''
        if found:
            if self.tabWidget.currentWidget().filename != path:
                focus_widget = QApplication.focusWidget()
                self.on_load_request(path)
                focus_widget.setFocus()
            comment_start = self.tabWidget.currentWidget().document().find('<!--', index, QTextDocument.FindBackward)
            if not comment_start.isNull():
                comment_end = self.tabWidget.currentWidget().document().find('-->', comment_start)
                if not comment_end.isNull() and comment_end.position() > index + len(search_text):
                    # commented -> retrun
                    return
        self.on_search_result(search_text, found, path, index)

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

    def _create_tag_button(self, parent=None):
        btn = QPushButton(parent)
        btn.setObjectName("tagButton")
        btn.setText(self._translate("Add &tag"))
        btn.setShortcut("Ctrl+T")
        btn.setToolTip('Adds a ROS launch tag to launch file (Ctrl+T)')
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
        add_param_tag_action = QAction("<param>", self, statusTip="", triggered=self._on_add_param_tag)
        add_param_tag_action.setShortcuts(QKeySequence("Ctrl+Shift+p"))
        tag_menu.addAction(add_param_tag_action)
        # param capability group tag
        add_param_cap_group_tag_action = QAction("<param capability group>", self, statusTip="", triggered=self._on_add_param_cap_group_tag)
        add_param_cap_group_tag_action.setShortcuts(QKeySequence("Ctrl+Alt+p"))
        tag_menu.addAction(add_param_cap_group_tag_action)
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
        return tag_menu

    def _insert_text(self, text):
        cursor = self.tabWidget.currentWidget().textCursor()
        if not cursor.isNull():
            col = cursor.columnNumber()
            spaces = ''.join([' ' for _ in range(col)])
            cursor.insertText(text.replace('\n', '\n%s' % spaces))
            self.tabWidget.currentWidget().setFocus(Qt.OtherFocusReason)

    def _on_add_group_tag(self):
        self._insert_text('<group ns="namespace" clear_params="true|false">\n'
                          '</group>')

    def _on_add_node_tag(self):
        dia = PackageDialog()
        if dia.exec_():
            self._insert_text('<node name="%s" pkg="%s" type="%s">\n'
                              '</node>' % (dia.binary, dia.package, dia.binary))

    def _on_add_node_tag_all(self):
        dia = PackageDialog()
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
                          '</include>')

    def _on_add_remap_tag(self):
        self._insert_text('<remap from="original" to="new"/>')

    def _on_add_env_tag(self):
        self._insert_text('<env name="variable" value="value"/>')

    def _on_add_param_tag(self):
        self._insert_text('<param name="ns_name" value="value" />')

    def _on_add_param_cap_group_tag(self):
        self._insert_text('<param name="capability_group" value="demo" />')

    def _on_add_param_tag_all(self):
        self._insert_text('<param name="ns_name" value="value"\n'
                          '       type="str|int|double|bool"\n'
                          '       textfile="$(find pkg-name)/path/file.txt"\n'
                          '       binfile="$(find pkg-name)/path/file"\n'
                          '       command="$(find pkg-name)/exe \'$(find pkg-name)/arg.txt\'">\n'
                          '</param>')

    def _on_add_rosparam_tag_all(self):
        self._insert_text('<rosparam param="param-name"\n'
                          '       file="$(find pkg-name)/path/foo.yaml"\n'
                          '       command="load|dump|delete"\n'
                          '       ns="namespace">\n'
                          '</rosparam>')

    def _on_add_arg_tag_default(self):
        self._insert_text('<arg name="foo" default="1" />')

    def _on_add_arg_tag_value(self):
        self._insert_text('<arg name="foo" value="bar" />')

    def _on_add_test_tag(self):
        dia = PackageDialog()
        if dia.exec_():
            self._insert_text('<test name="%s" pkg="%s" type="%s" test-name="test_%s">\n'
                              '</test>' % (dia.binary, dia.package, dia.binary, dia.binary))

    def _on_add_test_tag_all(self):
        dia = PackageDialog()
        if dia.exec_():
            self._insert_text('<test name="%s" pkg="%s" type="%s" test-name="test_%s">\n'
                              '      args="arg1" time-limit="60.0"\n'
                              '      ns="foo" clear_params="true|false"\n'
                              '      cwd="ROS_HOME|node" retry="0"\n'
                              '      launch-prefix="prefix arguments">\n'
                              '</test>' % (dia.binary, dia.package, dia.binary, dia.binary))

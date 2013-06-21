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

import os
from python_qt_binding import QtGui
from python_qt_binding import QtCore

import roslib
import rospy
from xml_highlighter import XmlHighlighter

from common import package_name

class Editor(QtGui.QTextEdit):
  '''
  The XML editor to handle the included files. If an included file in the opened
  launch file is detected, this can be open by STRG+(mouse click) in a new
  editor.
  '''
  
  load_request_signal = QtCore.Signal(str)
  ''' @ivar: A signal for request to open a configuration file'''
  
  def __init__(self, filename, parent=None):
    self.parent = parent
    QtGui.QTextEdit.__init__(self, parent)
    self.setObjectName(' - '.join(['Editor', filename]))
    font = QtGui.QFont()
    font.setFamily("Fixed".decode("utf-8"))
    font.setPointSize(12)
    self.setFont(font)
    self.setLineWrapMode(QtGui.QTextEdit.NoWrap)
    self.setTabStopWidth(25)
    self.setAcceptRichText(False)
    self.setCursorWidth(2)
    self.setFontFamily("courier new")
    self.setProperty("backgroundVisible", True)
    self.regexp_list = [QtCore.QRegExp("\\binclude\\b"), QtCore.QRegExp("\\btextfile\\b"),
                        QtCore.QRegExp("\\bfile\\b")]
    self.filename = filename
    if self.filename:
      file = QtCore.QFile(filename);
      if file.open(QtCore.QIODevice.ReadOnly | QtCore.QIODevice.Text):
        self.setText(unicode(file.readAll(), "utf-8"))

    self.path = '.'

#  def __del__(self):
#    print "********** desctroy:", self.objectName()

  def save(self):
    '''
    Saves changes to the file.
    '''
    if self.document().isModified():
      file = QtCore.QFile(self.filename)
      if file.open(QtCore.QIODevice.WriteOnly | QtCore.QIODevice.Text):
        file.write(self.toPlainText().encode('utf-8'))
        self.document().setModified(False)
        return True
      else:
        QtGui.QMessageBox.critical(self, "Error", "Cannot write XML file")
        return False
    return False

  def setCurrentPath(self, path):
    '''
    Sets the current working path. This path is to open the included files, which
    contains the relative path.
    @param path: the path of the current opened file (without the file)
    @type path: C{str}
    '''
    self.path = path

  def interpretPath(self, path):
    '''
    Tries to determine the path of the included file. The statement of 
    C{$(find 'package')} will be resolved.
    @param path: the sting which contains the included path
    @type path: C{str}
    @return: if no leading C{os.sep} is detected, the path setted by L{setCurrentPath()}
    will be prepend. C{$(find 'package')} will be resolved. Otherwise the parameter 
    itself will be returned
    @rtype: C{str} 
    '''
    path = path.strip()
    index = path.find('$')
    if index > -1:
      startIndex = path.find('(', index)
      if startIndex > -1:
        endIndex = path.find(')', startIndex+1)
        script = path[startIndex+1:endIndex].split()
        if len(script) == 2 and (script[0] == 'find'):
          pkg = roslib.packages.get_pkg_dir(script[1])
          return os.path.normpath(''.join([pkg, '/', path[endIndex+1:]]))
    elif len(path) > 0 and path[0] != '/':
      return os.path.normpath(''.join([self.path, '/', path]))
    return os.path.normpath(path)
    
  def index(self, text):
    '''
    Searches in the given text for key indicates the including of a file and 
    return their index.
    @param text: text to find
    @type text: C{str}
    @return: the index of the including key or -1
    @rtype: C{int}
    '''
    for pattern in self.regexp_list:
      index = pattern.indexIn(text)
      if index > -1:
        return index
    return -1

  def includedFiles(self):
    '''
    Returns all included files in the document.
    '''
    result = []
    b = self.document().begin()
    while b != self.document().end():
      text = b.text()
      index = self.index(text)
      if index > -1:
        startIndex = text.find('"', index)
        if startIndex > -1:
          endIndex = text.find('"', startIndex+1)
          fileName = text[startIndex+1:endIndex]
          if len(fileName) > 0:
            path = self.interpretPath(fileName)
            file = QtCore.QFile(path)
            if file.exists():
              result.append(path)
      b = b.next()
    return result

  def fileWithText(self, search_text):
    '''
    Searches for given text in this document and all included files.
    @param search_text: text to find
    @type search_text: C{str}
    @return: the list with all files contain the text
    @rtype: C{[str, ...]}
    '''
    result = []
    start_pos = QtGui.QTextCursor()
    search_result = self.document().find(search_text, start_pos.position()+1)
    if not search_result.isNull():
      result.append(self.filename)
    inc_files = self.includedFiles()
    for f in inc_files:
      editor = Editor(f, None)
      result[len(result):] = editor.fileWithText(search_text)
    return result

  def mouseReleaseEvent(self, event):
    '''
    Opens the new editor, if the user clicked on the included file and sets the 
    default cursor.
    '''
    if event.modifiers() == QtCore.Qt.ControlModifier or event.modifiers() == QtCore.Qt.ShiftModifier:
      cursor = self.cursorForPosition(event.pos())
      index = self.index(cursor.block().text())
      if index > -1:
        startIndex = cursor.block().text().find('"', index)
        if startIndex > -1:
          endIndex = cursor.block().text().find('"', startIndex+1)
          fileName = cursor.block().text()[startIndex+1:endIndex]
          if len(fileName) > 0:
            file = QtCore.QFile(self.interpretPath(fileName))
            if not file.exists():
              # create a new file, if it does not exists
              result = QtGui.QMessageBox.question(self, "File not found", '\n\n'.join(["Create a new file?", file.fileName()]), QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
              if result == QtGui.QMessageBox.Yes:
                dir = os.path.dirname(file.fileName())
                if not os.path.exists(dir):
                  os.makedirs(dir)
                with open(file.fileName(),'w') as f:
                  if file.fileName().endswith('.launch'):
                    f.write('<launch>\n\n</launch>')
                self.load_request_signal.emit(file.fileName())
            else:
              self.load_request_signal.emit(file.fileName())
    QtGui.QTextEdit.mouseReleaseEvent(self, event)
  
  def mouseMoveEvent(self, event):
    '''
    Sets the X{QtCore.Qt.PointingHandCursor} if the control key is pressed and 
    the mouse is over the included file.
    '''
    if event.modifiers() == QtCore.Qt.ControlModifier or event.modifiers() == QtCore.Qt.ShiftModifier:
      cursor = self.cursorForPosition(event.pos())
      index = self.index(cursor.block().text())
      if index > -1:
        self.viewport().setCursor(QtCore.Qt.PointingHandCursor)
      else:
        self.viewport().setCursor(QtCore.Qt.IBeamCursor)
    else:
      self.viewport().setCursor(QtCore.Qt.IBeamCursor)
    QtGui.QTextEdit.mouseMoveEvent(self, event)
  
  def keyPressEvent(self, event):
    '''
    Enable the mouse tracking by X{setMouseTracking()} if the control key is pressed.
    '''
    if event.key() == QtCore.Qt.Key_Control or event.key() == QtCore.Qt.Key_Shift:
      self.setMouseTracking(True)
    if event.key() != QtCore.Qt.Key_Escape:
      # handle the shifting of the block
      if event.key() == QtCore.Qt.Key_Tab:
        self.shiftText()
      else:
        QtGui.QTextEdit.keyPressEvent(self, event)
    else:
      event.accept()
      QtGui.QTextEdit.keyPressEvent(self, event)

  def keyReleaseEvent(self, event):
    '''
    Disable the mouse tracking by X{setMouseTracking()} if the control key is 
    released and set the cursor back to X{QtCore.Qt.IBeamCursor}.
    '''
    if event.key() == QtCore.Qt.Key_Control or event.key() == QtCore.Qt.Key_Shift:
      self.setMouseTracking(False)
      self.viewport().setCursor(QtCore.Qt.IBeamCursor)
    QtGui.QTextEdit.keyReleaseEvent(self, event)

  def shiftText(self):
    '''
    Increase (Decrease) indentation using Tab (Ctrl+Tab).
    '''
    cursor = self.textCursor()
    if not cursor.isNull():
      key_mod = QtGui.QApplication.keyboardModifiers()
      # one undo operation
      cursor.beginEditBlock()
      start = cursor.selectionStart()
      end = cursor.selectionEnd()
      cursor.setPosition(start)
      block_start = cursor.blockNumber()
      cursor.setPosition(end)
      block_end = cursor.blockNumber()
      if block_end-block_start == 0:
        # shift one line two spaces to the left
        if key_mod & QtCore.Qt.ControlModifier:
          for s in range(2):
            cursor.movePosition(QtGui.QTextCursor.StartOfLine)
            cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, 1)
            if cursor.selectedText() == ' ':
              cursor.insertText('')
            elif cursor.selectedText() == "\t":
              cursor.insertText('')
              break
          cursor.movePosition(QtGui.QTextCursor.StartOfLine)
        else:
        # shift one line two spaces to the right
          cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, end-start)
          cursor.insertText('  ')
      else:
        # shift the selected block two spaces to the left
        if key_mod & QtCore.Qt.ControlModifier:
          removed = 0
          for i in reversed(range(start, end)):
            cursor.setPosition(i)
            if cursor.atBlockStart():
              cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, 2)
              if cursor.selectedText() == '  ':
                cursor.insertText('')
                removed += 2
              else:
                cursor.movePosition(QtGui.QTextCursor.StartOfLine)
                cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, 1)
                if cursor.selectedText() == ' ':
                  cursor.insertText('')
                  removed += 1
                elif cursor.selectedText() == "\t":
                  cursor.insertText('')
                  removed += 1
          cursor.setPosition(start)
          cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, end-start-removed)
        else:
          # shift selected block two spaces to the right
          inserted = 0
          for i in reversed(range(start, end)):
            cursor.setPosition(i)
            if cursor.atBlockStart():
              cursor.insertText('  ')
              inserted += 2
          cursor.setPosition(start)
          cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, end-start+inserted)
      self.setTextCursor(cursor)
      cursor.endEditBlock()


class FindDialog(QtGui.QDialog):
  '''
  A dialog to find text in the Editor.
  '''

  def __init__(self, parent=None):
    QtGui.QDialog.__init__(self, parent)
    self.setObjectName('FindDialog')
    self.setWindowTitle('Search')
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")

    self.content = QtGui.QWidget(self)
    self.contentLayout = QtGui.QFormLayout(self.content)
#    self.contentLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
#    self.contentLayout.setVerticalSpacing(0)
    self.contentLayout.setContentsMargins(0, 0, 0, 0)
    self.verticalLayout.addWidget(self.content)

    label = QtGui.QLabel("Find:", self.content)
    self.search_field = QtGui.QLineEdit(self.content)
    self.contentLayout.addRow(label, self.search_field)
    replace_label = QtGui.QLabel("Replace:", self.content)
    self.replace_field = QtGui.QLineEdit(self.content)
    self.contentLayout.addRow(replace_label, self.replace_field)
    self.recursive = QtGui.QCheckBox("recursive search")
    self.contentLayout.addRow(self.recursive)
    self.result_label = QtGui.QLabel("")
    self.verticalLayout.addWidget(self.result_label)
    self.found_files = QtGui.QListWidget()
    self.found_files.setVisible(False)
    self.found_files.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    self.verticalLayout.addWidget(self.found_files)
    
    self.buttonBox = QtGui.QDialogButtonBox(self)
    self.find_button = QtGui.QPushButton(self.tr("&Find"))
    self.find_button.setDefault(True)
    self.buttonBox.addButton(self.find_button, QtGui.QDialogButtonBox.ActionRole)
    self.replace_button = QtGui.QPushButton(self.tr("&Replace/Find"))
    self.buttonBox.addButton(self.replace_button, QtGui.QDialogButtonBox.ActionRole)
    self.buttonBox.addButton(QtGui.QDialogButtonBox.Close)
    self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
    self.buttonBox.setObjectName("buttonBox")
    self.verticalLayout.addWidget(self.buttonBox)

#    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), self.accept)
    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), self.reject)
    QtCore.QMetaObject.connectSlotsByName(self)
    
    self.search_text = ''
    self.search_pos = QtGui.QTextCursor()

#  def __del__(self):
#    print "********** desctroy:", self.objectName()


class XmlEditor(QtGui.QDialog):
  '''
  Creates a dialog to edit a launch file.
  '''
  
  finished_signal = QtCore.Signal(list)
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
    QtGui.QDialog.__init__(self, parent)
    self.setObjectName(' - '.join(['xmlEditor', str(filenames)]))
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    self.setWindowFlags(QtCore.Qt.Window)
    self.resize(800,640)
    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_edit_launch.png")
    self.setWindowIcon(self.mIcon)
    self.setWindowTitle("ROSLaunch Editor");
#    self.finished.connect(self.closeEvent)
    self.init_filenames = list(filenames)
    
    self.files = []
    '''@ivar: list with all open files '''
    
    # create tabs for files
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setContentsMargins(0, 0, 0, 0)
    self.verticalLayout.setObjectName("verticalLayout")
    self.tabWidget = QtGui.QTabWidget(self)
    self.tabWidget.setTabPosition(QtGui.QTabWidget.North)
    self.tabWidget.setDocumentMode(True)
    self.tabWidget.setTabsClosable(True)
    self.tabWidget.setMovable(False)
    self.tabWidget.setObjectName("tabWidget")
    self.tabWidget.tabCloseRequested.connect(self.on_close_tab)
    self.verticalLayout.addWidget(self.tabWidget)
    
    # create the buttons line
    self.buttons = QtGui.QWidget(self)
    self.horizontalLayout = QtGui.QHBoxLayout(self.buttons)
    self.horizontalLayout.setContentsMargins(4, 0, 4, 0)
    self.horizontalLayout.setObjectName("horizontalLayout")
    # add the search button
    self.searchButton = QtGui.QPushButton(self)
    self.searchButton.setObjectName("searchButton")
    self.searchButton.clicked.connect(self.on_shortcut_find)
    self.searchButton.setText(QtGui.QApplication.translate("XmlEditor", "Search", None, QtGui.QApplication.UnicodeUTF8))
    self.searchButton.setShortcut(QtGui.QApplication.translate("XmlEditor", "Ctrl+F", None, QtGui.QApplication.UnicodeUTF8))
    self.searchButton.setToolTip('Open a search dialog (Ctrl+F)')
    self.horizontalLayout.addWidget(self.searchButton)
    # add the got button
    self.gotoButton = QtGui.QPushButton(self)
    self.gotoButton.setObjectName("gotoButton")
    self.gotoButton.clicked.connect(self.on_shortcut_goto)
    self.gotoButton.setText(QtGui.QApplication.translate("XmlEditor", "Goto line", None, QtGui.QApplication.UnicodeUTF8))
    self.gotoButton.setShortcut(QtGui.QApplication.translate("XmlEditor", "Ctrl+L", None, QtGui.QApplication.UnicodeUTF8))
    self.gotoButton.setToolTip('Open a goto dialog (Ctrl+L)')
    self.horizontalLayout.addWidget(self.gotoButton)
    # add spacer
    spacerItem = QtGui.QSpacerItem(515, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
    self.horizontalLayout.addItem(spacerItem)
    # add line number label
    self.pos_label = QtGui.QLabel()
    self.horizontalLayout.addWidget(self.pos_label)
    # add spacer
    spacerItem = QtGui.QSpacerItem(515, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
    self.horizontalLayout.addItem(spacerItem)
    # add save button
    self.saveButton = QtGui.QPushButton(self)
    self.saveButton.setObjectName("saveButton")
    self.saveButton.clicked.connect(self.on_saveButton_clicked)
    self.saveButton.setText(QtGui.QApplication.translate("XmlEditor", "Save", None, QtGui.QApplication.UnicodeUTF8))
    self.saveButton.setShortcut(QtGui.QApplication.translate("XmlEditor", "Ctrl+S", None, QtGui.QApplication.UnicodeUTF8))
    self.saveButton.setToolTip('Save the changes to the file (Ctrl+S)')
    self.horizontalLayout.addWidget(self.saveButton)
    self.verticalLayout.addWidget(self.buttons)

    #create the find dialog
    self.find_dialog = FindDialog(self)
    self.find_dialog.buttonBox.clicked.connect(self.on_find_dialog_clicked)
    self.find_dialog.found_files.itemActivated.connect(self.find_dialog_itemActivated)

#    self._shortcut_find = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+F", "find text")), self)
#    self._shortcut_find.activated.connect(self.on_shortcut_find)

    #open the files
    for f in filenames:
      if f:
        self.on_load_request(os.path.normpath(f), search_text)

#    print "================ create", self.objectName()
#
#  def __del__(self):
#    print "******** destroy", self.objectName()

  def on_load_request(self, filename, search_text=''):
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
      if not filename in self.files:
        tab_name = self.__getTabName(filename)
        editor = Editor(filename, self.tabWidget)
        tab_index = self.tabWidget.addTab(editor, tab_name)
        self.files.append(filename)
        editor.setCurrentPath(os.path.basename(filename))
        editor.load_request_signal.connect(self.on_load_request)

        hl = XmlHighlighter(editor.document())
        editor.textChanged.connect(self.on_editor_textChanged)
        editor.cursorPositionChanged.connect(self.on_editor_positionChanged)
        editor.setFocus(QtCore.Qt.OtherFocusReason)
        self.tabWidget.setCurrentIndex(tab_index)
      else:
        for i in range(self.tabWidget.count()):
          if self.tabWidget.widget(i).filename == filename:
            self.tabWidget.setCurrentIndex(i)
            break
    except:
      import traceback
      rospy.logwarn("Error while open %s: %s", filename, traceback.format_exc())
    
    self.tabWidget.setUpdatesEnabled(True)
    if search_text:
      if self.find(search_text, False):
        if not self.find_dialog.search_pos.isNull():
          self.tabWidget.currentWidget().moveCursor(QtGui.QTextCursor.StartOfLine)

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
        result = QtGui.QMessageBox.question(self, "Unsaved Changes", '\n\n'.join(["Save the file before closing?", name]), QtGui.QMessageBox.Yes | QtGui.QMessageBox.No | QtGui.QMessageBox.Cancel)
        if result == QtGui.QMessageBox.Yes:
          self.tabWidget.currentWidget().save()
        elif result == QtGui.QMessageBox.No:
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
    except:
      import traceback
      rospy.logwarn("Error while close tab %s: %s", str(tab_index), traceback.format_exc())

#  def hideEvent(self, event):
#    self.close()

  def closeEvent (self, event):
    '''
    Test the open files for changes and save this if needed.
    '''
    changed = []
    #get the names of all changed files
    for i in range(self.tabWidget.count()):
      w = self.tabWidget.widget(i)
      if w.document().isModified():
        changed.append(self.__getTabName(w.filename))
    if changed:
      # ask the user for save changes
      if self.isHidden():
        buttons = QtGui.QMessageBox.Yes | QtGui.QMessageBox.No
      else:
        buttons = QtGui.QMessageBox.Yes | QtGui.QMessageBox.No | QtGui.QMessageBox.Cancel
      result = QtGui.QMessageBox.question(self, "Unsaved Changes", '\n\n'.join(["Save the file before closing?", '\n'.join(changed)]), buttons)
      if result == QtGui.QMessageBox.Yes:
        for i in range(self.tabWidget.count()):
          w = self.tabWidget.widget(i).save()
        event.accept() 
      elif result == QtGui.QMessageBox.No:
        event.accept()
      else:
        event.ignore()
    else:
      event.accept()
    if event.isAccepted():
      self.finished_signal.emit(self.init_filenames)

  def on_saveButton_clicked(self):
    '''
    Saves the current document. This method is called if the C{save button} 
    was clicked.
    '''
    if self.tabWidget.currentWidget().save():
      self.on_editor_textChanged()
  
  def on_editor_textChanged(self):
    '''
    If the content was changed, a '*' will be shown in the tab name.
    '''
    tab_name = self.__getTabName(self.tabWidget.currentWidget().filename)
    if (self.tabWidget.currentWidget().document().isModified()):
      tab_name = ''.join(['*', tab_name])
    self.tabWidget.setTabText(self.tabWidget.currentIndex(), tab_name)

  def on_editor_positionChanged(self):
    '''
    Shows the number of the line and column in a label.
    '''
    cursor = self.tabWidget.currentWidget().textCursor()
    self.pos_label.setText(''.join([str(cursor.blockNumber()+1), ':', str(cursor.columnNumber()+1)]))
  
  def on_shortcut_find(self):
    '''
    Opens a find dialog.
    '''
    self.find_dialog.show()
    self.find_dialog.raise_()
    self.find_dialog.activateWindow()

  def on_shortcut_goto(self):
    '''
    Opens a C{goto} dialog.
    '''
    value, ok = QtGui.QInputDialog.getInt(self, "Goto",
                                      self.tr("Line number:"), QtGui.QLineEdit.Normal,
                                      minValue=1, step=1)
    if ok:
      if value > self.tabWidget.currentWidget().document().blockCount():
        value = self.tabWidget.currentWidget().document().blockCount()
      curpos = self.tabWidget.currentWidget().textCursor().blockNumber()+1
      while curpos != value:
        mov = QtGui.QTextCursor.NextBlock if curpos < value else QtGui.QTextCursor.PreviousBlock
        self.tabWidget.currentWidget().moveCursor(mov)
        curpos = self.tabWidget.currentWidget().textCursor().blockNumber()+1

  def __getTabName(self, file):
    base = os.path.basename(file).replace('.launch', '')
    (package, path) = package_name(os.path.dirname(file))
    return ''.join([str(base), ' [', str(package),']'])

  def on_find_dialog_clicked(self, button):
    '''
    Method to handle the button actions of the C{find dialog}. 
    '''
    if button == self.find_dialog.find_button:
      self.find(self.find_dialog.search_field.text(), self.find_dialog.recursive.isChecked())
    elif button == self.find_dialog.replace_button:
      self.find_dialog.recursive.setChecked(False)
      cursor = self.tabWidget.currentWidget().textCursor()
      if self.find_dialog.search_field.text() and cursor.selectedText() == self.find_dialog.search_field.text():
        cursor.insertText(self.find_dialog.replace_field.text())
        currentLine = str(cursor.blockNumber()+1)
        self.find_dialog.result_label.setText(''.join(["'", self.find_dialog.search_text, "'", ' replaced at line: ', currentLine, ' by ', "'", self.find_dialog.replace_field.text(),"'"]))
        self.tabWidget.currentWidget().setTextCursor(cursor)
      self.find(self.find_dialog.search_field.text(), self.find_dialog.recursive.isChecked())

  def find(self, search_text, recursive):
    '''
    Searches for text in the current text editor. If `recursive` is C{True}, 
    the included files will be searched.
    @param search_text: text to find
    @type search_text: C{str}
    @param recursive: search in included files if this is C{True}
    @type recursive: C{bool}
    '''
    found = False
    if self.find_dialog.search_text != search_text:
      self.find_dialog.search_pos = QtGui.QTextCursor()
      self.find_dialog.found_files.clear()
      self.find_dialog.found_files.setVisible(False)
      self.find_dialog.result_label.setText(''.join(["'", search_text, "'", ' not found!']))
    self.find_dialog.search_text = search_text
    if search_text:
      if recursive:
        files = self.tabWidget.currentWidget().fileWithText(search_text)
        items = list(set(files))
        self.find_dialog.result_label.setText(''.join(["'", search_text, "'", ' found in ', str(len(items)), ' files:']))
        self.find_dialog.found_files.clear()
        self.find_dialog.found_files.addItems(items)
        self.find_dialog.found_files.setVisible(True)
        self.find_dialog.resize(self.find_dialog.found_files.contentsSize())
        found = True
      else:
        tmp_pos = self.find_dialog.search_pos
        self.find_dialog.search_pos = self.tabWidget.currentWidget().document().find(search_text, self.find_dialog.search_pos.position()+1)
        if self.find_dialog.search_pos.isNull() and not tmp_pos.isNull():
          self.find_dialog.search_pos = self.tabWidget.currentWidget().document().find(search_text, self.find_dialog.search_pos.position()+1)
          # do recursive search
        if not self.find_dialog.search_pos.isNull():
          self.tabWidget.currentWidget().setTextCursor(self.find_dialog.search_pos)
          currentTabName = self.tabWidget.tabText(self.tabWidget.currentIndex())
          currentLine = str(self.tabWidget.currentWidget().textCursor().blockNumber()+1)
          self.find_dialog.result_label.setText(''.join(["'", search_text, "'", ' found at line: ', currentLine, ' in ', "'", currentTabName,"'"]))
          found = True
        else:
          self.find_dialog.result_label.setText(''.join(["'", search_text, "'", ' not found!']))
    return found

  def find_dialog_itemActivated(self, item):
    '''
    On recursive search all files contained the search text are listed. If one of
    this file is activated, it will be open in a new tab and the cursor moved to
    the C{search text} position.
    @param item: The activated item of the C{QListWidget}
    @type item: L{PySide.QtGui.QListWidgetItem}
    '''
    self.find_dialog.recursive.setChecked(False)
    self.on_load_request(item.text(), self.find_dialog.search_text)
    self.find(self.find_dialog.search_text, False)
    currentTabName = self.tabWidget.tabText(self.tabWidget.currentIndex())
    currentLine = str(self.tabWidget.currentWidget().textCursor().blockNumber()+1)
    self.find_dialog.result_label.setText(''.join(["'", self.find_dialog.search_text, "'", ' found at line: ', currentLine, ' in ', "'", currentTabName,"'"]))


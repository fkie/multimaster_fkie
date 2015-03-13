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
from yaml_highlighter import YamlHighlighter
from detailed_msg_box import WarningMessageBox

import node_manager_fkie as nm
from master_discovery_fkie.common import resolve_url
from common import package_name
from run_dialog import PackageDialog

class Editor(QtGui.QTextEdit):
  '''
  The XML editor to handle the included files. If an included file in the opened
  launch file is detected, this can be open by STRG+(mouse click) in a new
  editor.
  '''

  load_request_signal = QtCore.Signal(str)
  ''' @ivar: A signal for request to open a configuration file'''

  SUBSTITUTION_ARGS = ['env', 'optenv', 'find', 'anon', 'arg']
  CONTEXT_FILE_EXT = ['.launch', '.test', '.xml']
  YAML_VALIDATION_FILES = ['.yaml', '.iface', '.sync']

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
                        QtCore.QRegExp("\\bfile\\b"), QtCore.QRegExp("\\bvalue=.*pkg:\/\/\\b"),
                        QtCore.QRegExp("\\bvalue=.*package:\/\/\\b"),
                        QtCore.QRegExp("\\bvalue=.*\$\(find\\b")]
    self.filename = filename
    self.file_info = None
    if self.filename:
      f = QtCore.QFile(filename);
      if f.open(QtCore.QIODevice.ReadOnly | QtCore.QIODevice.Text):
        self.file_info = QtCore.QFileInfo(filename)
        self.setText(unicode(f.readAll(), "utf-8"))

    self.path = '.'
    # enables drop events
    self.setAcceptDrops(True)
    if filename.endswith('.launch'):
      self.hl = XmlHighlighter(self.document())
    else:
      self.hl = YamlHighlighter(self.document())

#  def __del__(self):
#    print "********** desctroy:", self.objectName()

  def save(self, force=False):
    '''
    Saves changes to the file.
    :return: saved, errors, msg
    :rtype: bool, bool, str
    '''
    if force or self.document().isModified() or not QtCore.QFileInfo(self.filename).exists():
      f = QtCore.QFile(self.filename)
      if f.open(QtCore.QIODevice.WriteOnly | QtCore.QIODevice.Text):
        f.write(self.toPlainText().encode('utf-8'))
        self.document().setModified(False)
        self.file_info = QtCore.QFileInfo(self.filename)

        ext = os.path.splitext(self.filename)
        # validate the xml structure of the launch files
        if ext[1] in self.CONTEXT_FILE_EXT:
          imported = False
          try:
            from lxml import etree
            imported = True
            parser = etree.XMLParser()
            etree.fromstring(self.toPlainText().encode('utf-8'), parser)
          except Exception as e:
            if imported:
              self.markLine(e.position[0])
              return True, True, "%s"%e
        # validate the yaml structure of yaml files
        elif ext[1] in self.YAML_VALIDATION_FILES:
          try:
            import yaml
            yaml.load(self.toPlainText().encode('utf-8'))
          except yaml.MarkedYAMLError as e:
            return True, True, "%s"%e
        return True, False, ''
      else:
        return False, True, "Cannot write XML file"
    return False, False, ''

  def markLine(self, no):
    try:
      cursor = self.textCursor()
      cursor.setPosition(0, QtGui.QTextCursor.MoveAnchor)
      while (cursor.block().blockNumber()+1 < no):
        cursor.movePosition(QtGui.QTextCursor.NextBlock, QtGui.QTextCursor.MoveAnchor)
      cursor.movePosition(QtGui.QTextCursor.EndOfBlock, QtGui.QTextCursor.KeepAnchor)
      self.setTextCursor(cursor)
    except:
      pass

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
          try:
            pkg = roslib.packages.get_pkg_dir(script[1])
            return os.path.normpath(''.join([pkg, '/', path[endIndex+1:]]))
          except Exception as e:
            rospy.logwarn(str(e))
    else:
      try:
        return resolve_url(path)
      except ValueError, e:
        if len(path) > 0 and path[0] != '/':
          return os.path.normpath(''.join([self.path, '/', path]))
#    elif len(path) > 0 and path[0] != '/':
#      return os.path.normpath(''.join([self.path, '/', path]))
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
            try:
              path = self.interpretPath(fileName)
              f = QtCore.QFile(path)
              ext = os.path.splitext(path)
              if f.exists() and ext[1] in nm.settings().SEARCH_IN_EXT:
                result.append(path)
            except:
              import traceback
              print traceback.format_exc(1)
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

  def focusInEvent(self, event):
    # check for file changes
    try:
      if self.filename and self.file_info:
        if self.file_info.lastModified() != QtCore.QFileInfo(self.filename).lastModified():
          self.file_info = QtCore.QFileInfo(self.filename)
          result = QtGui.QMessageBox.question(self, "File changed", "File was changed, reload?", QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
          if result == QtGui.QMessageBox.Yes:
            f = QtCore.QFile(self.filename);
            if f.open(QtCore.QIODevice.ReadOnly | QtCore.QIODevice.Text):
              self.setText(unicode(f.readAll(), "utf-8"))
              self.document().setModified(False)
              self.textChanged.emit()
            else:
              QtGui.QMessageBox.critical(self, "Error", "Cannot open launch file%s"%self.filename)
    except:
      pass
    QtGui.QTextEdit.focusInEvent(self, event)

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
            try:
              qf = QtCore.QFile(self.interpretPath(fileName))
              if not qf.exists():
                # create a new file, if it does not exists
                result = QtGui.QMessageBox.question(self, "File not found", '\n\n'.join(["Create a new file?", qf.fileName()]), QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
                if result == QtGui.QMessageBox.Yes:
                  d = os.path.dirname(qf.fileName())
                  if not os.path.exists(d):
                    os.makedirs(d)
                  with open(qf.fileName(),'w') as f:
                    if qf.fileName().endswith('.launch'):
                      f.write('<launch>\n\n</launch>')
                  self.load_request_signal.emit(qf.fileName())
              else:
                self.load_request_signal.emit(qf.fileName())
            except Exception, e:
              WarningMessageBox(QtGui.QMessageBox.Warning, "File not found %s"%fileName, str(e)).exec_()
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
    if event.modifiers() == QtCore.Qt.ControlModifier and event.key() == QtCore.Qt.Key_7:
      self.commentText()
    elif event.modifiers() == QtCore.Qt.AltModifier and event.key() == QtCore.Qt.Key_Space:
      ext = os.path.splitext(self.filename)
      if ext[1] in self.CONTEXT_FILE_EXT:
        menu = self._create_context_substitution_menu()
        if menu is None:
          menu = self._create_context_tag_menu()
        if menu:
          menu.exec_(self.mapToGlobal(self.cursorRect().bottomRight()))
    elif event.key() != QtCore.Qt.Key_Escape:
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
    else:
      event.accept()
      QtGui.QTextEdit.keyReleaseEvent(self, event)

  def commentText(self):
    cursor = self.textCursor()
    if not cursor.isNull():
      cursor.beginEditBlock()
      start = cursor.selectionStart()
      end = cursor.selectionEnd()
      cursor.setPosition(start)
      block_start = cursor.blockNumber()
      cursor.setPosition(end)
      block_end = cursor.blockNumber()
      if block_end-block_start > 0 and end-cursor.block().position() <= 0:
        # skip the last block, if no characters are selected
        block_end -= 1
      cursor.setPosition(start, QtGui.QTextCursor.MoveAnchor)
      cursor.movePosition(QtGui.QTextCursor.StartOfLine)
      start = cursor.position()
      while (cursor.block().blockNumber() < block_end+1):
        cursor.movePosition(QtGui.QTextCursor.StartOfLine)
        ext = os.path.splitext(self.filename)
        # XML comment
        if ext[1] in self.CONTEXT_FILE_EXT:
          if cursor.block().length() < 4:
            cursor.movePosition(QtGui.QTextCursor.NextBlock)
            continue
          cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, 4)
          # only comments breakers at the start of the line are removed 
          if cursor.selectedText() == '<!--':
            cursor.insertText('')
            cursor.movePosition(QtGui.QTextCursor.EndOfLine)
            cursor.movePosition(QtGui.QTextCursor.PreviousCharacter, QtGui.QTextCursor.KeepAnchor, 3)
            if cursor.selectedText() == '-->':
              cursor.insertText('')
          else:
            cursor.movePosition(QtGui.QTextCursor.StartOfLine)
            cursor.movePosition(QtGui.QTextCursor.EndOfLine, QtGui.QTextCursor.KeepAnchor)
            # only comment out, if no comments are found
            if cursor.selectedText().find('<!--') < 0 and cursor.selectedText().find('-->') < 0:
              cursor.movePosition(QtGui.QTextCursor.StartOfLine)
              cursor.insertText('<!--')
              cursor.movePosition(QtGui.QTextCursor.EndOfLine)
              cursor.insertText('-->')
        else: # other comments
          if cursor.block().length() < 2:
            cursor.movePosition(QtGui.QTextCursor.NextBlock)
            continue
          cursor.movePosition(QtGui.QTextCursor.NextCharacter, QtGui.QTextCursor.KeepAnchor, 2)
          # only comments breakers at the start of the line are removed
          if cursor.selectedText() == '# ':
            cursor.insertText('')
          else:
            cursor.movePosition(QtGui.QTextCursor.StartOfLine)
            cursor.insertText('# ')
        cursor.movePosition(QtGui.QTextCursor.NextBlock)
      # Set our cursor's selection to span all of the involved lines.
      cursor.endEditBlock()
      cursor.setPosition(start, QtGui.QTextCursor.MoveAnchor)
      cursor.movePosition(QtGui.QTextCursor.StartOfBlock, QtGui.QTextCursor.MoveAnchor)
      while (cursor.block().blockNumber() < block_end):
        cursor.movePosition(QtGui.QTextCursor.NextBlock, QtGui.QTextCursor.KeepAnchor)
      cursor.movePosition(QtGui.QTextCursor.EndOfBlock, QtGui.QTextCursor.KeepAnchor)
      # set the cursor 
      self.setTextCursor(cursor)

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
        if key_mod & QtCore.Qt.ControlModifier or key_mod & QtCore.Qt.ShiftModifier:
          for _ in range(2):
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
        if key_mod & QtCore.Qt.ControlModifier or key_mod & QtCore.Qt.ShiftModifier:
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

  #############################################################################
  ########## Drag&Drop                                                   ###### 
  #############################################################################

  def dragEnterEvent(self, e):
    if e.mimeData().hasFormat('text/plain'):
      e.accept()
    else:
      e.ignore()

  def dragMoveEvent(self, e):
    e.accept()

  def dropEvent(self, e):
    cursor = self.cursorForPosition(e.pos())
    if not cursor.isNull():
      text = e.mimeData().text()
      # the files will be included 
      if text.startswith('file://'):
        text = text[7:]
      if os.path.exists(text) and os.path.isfile(text):
        # find the package name containing the included file 
        (package, path) = package_name(os.path.dirname(text))
        if text.endswith('.launch'):
          if package:
            cursor.insertText('<include file="$(find %s)%s" />'%(package, text.replace(path, '')))
          else:
            cursor.insertText('<include file="%s" />'%text)
        else:
          if package:
            cursor.insertText('<rosparam file="$(find %s)%s" command="load" />'%(package, text.replace(path, '')))
          else:
            cursor.insertText('<rosparam file="%s" command="load" />'%text)
      else:
        cursor.insertText(e.mimeData().text())
    e.accept()

  #############################################################################
  ########## Ctrl&Space  Context menu                                    ###### 
  #############################################################################

  def _create_context_tag_menu(self):
    parent_tag, inblock, attrs = self._get_parent_tag()
    if not parent_tag:
      return None
    menu = QtGui.QMenu(self)
    menu.triggered.connect(self._context_activated)
    text = self.toPlainText()
    pos = self.textCursor().position() - 1
    try:
      if not inblock:
        # create a menu with attributes
        attributes = sorted(list((set(XmlHighlighter.LAUNCH_ATTR[parent_tag]) - set(attrs))))
        for attr in attributes:
          action = menu.addAction(attr.rstrip('='))
          action.setData('%s"'%attr if text[pos] == ' ' else ' %s"'%attr)
      else:
        # create a menu with tags
        tags = sorted(XmlHighlighter.LAUNCH_CHILDS[parent_tag])
        if not tags:
          return None
        for tag in tags:
          data = '<%s></%s>'%(tag, tag) if XmlHighlighter.LAUNCH_CHILDS[tag] else '<%s/>'%tag
          if text[pos] == '<':
            data = data[1:]
          action = menu.addAction(tag)
          action.setData(data)
    except:
#      import traceback
#      print traceback.format_exc(1)
      return None
    return menu

  def _create_context_substitution_menu(self):
    text = self.toPlainText()
    pos = self.textCursor().position() - 1
    try:
      if text[pos] == '$' or (text[pos] == '(' and text[pos-1] == '$'):
        menu = QtGui.QMenu(self)
        menu.triggered.connect(self._context_activated)
        for arg in self.SUBSTITUTION_ARGS:
          action = menu.addAction("%s"%arg)
          action.setData("(%s"%arg if text[pos] == '$' else "%s"%arg)
        return menu
    except:
      pass
    return None

  def _get_parent_tag(self):
    text = self.toPlainText()
    pos = self.textCursor().position() - 1
    # do not parse, if the menu was requested in a string sequence
    try:
      if not (text[pos] in [' ', '<', '>', '"', '\n'] or text[pos+1] in [' ', '<','"', '\n']):
        return '', False, []
    except:
      pass
    instr = (text[:pos+1].count('"') % 2)
    # do not parse, if the menu was requested in a string definition
    if instr:
      return '', False, []
    # some parameter definition
    closed_tags = []
    current_attr = []
    tag_reading = True
    tag = ''
    attr_reading = False
    attr = ''
    closed_gts = False
    # parse the text from current position to the beginning
    i = pos
    while i >= 0:
      if text[i] == '"':
        instr = not instr
      elif not instr:
        # parse only text which is not in string definitions
        if text[i] == '=':
          attr_reading = True
        elif text[i] in ['<', '/', '>']:
          if text[i] == '>':
            closed_gts = True
            tag = ''
          elif text[i] == '/' and closed_gts:
            closed_gts = False
            closed_tags.append(tag if tag else '/')
            tag = ''
          elif text[i] == '<':
            if closed_tags and (tag == closed_tags[-1] or closed_tags[-1] == '/'):
              closed_tags.pop()
              current_attr = []
              tag = ''
            elif tag:
              return tag[::-1], closed_gts, current_attr # reverse the tag
        # start or end of attribute parsing
        elif text[i] == ' ':
          if attr_reading and attr:
            current_attr.append("%s="%attr[::-1])
            attr_reading = False
          attr = ''
          tag = ''
          tag_reading = True
        else:
          if tag_reading or closed_gts:
            tag += text[i]
          if attr_reading:
            attr += text[i]
      i -= 1
    return '', False, []

  def _context_activated(self, arg):
    cursor = self.textCursor()
    if not cursor.isNull():
      cursor.insertText(arg.data())


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
    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_edit_launch.png")
    self._error_icon = QtGui.QIcon(":/icons/crystal_clear_warning.png")
    self._empty_icon = QtGui.QIcon()
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
    # add the goto button
    self.gotoButton = QtGui.QPushButton(self)
    self.gotoButton.setObjectName("gotoButton")
    self.gotoButton.clicked.connect(self.on_shortcut_goto)
    self.gotoButton.setText(QtGui.QApplication.translate("XmlEditor", "Goto line", None, QtGui.QApplication.UnicodeUTF8))
    self.gotoButton.setShortcut(QtGui.QApplication.translate("XmlEditor", "Ctrl+L", None, QtGui.QApplication.UnicodeUTF8))
    self.gotoButton.setToolTip('Open a goto dialog (Ctrl+L)')
    self.horizontalLayout.addWidget(self.gotoButton)
    # add a tag button
    self.tagButton = self._create_tag_button(self)
    self.horizontalLayout.addWidget(self.tagButton)

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

    self.readSettings()
#    print "================ create", self.objectName()
#
#  def __del__(self):
#    print "******** destroy", self.objectName()
  def readSettings(self):
    if nm.settings().store_geometry:
      settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
      settings.beginGroup("editor")
      maximized = settings.value("maximized", 'false') == 'true'
      if maximized:
        self.showMaximized()
      else:
        self.resize(settings.value("size", QtCore.QSize(800,640)))
        self.move(settings.value("pos", QtCore.QPoint(0, 0)))
      settings.endGroup()

  def storeSetting(self):
    if nm.settings().store_geometry:
      settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
      settings.beginGroup("editor")
      settings.setValue("size", self.size())
      settings.setValue("pos", self.pos())
      settings.setValue("maximized", self.isMaximized())
      settings.endGroup()

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
      rospy.logwarn("Error while open %s: %s", filename, traceback.format_exc(1))
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
      rospy.logwarn("Error while close tab %s: %s", str(tab_index), traceback.format_exc(1))

#  def hideEvent(self, event):
#    self.close()

  def reject(self):
    self.close()

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
      self.storeSetting()
      self.finished_signal.emit(self.init_filenames)

  def on_saveButton_clicked(self):
    '''
    Saves the current document. This method is called if the C{save button} 
    was clicked.
    '''
    saved, errors, msg = self.tabWidget.currentWidget().save(True)
    if errors:
      QtGui.QMessageBox.critical(self, "Error", msg)
      self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._error_icon)
      self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), msg)
    elif saved:
      self.tabWidget.setTabIcon(self.tabWidget.currentIndex(), self._empty_icon)
      self.tabWidget.setTabToolTip(self.tabWidget.currentIndex(), '')
      self.on_editor_textChanged()

  def on_editor_textChanged(self):
    '''
    If the content was changed, a '*' will be shown in the tab name.
    '''
    tab_name = self.__getTabName(self.tabWidget.currentWidget().filename)
    if (self.tabWidget.currentWidget().document().isModified()) or not QtCore.QFileInfo(self.tabWidget.currentWidget().filename).exists():
      tab_name = ''.join(['*', tab_name])
    self.tabWidget.setTabText(self.tabWidget.currentIndex(), tab_name)

  def on_editor_positionChanged(self):
    '''
    Shows the number of the line and column in a label.
    '''
    cursor = self.tabWidget.currentWidget().textCursor()
    self.pos_label.setText('%s:%s'%(cursor.blockNumber()+1, cursor.columnNumber()))

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
    value = 1
    ok = False
    try:
      value, ok = QtGui.QInputDialog.getInt(self, "Goto",
                                        self.tr("Line number:"), QtGui.QLineEdit.Normal,
                                        minValue=1, step=1)
    except:
      value, ok = QtGui.QInputDialog.getInt(self, "Goto",
                                  self.tr("Line number:"), QtGui.QLineEdit.Normal,
                                  min=1, step=1)
    if ok:
      if value > self.tabWidget.currentWidget().document().blockCount():
        value = self.tabWidget.currentWidget().document().blockCount()
      curpos = self.tabWidget.currentWidget().textCursor().blockNumber()+1
      while curpos != value:
        mov = QtGui.QTextCursor.NextBlock if curpos < value else QtGui.QTextCursor.PreviousBlock
        self.tabWidget.currentWidget().moveCursor(mov)
        curpos = self.tabWidget.currentWidget().textCursor().blockNumber()+1
    self.tabWidget.currentWidget().setFocus(QtCore.Qt.ActiveWindowFocusReason)

  def __getTabName(self, lfile):
    base = os.path.basename(lfile).replace('.launch', '')
    (package, _) = package_name(os.path.dirname(lfile))
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
#        self.find_dialog.resize(self.find_dialog.found_files.contentsSize())
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

  ##############################################################################
  # LAUNCH TAG insertion 
  ##############################################################################

  def _create_tag_button(self, parent=None):
    btn = QtGui.QPushButton(parent)
    btn.setObjectName("tagButton")
    btn.setText(QtGui.QApplication.translate("XmlEditor", "Add tag", None, QtGui.QApplication.UnicodeUTF8))
    btn.setShortcut(QtGui.QApplication.translate("XmlEditor", "Ctrl+T", None, QtGui.QApplication.UnicodeUTF8))
    btn.setToolTip('Adds a ROS launch tag to launch file (Ctrl+T)')
    # creates a tag menu
    tag_menu = QtGui.QMenu(btn)
    # group tag
    add_group_tag_action = QtGui.QAction("<group>", self, statusTip="", triggered=self._on_add_group_tag)
    tag_menu.addAction(add_group_tag_action)
    # node tag
    add_node_tag_action = QtGui.QAction("<node>", self, statusTip="", triggered=self._on_add_node_tag)
    tag_menu.addAction(add_node_tag_action)
    # node tag with all attributes
    add_node_tag_all_action = QtGui.QAction("<node all>", self, statusTip="", triggered=self._on_add_node_tag_all)
    tag_menu.addAction(add_node_tag_all_action)
    # include tag with all attributes
    add_include_tag_all_action = QtGui.QAction("<include>", self, statusTip="", triggered=self._on_add_include_tag_all)
    tag_menu.addAction(add_include_tag_all_action)
    # remap
    add_remap_tag_action = QtGui.QAction("<remap>", self, statusTip="", triggered=self._on_add_remap_tag)
    tag_menu.addAction(add_remap_tag_action)
    # env tag
    add_env_tag_action = QtGui.QAction("<env>", self, statusTip="", triggered=self._on_add_env_tag)
    tag_menu.addAction(add_env_tag_action)
    # param tag
    add_param_tag_action = QtGui.QAction("<param>", self, statusTip="", triggered=self._on_add_param_tag)
    tag_menu.addAction(add_param_tag_action)
    # param capability group tag
    add_param_cap_group_tag_action = QtGui.QAction("<param capability group>", self, statusTip="", triggered=self._on_add_param_cap_group_tag)
    tag_menu.addAction(add_param_cap_group_tag_action)
    # param tag with all attributes
    add_param_tag_all_action = QtGui.QAction("<param all>", self, statusTip="", triggered=self._on_add_param_tag_all)
    tag_menu.addAction(add_param_tag_all_action)
    # rosparam tag with all attributes
    add_rosparam_tag_all_action = QtGui.QAction("<rosparam>", self, statusTip="", triggered=self._on_add_rosparam_tag_all)
    tag_menu.addAction(add_rosparam_tag_all_action)
    # arg tag with default definition
    add_arg_tag_default_action = QtGui.QAction("<arg default>", self, statusTip="", triggered=self._on_add_arg_tag_default)
    tag_menu.addAction(add_arg_tag_default_action)
    # arg tag with value definition
    add_arg_tag_value_action = QtGui.QAction("<arg value>", self, statusTip="", triggered=self._on_add_arg_tag_value)
    tag_menu.addAction(add_arg_tag_value_action)

    # test tag
    add_test_tag_action = QtGui.QAction("<test>", self, statusTip="", triggered=self._on_add_test_tag)
    tag_menu.addAction(add_test_tag_action)
    # test tag with all attributes
    add_test_tag_all_action = QtGui.QAction("<test all>", self, statusTip="", triggered=self._on_add_test_tag_all)
    tag_menu.addAction(add_test_tag_all_action)


    btn.setMenu(tag_menu)
    return btn

  def _insert_text(self, text):
    cursor = self.tabWidget.currentWidget().textCursor()
    if not cursor.isNull():
      col = cursor.columnNumber()
      spaces = ''.join([' ' for _ in range(col)])
      cursor.insertText(text.replace('\n','\n%s'%spaces))
      self.tabWidget.currentWidget().setFocus(QtCore.Qt.OtherFocusReason)

  def _on_add_group_tag(self):
    self._insert_text('<group ns="namespace" clear_params="true|false">\n'
                      '</group>')

  def _on_add_node_tag(self):
    dia = PackageDialog()
    if dia.exec_():
      self._insert_text('<node name="%s" pkg="%s" type="%s">\n'
                        '</node>'%(dia.binary, dia.package, dia.binary))

  def _on_add_node_tag_all(self):
    dia = PackageDialog()
    if dia.exec_():
      self._insert_text('<node name="%s" pkg="%s" type="%s"\n'
                        '      args="arg1" machine="machine-name"\n'
                        '      respawn="true" required="true"\n'
                        '      ns="foo" clear_params="true|false"\n'
                        '      output="log|screen" cwd="ROS_HOME|node"\n'
                        '      launch-prefix="prefix arguments">\n'
                        '</node>'%(dia.binary, dia.package, dia.binary))

  def _on_add_include_tag_all(self):
    self._insert_text('<include file="$(find pkg-name)/path/filename.xml"\n'
                      '         ns="foo" clear_params="true|false">\n'
                      '</include>')

  def _on_add_remap_tag(self):
    self._insert_text('<remap from="original" to="new"/>')

  def _on_add_env_tag(self):
    self._insert_text('<env name="variable" value="value"/>')

  def _on_add_param_tag(self):
    self._insert_text('<param name="namespace/name" value="value" />')

  def _on_add_param_cap_group_tag(self):
    self._insert_text('<param name="capability_group" value="demo" />')

  def _on_add_param_tag_all(self):
    self._insert_text('<param name="namespace/name" value="value"\n'
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
                        '</test>'%(dia.binary, dia.package, dia.binary, dia.binary))

  def _on_add_test_tag_all(self):
    dia = PackageDialog()
    if dia.exec_():
      self._insert_text('<test name="%s" pkg="%s" type="%s" test-name="test_%s">\n'
                        '      args="arg1" time-limit="60.0"\n'
                        '      ns="foo" clear_params="true|false"\n'
                        '      cwd="ROS_HOME|node" retry="0"\n'
                        '      launch-prefix="prefix arguments">\n'
                        '</test>'%(dia.binary, dia.package, dia.binary, dia.binary))

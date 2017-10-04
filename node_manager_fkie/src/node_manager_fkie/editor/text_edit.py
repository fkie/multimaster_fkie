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

from python_qt_binding.QtCore import QFile, QFileInfo, QIODevice, QRegExp, Qt, Signal
from python_qt_binding.QtGui import QFont, QTextCursor
import os
import re

from node_manager_fkie.common import package_name, utf8
from node_manager_fkie.detailed_msg_box import MessageBox
from node_manager_fkie.launch_config import LaunchConfig
import node_manager_fkie as nm

from .parser_functions import interpret_path
from .xml_highlighter import XmlHighlighter
from .yaml_highlighter import YamlHighlighter


try:
    from python_qt_binding.QtGui import QApplication, QMenu, QTextEdit
except:
    from python_qt_binding.QtWidgets import QApplication, QMenu, QTextEdit


class TextEdit(QTextEdit):
    '''
    The XML editor to handle the included files. If an included file in the opened
    launch file is detected, this can be open by STRG+(mouse click) in a new
    editor.
    '''

    load_request_signal = Signal(str)
    ''' @ivar: A signal for request to open a configuration file'''

    search_result_signal = Signal(str, bool, str, int)
    ''' @ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, position in text)
        for each result a signal will be emitted.
    '''

    SUBSTITUTION_ARGS = ['env', 'optenv', 'find', 'anon', 'arg']
    CONTEXT_FILE_EXT = ['.launch', '.test', '.xml']
    YAML_VALIDATION_FILES = ['.yaml', '.iface', '.sync']

    def __init__(self, filename, parent=None):
        self.parent = parent
        QTextEdit.__init__(self, parent)
        self.setObjectName(' - '.join(['Editor', filename]))
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_custom_context_menu)
#        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.setAcceptRichText(False)
        font = QFont()
        font.setFamily("Fixed".decode("utf-8"))
        font.setPointSize(12)
        self.setFont(font)
        self.setLineWrapMode(QTextEdit.NoWrap)
        self.setTabStopWidth(25)
        self.setAcceptRichText(False)
        self.setCursorWidth(2)
        self.setFontFamily("courier new")
        self.setProperty("backgroundVisible", True)
        self.regexp_list = [QRegExp("\\binclude\\b"), QRegExp("\\btextfile\\b"),
                            QRegExp("\\bfile\\b"), QRegExp("\\bvalue=.*pkg:\/\/\\b"),
                            QRegExp("\\bvalue=.*package:\/\/\\b"),
                            QRegExp("\\bvalue=.*\$\(find\\b"),
                            QRegExp("\\bargs=.*\$\(find\\b"),
                            QRegExp("\\bdefault=.*\$\(find\\b")]
        self.filename = filename
        self.file_info = None
        if self.filename:
            f = QFile(filename)
            if f.open(QIODevice.ReadOnly | QIODevice.Text):
                self.file_info = QFileInfo(filename)
                self.setText(unicode(f.readAll(), "utf-8"))

        self.path = '.'
        # enables drop events
        self.setAcceptDrops(True)
        if filename.endswith('.launch'):
            self.hl = XmlHighlighter(self.document())
            self.cursorPositionChanged.connect(self._document_position_changed)
        else:
            self.hl = YamlHighlighter(self.document())
        # variables for threaded search
        self._search_thread = None
        self._stop = False

    def _document_position_changed(self):
        if isinstance(self.hl, XmlHighlighter) and nm.settings().highlight_xml_blocks:
            #            import time
            #            start_time = time.time()
            self.hl.mark_block(self.textCursor().block(), self.textCursor().positionInBlock())
            #            print("--- mark_tag_block %.6f seconds ---" % (time.time() - start_time))

    def save(self, force=False):
        '''
        Saves changes to the file.
        :return: saved, errors, msg
        :rtype: bool, bool, str
        '''
        if force or self.document().isModified() or not QFileInfo(self.filename).exists():
            f = QFile(self.filename)
            if f.open(QIODevice.WriteOnly | QIODevice.Text):
                f.write(self.toPlainText().encode('utf-8'))
                self.document().setModified(False)
                self.file_info = QFileInfo(self.filename)

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
                            return True, True, "%s" % e
                # validate the yaml structure of yaml files
                elif ext[1] in self.YAML_VALIDATION_FILES:
                    try:
                        import yaml
                        yaml.load(self.toPlainText().encode('utf-8'))
                    except yaml.MarkedYAMLError as e:
                        return True, True, "%s" % e
                return True, False, ''
            else:
                return False, True, "Cannot write XML file"
        return False, False, ''

    def markLine(self, no):
        try:
            cursor = self.textCursor()
            cursor.setPosition(0, QTextCursor.MoveAnchor)
            while (cursor.block().blockNumber() + 1 < no):
                cursor.movePosition(QTextCursor.NextBlock, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            self.setTextCursor(cursor)
        except:
            pass

    def setCurrentPath(self, path):
        '''
        Sets the current working path. This path is to open the included files,
        which contains the relative path.
        @param path: the path of the current opened file (without the file)
        @type path: C{str}
        '''
        self.path = path

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
                    endIndex = text.find('"', startIndex + 1)
                    fileName = text[startIndex + 1:endIndex]
                    if len(fileName) > 0:
                        try:
                            path = interpret_path(fileName)
                            f = QFile(path)
                            ext = os.path.splitext(path)
                            if f.exists() and ext[1] in nm.settings().SEARCH_IN_EXT:
                                result.append(path)
                        except:
                            import traceback
                            print traceback.format_exc(1)
            b = b.next()
        return result

    def focusInEvent(self, event):
        # check for file changes
        try:
            if self.filename and self.file_info:
                if self.file_info.lastModified() != QFileInfo(self.filename).lastModified():
                    self.file_info = QFileInfo(self.filename)
                    result = MessageBox.question(self, "File changed", "File was changed, reload?", buttons=MessageBox.Yes | MessageBox.No)
                    if result == MessageBox.Yes:
                        f = QFile(self.filename)
                        if f.open(QIODevice.ReadOnly | QIODevice.Text):
                            self.setText(unicode(f.readAll(), "utf-8"))
                            self.document().setModified(False)
                            self.textChanged.emit()
                        else:
                            MessageBox.critical(self, "Error", "Cannot open launch file%s" % self.filename)
        except:
            pass
        QTextEdit.focusInEvent(self, event)

    def mouseReleaseEvent(self, event):
        '''
        Opens the new editor, if the user clicked on the included file and sets the
        default cursor.
        '''
        if event.modifiers() == Qt.ControlModifier or event.modifiers() == Qt.ShiftModifier:
            cursor = self.cursorForPosition(event.pos())
            inc_files = LaunchConfig.included_files(cursor.block().text(), recursive=False)
            if inc_files:
                try:
                    qf = QFile(inc_files[0])
                    if not qf.exists():
                        # create a new file, if it does not exists
                        result = MessageBox.question(self, "File not found", '\n\n'.join(["Create a new file?", qf.fileName()]), buttons=MessageBox.Yes | MessageBox.No)
                        if result == MessageBox.Yes:
                            d = os.path.dirname(qf.fileName())
                            if not os.path.exists(d):
                                os.makedirs(d)
                            with open(qf.fileName(), 'w') as f:
                                if qf.fileName().endswith('.launch'):
                                    f.write('<launch>\n\n</launch>')
                            event.setAccepted(True)
                            self.load_request_signal.emit(qf.fileName())
                    else:
                        event.setAccepted(True)
                        self.load_request_signal.emit(qf.fileName())
                except Exception, e:
                    MessageBox.critical(self, "Error", "File not found %s" % inc_files[0], detailed_text=utf8(e))
        QTextEdit.mouseReleaseEvent(self, event)

    def mouseMoveEvent(self, event):
        '''
        Sets the X{Qt.PointingHandCursor} if the control key is pressed and
        the mouse is over the included file.
        '''
        if event.modifiers() == Qt.ControlModifier or event.modifiers() == Qt.ShiftModifier:
            cursor = self.cursorForPosition(event.pos())
            index = self.index(cursor.block().text())
            if index > -1:
                self.viewport().setCursor(Qt.PointingHandCursor)
            else:
                self.viewport().setCursor(Qt.IBeamCursor)
        else:
            self.viewport().setCursor(Qt.IBeamCursor)
        QTextEdit.mouseMoveEvent(self, event)

    def keyPressEvent(self, event):
        '''
        Enable the mouse tracking by X{setMouseTracking()} if the control key is pressed.
        '''
        if event.key() == Qt.Key_Control or event.key() == Qt.Key_Shift:
            self.setMouseTracking(True)
        if event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_7:
            self.commentText()
        elif event.modifiers() == Qt.ControlModifier | Qt.ShiftModifier and event.key() == Qt.Key_Slash:
            self.commentText()
        elif event.modifiers() == Qt.AltModifier and event.key() == Qt.Key_Space:
            ext = os.path.splitext(self.filename)
            if ext[1] in self.CONTEXT_FILE_EXT:
                menu = self._create_context_substitution_menu(False)
                if menu is None:
                    menu = self._create_context_menu_for_tag()
                if menu:
                    menu.exec_(self.mapToGlobal(self.cursorRect().bottomRight()))
        elif event.key() != Qt.Key_Escape:
            # handle the shifting of the block
            if event.modifiers() == Qt.NoModifier and event.key() == Qt.Key_Tab:
                self.shiftText()
            elif event.modifiers() == Qt.ShiftModifier and event.key() == Qt.Key_Backtab:
                self.shiftText(back=True)
            else:
                event.accept()
                if event.key() in [Qt.Key_Enter, Qt.Key_Return]:
                    ident = self.getIdentOfCurretLine()
                QTextEdit.keyPressEvent(self, event)
                if event.key() in [Qt.Key_Enter, Qt.Key_Return]:
                    self.indentCurrentLine(ident)
        else:
            event.accept()
            QTextEdit.keyPressEvent(self, event)

    def keyReleaseEvent(self, event):
        '''
        Disable the mouse tracking by X{setMouseTracking()} if the control key is
        released and set the cursor back to X{Qt.IBeamCursor}.
        '''
        if event.key() == Qt.Key_Control or event.key() == Qt.Key_Shift:
            self.setMouseTracking(False)
            self.viewport().setCursor(Qt.IBeamCursor)
        else:
            event.accept()
            QTextEdit.keyReleaseEvent(self, event)

    def _has_uncommented(self):
        cursor = QTextCursor(self.textCursor())
        if not cursor.isNull():
            start = cursor.selectionStart()
            end = cursor.selectionEnd()
            cursor.setPosition(start)
            block_start = cursor.blockNumber()
            cursor.setPosition(end)
            block_end = cursor.blockNumber()
            if block_end - block_start > 0 and end - cursor.block().position() <= 0:
                # skip the last block, if no characters are selected
                block_end -= 1
            cursor.setPosition(start, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.StartOfLine)
            start = cursor.position()
            xmlre = re.compile(r"\A\s*<!--")
            otherre = re.compile(r"\A\s*#")
            ext = os.path.splitext(self.filename)
            # XML comment
            xml_file = ext[1] in self.CONTEXT_FILE_EXT
            while (cursor.block().blockNumber() < block_end + 1):
                cursor.movePosition(QTextCursor.StartOfLine)
                cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
                if xml_file:
                    if not xmlre.match(cursor.selectedText()):
                        return True
                else:
                    if not otherre.match(cursor.selectedText()):
                        return True
                cursor.movePosition(QTextCursor.NextBlock)
        return False

    def commentText(self):
        do_comment = self._has_uncommented()
        cursor = self.textCursor()
        if not cursor.isNull():
            cursor.beginEditBlock()
            start = cursor.selectionStart()
            end = cursor.selectionEnd()
            cursor.setPosition(start)
            block_start = cursor.blockNumber()
            cursor.setPosition(end)
            block_end = cursor.blockNumber()
            if block_end - block_start > 0 and end - cursor.block().position() <= 0:
                # skip the last block, if no characters are selected
                block_end -= 1
            cursor.setPosition(start, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.StartOfLine)
            start = cursor.position()
            ext = os.path.splitext(self.filename)
            # XML comment
            xml_file = ext[1] in self.CONTEXT_FILE_EXT
            while (cursor.block().blockNumber() < block_end + 1):
                cursor.movePosition(QTextCursor.StartOfLine)
                # XML comment
                if xml_file:
                    xmlre_start = re.compile(r"<!-- ?")
                    xmlre_end = re.compile(r" ?-->")
                    cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
                    if do_comment:
                        cursor.insertText("<!-- %s -->" % cursor.selectedText().replace("--", "- -"))
                    else:
                        res = cursor.selectedText()
                        mstart = xmlre_start.search(res)
                        if mstart:
                            res = res.replace(mstart.group(), "", 1)
                            res = res.replace("<!- -", "<!--", 1)
                        mend = xmlre_end.search(res)
                        if mend:
                            res = res.replace(mend.group(), "", 1)
                            last_pos = res.rfind("- ->")
                            if last_pos > -1:
                                res = "%s-->" % res[0:last_pos]
                        cursor.insertText(res)
                else:  # other comments
                    hash_re = re.compile(r"# ?")
                    if do_comment:
                        cursor.insertText('# ')
                    else:
                        cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
                        res = cursor.selectedText()
                        hres = hash_re.search(res)
                        if hres:
                            res = res.replace(hres.group(), "", 1)
                        cursor.insertText(res)
                cursor.movePosition(QTextCursor.NextBlock)
            # Set our cursor's selection to span all of the involved lines.
            cursor.endEditBlock()
            cursor.setPosition(start, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.StartOfBlock, QTextCursor.MoveAnchor)
            while (cursor.block().blockNumber() < block_end):
                cursor.movePosition(QTextCursor.NextBlock, QTextCursor.KeepAnchor)
            cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            # set the cursor
            self.setTextCursor(cursor)

    def shiftText(self, back=False):
        '''
        Increase (Decrease) indentation using Tab (Ctrl+Tab).
        '''
        cursor = self.textCursor()
        if not cursor.isNull():
            # one undo operation
            cursor.beginEditBlock()
            start = cursor.selectionStart()
            end = cursor.selectionEnd()
            cursor.setPosition(start)
            block_start = cursor.blockNumber()
            cursor.setPosition(end)
            block_end = cursor.blockNumber()
            if block_end - block_start == 0:
                # shift one line two spaces to the left
                if back:
                    for _ in range(2):
                        cursor.movePosition(QTextCursor.StartOfLine)
                        cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, 1)
                        if cursor.selectedText() == ' ':
                            cursor.insertText('')
                        elif cursor.selectedText() == "\t":
                            cursor.insertText('')
                            break
                    cursor.movePosition(QTextCursor.StartOfLine)
                else:
                    # shift one line two spaces to the right
                    indent_prev = self.getIndentOfPreviewsBlock()
                    if self.textCursor().positionInBlock() >= indent_prev:
                        cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, end - start)
                        cursor.insertText('  ')
                    else:
                        # move to the position of previous indent
                        cursor.movePosition(QTextCursor.StartOfLine)
                        pose_of_line = cursor.position()
                        cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
                        cursor.insertText("%s%s" % (' ' * indent_prev, cursor.selectedText().lstrip()))
                        cursor.setPosition(pose_of_line + indent_prev, QTextCursor.MoveAnchor)
            else:
                # shift the selected block two spaces to the left
                if back:
                    removed = 0
                    for i in reversed(range(start, end)):
                        cursor.setPosition(i)
                        if cursor.atBlockStart():
                            cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, 2)
                            if cursor.selectedText() == '  ':
                                cursor.insertText('')
                                removed += 2
                            else:
                                cursor.movePosition(QTextCursor.StartOfLine)
                                cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, 1)
                                if cursor.selectedText() == ' ':
                                    cursor.insertText('')
                                    removed += 1
                                elif cursor.selectedText() == "\t":
                                    cursor.insertText('')
                                    removed += 1
                    cursor.setPosition(start)
                    cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, end - start - removed)
                else:
                    # shift selected block two spaces to the right
                    inserted = 0
                    for i in reversed(range(start, end)):
                        cursor.setPosition(i)
                        if cursor.atBlockStart():
                            cursor.insertText('  ')
                            inserted += 2
                    cursor.setPosition(start)
                    cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, end - start + inserted)
            self.setTextCursor(cursor)
            cursor.endEditBlock()

    def indentCurrentLine(self, count=0):
        '''
        Increase indentation of current line according to the preview line.
        '''
        cursor = self.textCursor()
        if not cursor.isNull():
            # one undo operation
            cursor.beginEditBlock()
            start = cursor.selectionStart()
            end = cursor.selectionEnd()
            cursor.setPosition(start)
            block_start = cursor.blockNumber()
            cursor.setPosition(end)
            block_end = cursor.blockNumber()
            ident = ''
            for _ in range(count):
                ident += ' '
            if block_end - block_start == 0:
                # shift one line of count spaces to the right
                cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, end - start)
                cursor.insertText(ident)
            else:
                # shift selected block two spaces to the right
                inserted = 0
                for i in reversed(range(start, end)):
                    cursor.setPosition(i)
                    if cursor.atBlockStart():
                        cursor.insertText(ident)
                        inserted += count
                cursor.setPosition(start)
                cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, end - start + inserted)
            self.setTextCursor(cursor)
            cursor.endEditBlock()

    def getIdentOfCurretLine(self):
        cursor = self.textCursor()
        if not cursor.isNull():
            cursor.movePosition(QTextCursor.StartOfLine)
            cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
            line = cursor.selectedText()
            return len(line) - len(line.lstrip(' '))
        return 0

    def getIndentOfPreviewsBlock(self):
        cursor = self.textCursor()
        if not cursor.isNull():
            cursor.movePosition(QTextCursor.PreviousBlock)
            cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
            line = cursor.selectedText()
            return len(line) - len(line.lstrip(' '))
        return 0

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
                        cursor.insertText('<include file="$(find %s)%s" />' % (package, text.replace(path, '')))
                    else:
                        cursor.insertText('<include file="%s" />' % text)
                else:
                    if package:
                        cursor.insertText('<rosparam file="$(find %s)%s" command="load" />' % (package, text.replace(path, '')))
                    else:
                        cursor.insertText('<rosparam file="%s" command="load" />' % text)
            else:
                cursor.insertText(e.mimeData().text())
        e.accept()

    #############################################################################
    ########## Ctrl&Space  Context menu                                    ######
    #############################################################################

    def show_custom_context_menu(self, pos):
        menu = QTextEdit.createStandardContextMenu(self)
#    if not self.textCursor().selectedText():
#      self.setTextCursor(self.cursorForPosition(pos))
        submenu = self._create_context_menu_for_tag()
        if submenu is not None:
            menu.addMenu(submenu)
        argmenu = self._create_context_substitution_menu()
        if argmenu is not None:
            menu.addMenu(argmenu)
        menu.exec_(self.mapToGlobal(pos))

    def contextMenuEvent(self, event):
        QTextEdit.contextMenuEvent(self, event)

    def _create_context_menu_for_tag(self):
        if isinstance(self.hl, XmlHighlighter):
            tag = self.hl.get_tag_of_current_block(self.textCursor().block(), self.textCursor().positionInBlock())
            if tag:
                try:
                    menu = QMenu("ROS <%s>" % tag, self)
                    menu.triggered.connect(self._context_activated)
                    # create a menu with attributes
                    menu_attr = QMenu("attributes", menu)
                    attributes = sorted(list(set(XmlHighlighter.LAUNCH_ATTR[tag])))
                    for attr in attributes:
                        action = menu_attr.addAction(attr.rstrip('='))
                        action.setData('%s""' % attr)
                    menu.addMenu(menu_attr)
                    # create a menu with tags
                    tags = sorted(XmlHighlighter.LAUNCH_CHILDS[tag])
                    if tags:
                        menu_tags = QMenu("tags", menu)
                        for tag in tags:
                            data = '<%s></%s>' % (tag, tag) if XmlHighlighter.LAUNCH_CHILDS[tag] else '<%s/>' % tag
                            action = menu_tags.addAction(tag)
                            action.setData(data)
                        menu.addMenu(menu_tags)
                    return menu
                except:
                    import traceback
                    print traceback.format_exc(1)
                    return None
        return None

    def _create_context_substitution_menu(self, force_all=True):
        if isinstance(self.hl, XmlHighlighter):
            text = self.toPlainText()
            pos = self.textCursor().position() - 1
            try:
                if force_all or (text[pos] == '$' or (text[pos] == '(' and text[pos - 1] == '$')):
                    menu = QMenu("ROS substitution args", self)
                    menu.triggered.connect(self._context_activated)
                    for arg in self.SUBSTITUTION_ARGS:
                        action = menu.addAction("%s" % arg)
                        if force_all:
                            action.setData("$(%s )" % arg)
                        else:
                            action.setData("(%s" % arg if text[pos] == '$' else "%s" % arg)
                    return menu
            except:
                pass
        return None

    def _context_activated(self, arg):
        cursor = self.textCursor()
        if not cursor.isNull():
            cursor.insertText(arg.data())

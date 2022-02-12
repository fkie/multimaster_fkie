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



from python_qt_binding.QtCore import QRegExp, Qt, Signal, QEvent
from python_qt_binding.QtGui import QColor, QFont, QKeySequence, QTextCursor, QTextDocument
import os
import re
import rospy
import traceback

from fkie_node_manager_daemon import exceptions, file_item
from fkie_node_manager_daemon.common import find_included_files, get_arg_names, get_internal_args, replace_arg, utf8
from fkie_node_manager.common import package_name
from fkie_node_manager.detailed_msg_box import MessageBox
from fkie_node_manager.parameter_dialog import ParameterDialog
import fkie_node_manager as nm

from .xml_highlighter import XmlHighlighter
from .yaml_highlighter import YamlHighlighter


try:
    from python_qt_binding.QtGui import QAction, QApplication, QMenu, QTextEdit, QToolTip
except Exception:
    from python_qt_binding.QtWidgets import QAction, QApplication, QMenu, QTextEdit, QToolTip


class TextEdit(QTextEdit):
    '''
    The XML editor to handle the included files. If an included file in the opened
    launch file is detected, this can be open by STRG+(mouse click) in a new
    editor.
    '''

    load_request_signal = Signal(str)
    ''' :ivar load_request_signal: A signal for request to open a configuration file'''

    SUBSTITUTION_ARGS = ['env', 'optenv', 'find', 'anon', 'arg']
    CONTEXT_FILE_EXT = ['.launch', '.test', '.xml']
    YAML_VALIDATION_FILES = ['.yaml', '.iface', '.sync']

    def __init__(self, filename, parent=None):
        self.parent = parent
        QTextEdit.__init__(self, parent)
        self.setObjectName('Editor - %s' % filename)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_custom_context_menu)
#        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.setAcceptRichText(False)
        font = QFont("courier new", 12)
        self.setFont(font)
        self.setLineWrapMode(QTextEdit.NoWrap)
        self.setTabStopWidth(25)
        self.setAcceptRichText(False)
        self.setCursorWidth(2)
        self.setProperty("backgroundVisible", True)
        bg_style = "QTextEdit { background-color: #fffffc;}"
        self.setStyleSheet("%s" % (bg_style))
        self.setTextColor(QColor(0, 0, 0))
        self.regexp_list = [QRegExp("\\binclude\\b"), QRegExp("\\btextfile\\b"),
                            QRegExp("\\bfile\\b"), QRegExp("\\bvalue=.*pkg:\/\/\\b"),
                            QRegExp("\\bvalue=.*package:\/\/\\b"),
                            QRegExp("\\bvalue=.*\$\(find\\b"),
                            QRegExp("\\bargs=.*\$\(find\\b"),
                            QRegExp("\\bdefault=.*\$\(find\\b")]
        self.filename = filename
        self.file_mtime = 0
#             f = QFile(filename)
#             if f.open(QIODevice.ReadOnly | QIODevice.Text):
#                 self.file_info = QFileInfo(filename)
#                 self.setText(unicode(f.readAll(), "utf-8"))

        self.path = '.'
        # variables for threaded search
        self._search_thread = None
        self._stop = False
        self._internal_args = {}
        self._ext = os.path.splitext(filename)[1]
        self.setText("Loading file content ... press F5 to reload!")
        self.setReadOnly(True)
        self._to_select = []
        nm.nmd().file.file_content.connect(self._apply_file_content)
        nm.nmd().file.error.connect(self._on_nmd_error)
        if self.filename:
            nm.nmd().file.get_file_content_threaded(filename)

    def _apply_file_content(self, filename, file_size, file_mtime, content):
        if not self.isReadOnly():
            return
        if self.filename == filename:
            self.setText("")
            self.file_mtime = file_mtime
            if self._ext in self.CONTEXT_FILE_EXT:
                self._internal_args = get_internal_args(content)
            self.document().setPlainText(content)
            self.document().setModified(False)
            self._is_launchfile = False
            if self._ext in ['.launch', '.xml', '.xacro', '.srdf', '.urdf']:
                if self._ext in self.CONTEXT_FILE_EXT:
                    self._is_launchfile = True
                self.hl = XmlHighlighter(self.document(), is_launch=False)
                self.cursorPositionChanged.connect(self._document_position_changed)
            else:
                self.hl = YamlHighlighter(self.document())
            self.setReadOnly(False)
            # enables drop events
            self.setAcceptDrops(True)
            self._select()

    def _on_nmd_error(self, method, url, path, msg):
        if self.isReadOnly():
            if path == self.filename:
                self.setText("%s\n\npress F5 to reload!" % msg)

    def select(self, startpos, endpos, isnode):
        self._to_select.append((startpos, endpos, isnode))
        if not self.isReadOnly():
            self._select()

    def _select(self):
        if self._to_select:
            startpos, endpos, isnode = self._to_select[0]
            if isnode:
                comment_start = self.document().find('<!--', startpos, QTextDocument.FindBackward)
                if not comment_start.isNull():
                    comment_end = self.document().find('-->', comment_start)
                    if not comment_end.isNull() and comment_end.position() > endpos:
                        # commented -> return
                        self._to_select.pop(0)
                        return
            cursor = self.textCursor()
            cursor.beginEditBlock()
            cursor.setPosition(startpos, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, endpos - startpos)
            cursor.endEditBlock()
            self.setTextCursor(cursor)
            cursor_y = self.cursorRect(cursor).top()
            vbar = self.verticalScrollBar()
            vbar.setValue(vbar.value() + cursor_y * 0.8)
            self._to_select.pop(0)

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
        if self.isReadOnly():
            return False, True, "Cannot save, the content was not loaded properly!"
        if force or self.document().isModified():
            try:
                mtime = nm.nmd().file.save_file(self.filename, self.toPlainText().encode('utf-8'), 0 if force else self.file_mtime)
                self.file_mtime = mtime
                if mtime == 0:
                    MessageBox.warning(self, "Warning", "File not saved and not error reported: %s" % os.path.basename(self.filename))
                self.document().setModified(mtime == 0)
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
                            return True, True, utf8(e)
                # validate the yaml structure of yaml files
                elif ext[1] in self.YAML_VALIDATION_FILES:
                    try:
                        import ruamel.yaml
                        ruamel.yaml.load(self.toPlainText().encode('utf-8'), Loader=ruamel.yaml.Loader)
                    except ruamel.yaml.MarkedYAMLError as e:
                        return True, True, "YAML validation error: %s" % e
                return True, False, ''
            except IOError as ioe:
                if ioe.errno in [file_item.EFILE_CHANGED, file_item.EFILE_REMOVED]:
                    result = MessageBox.question(self, "Changed file", "%s\n%s" % (utf8(ioe), "Save anyway?"), buttons=MessageBox.Yes | MessageBox.No)
                    if result == MessageBox.Yes:
                        return self.save(force=True)
                else:
                    return False, True, utf8(ioe)
            except Exception as e:
                print(traceback.format_exc())
                return False, True, utf8(e)
        return False, False, ''

    def toprettyxml(self):
        try:
            from . import xmlformatter
            formatter = xmlformatter.Formatter(indent="2", indent_char=" ", encoding_output='utf-8', preserve=["literal"])
            if self._is_launchfile:
                formatter.attr_order = ['if', 'unless', 'name', 'pkg', 'type']
            xml_pretty_str = formatter.format_string(self.toPlainText().encode('utf-8'))
            cursor = self.textCursor()
            if not cursor.isNull():
                cursor.beginEditBlock()
                cursor.movePosition(QTextCursor.Start)
                cursor.movePosition(QTextCursor.End, QTextCursor.KeepAnchor)
                cursor.insertText(xml_pretty_str)
                cursor.endEditBlock()
        except Exception as err:
            msg = "Format XML failed: %s" % utf8(err)
            rospy.logwarn(msg)
            MessageBox.warning(self, "Warning", msg)

    def toprettyyaml(self):
        try:
            from . import yamlformatter
            formatter = yamlformatter.YamlFormatter()
            pretty_str = formatter.format_string(self.toPlainText().encode('utf-8'))
            cursor = self.textCursor()
            if not cursor.isNull():
                cursor.beginEditBlock()
                cursor.movePosition(QTextCursor.Start)
                cursor.movePosition(QTextCursor.End, QTextCursor.KeepAnchor)
                cursor.insertText(pretty_str)
                cursor.endEditBlock()
        except Exception as err:
            msg = "Format YAML failed: %s" % utf8(err)
            rospy.logwarn(msg)
            MessageBox.warning(self, "Warning", msg)

    def markLine(self, no):
        try:
            cursor = self.textCursor()
            cursor.setPosition(0, QTextCursor.MoveAnchor)
            while (cursor.block().blockNumber() + 1 < no):
                cursor.movePosition(QTextCursor.NextBlock, QTextCursor.MoveAnchor)
            cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            self.setTextCursor(cursor)
        except Exception:
            pass

    def setCurrentPath(self, path):
        '''
        Sets the current working path. This path is to open the included files,
        which contains the relative path.

        :param str path: the path of the current opened file (without the file)
        '''
        self.path = path

    def index(self, text):
        '''
        Searches in the given text for key indicates the including of a file and
        return their index.

        :param str text: text to find
        :return: the index of the including key or -1
        :rtype: int
        '''
        for pattern in self.regexp_list:
            index = pattern.indexIn(text)
            if index > -1:
                return index
        return -1

    def focusInEvent(self, event):
        # check for file changes
        if self.filename and self.file_mtime:
            nm.nmd().file.check_for_changed_files_threaded({self.filename: self.file_mtime})
        QTextEdit.focusInEvent(self, event)

    def file_changed(self, mtime):
        if self.file_mtime != mtime:
            self.file_mtime = mtime
            result = MessageBox.question(self, "File changed", "File was changed, reload?", buttons=MessageBox.Yes | MessageBox.No)
            if result == MessageBox.Yes:
                try:
                    _, self.file_mtime, file_content = nm.nmd().file.get_file_content(self.filename, force=True)
                    self.document().setPlainText(file_content)
                    self.document().setModified(False)
                    self.textChanged.emit()
                except Exception as err:
                    MessageBox.critical(self, "Error", "Cannot open launch file %s" % self.filename, utf8(err))

    def _strip_bad_parts(self, textblock, current_position):
        result = textblock
        startidx = textblock.rfind('$(find ', 0, current_position)
        if startidx == -1:
            startidx = textblock.rfind(' /', 0, current_position)
        if startidx == -1:
            startidx = textblock.rfind("'", 0, current_position)
        if startidx == -1:
            startidx = textblock.rfind('"', 0, current_position)
        endidx = textblock.find(' $(find ', current_position)
        if endidx == -1:
            endidx = textblock.find(' /', current_position)
        if endidx == -1:
            endidx = textblock.find("'", current_position)
        if endidx == -1:
            endidx = textblock.find('"', current_position)
        if startidx > 0 and endidx > 0:
            result = textblock[startidx:endidx]
        elif startidx > 0:
            result = textblock[startidx:]
        elif endidx > 0:
            result = textblock[:endidx]
        return result

    def mouseReleaseEvent(self, event):
        '''
        Opens the new editor, if the user clicked on the included file and sets the
        default cursor.
        '''
        if self.isReadOnly():
            event.accept()
            return
        if event.modifiers() == Qt.ControlModifier or event.modifiers() == Qt.ShiftModifier:
            cursor = self.cursorForPosition(event.pos())
            try:
                textblock = self._strip_bad_parts(cursor.block().text(), cursor.positionInBlock())
                for inc_file in find_included_files(textblock, False, False, search_in_ext=[]):
                    aval = inc_file.raw_inc_path
                    aitems = aval.split("'")
                    for search_for in aitems:
                        if not search_for:
                            continue
                        try:
                            rospy.logdebug("try to interpret: %s" % search_for)
                            args_in_name = get_arg_names(search_for)
                            resolved_args = {}
                            # if found arg in the name, try to detect values
                            if args_in_name:
                                rospy.logdebug("  args %s in filename found, try to resolve..." % args_in_name)
                                resolved_args = self.parent.graph_view.get_include_args(args_in_name, search_for, self.filename)
                            if resolved_args:
                                params = {}
                                self._internal_args
                                # create parameter dialog
                                for key, val in resolved_args.items():
                                    values = list(val)
                                    # add args defined in current file
                                    if key in self._internal_args and self._internal_args[key] not in values:
                                        values.append(self._internal_args[key])
                                    params[key] = {':type': 'string', ':value': values}
                                dia = ParameterDialog(params, store_geometry="open_launch_on_click")
                                dia.setFilterVisible(False)
                                dia.setWindowTitle('Select Parameter')
                                if dia.exec_():
                                    params = dia.getKeywords()
                                    search_for = replace_arg(search_for, params)
                                else:
                                    # canceled -> cancel interpretation
                                    QTextEdit.mouseReleaseEvent(self, event)
                                    return
                            # now resolve find-statements
                            rospy.logdebug("  send interpret request to daemon: %s" % search_for)
                            inc_files = nm.nmd().launch.get_interpreted_path(self.filename, text=[search_for])
                            for path, exists in inc_files:
                                try:
                                    rospy.logdebug("  received interpret request from daemon: %s, exists: %d" % (path, exists))
                                    if exists:
                                        event.setAccepted(True)
                                        self.load_request_signal.emit(path)
                                    else:
                                        _filename, file_extension = os.path.splitext(path)
                                        if file_extension in nm.settings().launch_view_file_ext:
                                            # create a new file, if it does not exists
                                            result = MessageBox.question(self, "File not exists", '\n\n'.join(["Create a new file?", path]), buttons=MessageBox.Yes | MessageBox.No)
                                            if result == MessageBox.Yes:
                                                content = '<launch>\n\n</launch>' if path.endswith('.launch') else ''
                                                nm.nmd().file.save_file(path, content.encode(), 0)
                                                event.setAccepted(True)
                                                self.load_request_signal.emit(path)
                                except Exception as e:
                                    MessageBox.critical(self, "Error", "File not found %s" % path, detailed_text=utf8(e))
                        except exceptions.ResourceNotFound as not_found:
                            MessageBox.critical(self, "Error", "Resource not found %s" % search_for, detailed_text=utf8(not_found.error))
            except Exception as err:
                print(traceback.format_exc())
                MessageBox.critical(self, "Error", "Error while request included file %s" % self.filename, detailed_text=utf8(err))
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
        if self.isReadOnly():
            if event.key() == Qt.Key_F5:
                nm.nmd().file.get_file_content_threaded(self.filename)
            else:
                event.accept()
                return
        if event.key() == Qt.Key_Control or event.key() == Qt.Key_Shift:
            self.setMouseTracking(True)
        if event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_7:
            self.commentText()
        elif event.modifiers() == Qt.ControlModifier | Qt.ShiftModifier and event.key() == Qt.Key_Slash:
            self.commentText()
        elif event.modifiers() == Qt.ControlModifier | Qt.ShiftModifier and event.key() == Qt.Key_F:
            if isinstance(self.hl, XmlHighlighter):
                self.toprettyxml()
            else:
                self.toprettyyaml()
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
                    self.indentCurrentLine(ident - self.getIdentOfCurretLine())
        else:
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
            QTextEdit.keyReleaseEvent(self, event)

    def goto(self, linenr, select_line=True):
            if linenr > self.document().blockCount():
                linenr = self.document().blockCount()
            curpos = self.textCursor().blockNumber() + 1
            while curpos != linenr:
                mov = QTextCursor.NextBlock if curpos < linenr else QTextCursor.PreviousBlock
                self.moveCursor(mov)
                curpos = self.textCursor().blockNumber() + 1
            self.moveCursor(QTextCursor.EndOfBlock)
            self.moveCursor(QTextCursor.StartOfBlock, QTextCursor.KeepAnchor)

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
            while (cursor.block().blockNumber() <= block_end):
                cursor.movePosition(QTextCursor.StartOfLine)
                # XML comment
                if xml_file:
                    xmlre_start = re.compile(r"<!-- ?")
                    xmlre_end = re.compile(r" ?-->")
                    cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
                    if do_comment:
                        cursor.insertText("<!-- %s -->" % cursor.selectedText().replace("--", "- - "))
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
                        cursor.insertText(res.replace("- - ", "--"))
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
                bn = cursor.block().blockNumber()
                cursor.movePosition(QTextCursor.NextBlock)
                if cursor.block().blockNumber() == bn:
                    # break if no new line is there
                    break
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

    # ############################################################################
    # ######### Drag&Drop                                                   ######
    # ############################################################################

    def dragEnterEvent(self, e):
        if e.mimeData().hasFormat('text/plain'):
            e.accept()
        else:
            e.ignore()

    def dragMoveEvent(self, e):
        e.accept()

    def dropEvent(self, e):
        if self.isReadOnly():
            return
        cursor = self.cursorForPosition(e.pos())
        if not cursor.isNull():
            text = e.mimeData().text()
            # the files will be included
            if text.startswith('grpc://'):
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

    # ############################################################################
    # ######### Ctrl&Space  Context menu                                    ######
    # ############################################################################

    def show_custom_context_menu(self, pos):
        if self.isReadOnly():
            return
        menu = QTextEdit.createStandardContextMenu(self)
        comment_action = QAction("Switch comment", self, statusTip="", triggered=self.commentText)
        comment_action.setShortcuts(QKeySequence("Ctrl+7"))
        menu.addAction(comment_action)
        formattext_action = None
        if isinstance(self.hl, XmlHighlighter):
            formattext_action = QAction("Format XML", self, statusTip="", triggered=self.toprettyxml)
        else:
            formattext_action = QAction("Format as YAML", self, statusTip="", triggered=self.toprettyyaml)
        formattext_action.setShortcuts(QKeySequence("Ctrl+Shift+F"))
        menu.addAction(formattext_action)
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
                    if tag in XmlHighlighter.LAUNCH_ATTR:
                        menu_attr = QMenu("attributes", menu)
                        attributes = sorted(list(set(XmlHighlighter.LAUNCH_ATTR[tag])))
                        for attr in attributes:
                            action = menu_attr.addAction(attr.rstrip('='))
                            action.setData('%s""' % attr)
                        menu.addMenu(menu_attr)
                    # create a menu with tags
                    if tag in XmlHighlighter.LAUNCH_CHILDS:
                        tags = sorted(XmlHighlighter.LAUNCH_CHILDS[tag])
                        if tags:
                            menu_tags = QMenu("tags", menu)
                            for tag in tags:
                                data = '<%s></%s>' % (tag, tag) if XmlHighlighter.LAUNCH_CHILDS[tag] else '<%s/>' % tag
                                action = menu_tags.addAction(tag)
                                action.setData(data)
                            menu.addMenu(menu_tags)
                    return menu
                except Exception:
                    print(traceback.format_exc(1))
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
            except Exception:
                pass
        return None

    def _context_activated(self, arg):
        cursor = self.textCursor()
        if not cursor.isNull():
            cursor.insertText(arg.data())

    # ############################################################################
    # ######### Tooltip                                                     ######
    # ############################################################################

    def event(self, event):
        if event.type() == QEvent.ToolTip:
            cursor = self.cursorForPosition(event.pos())
            cursor.select(QTextCursor.BlockUnderCursor)
            if cursor.selectedText():
                for dp, np in XmlHighlighter.DEPRECATED_PARAMETER.items():
                    if 'name="%s"' % dp in cursor.selectedText():
                        QToolTip.showText(event.globalPos(), ' %s is deprecated, use %s' % (dp, np))
            else:
                QToolTip.hideText()
            return True
        return QTextEdit.event(self, event)

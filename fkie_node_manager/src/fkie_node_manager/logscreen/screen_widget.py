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



from datetime import datetime
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, QFile, QIODevice, QRegExp
from python_qt_binding.QtGui import QColor, QIcon, QPalette, QTextCursor, QTextCharFormat, QTextDocument, QKeySequence
try:
    from python_qt_binding.QtGui import QWidget, QTextEdit, QDialog, QShortcut
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QTextEdit, QDialog, QShortcut
import os
import rospy
import shlex
import sys
import subprocess
import threading
import time

from .screen_highlighter import ScreenHighlighter
from .terminal_formats import TerminalFormats
from .logger_handler import LoggerHandler
import fkie_node_manager as nm
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon.common import sizeof_fmt
from fkie_node_manager_daemon.host import get_hostname


class ScreenTextBrowser(QTextEdit):

    def __init__(self, parent=None):
        QTextEdit.__init__(self, parent)
        self._reader = None
        self.setAutoFillBackground(False)
        self.setReadOnly(True)
        self.setUndoRedoEnabled(False)
        self.setAutoFormatting(QTextEdit.AutoNone)
        self.setAcceptRichText(False)
        # self.textBrowser.document().setMaximumBlockCount(100)
        p = QPalette()
        p.setColor(QPalette.Base, Qt.black)
        p.setColor(QPalette.Text, Qt.white)
        self.setPalette(p)
        # self.setFontFamily('Monospace')
        # self.setFontPointSize(12)

    def set_reader(self, reader):
        self._reader = reader

    def keyPressEvent(self, event):
        if self._reader is not None:
            if event.key() == Qt.Key_PageUp and self.verticalScrollBar().value() == 0:
                self._reader.reverse_read(self.verticalScrollBar().pageStep() / 10)
            elif event.key() == Qt.Key_Home and event.modifiers() == Qt.ShiftModifier:
                self._reader.reverse_read(-1)
        QTextEdit.keyPressEvent(self, event)

    def wheelEvent(self, event):
        if self._reader is not None:
            lines = event.angleDelta().y() / 40
            if lines > 0 and self.verticalScrollBar().value() == 0:
                self._reader.reverse_read(lines)
        QTextEdit.wheelEvent(self, event)


class ScreenWidget(QWidget):
    '''
    Shows the output of a screen.
    '''

    clear_signal = Signal()
    cleared_signal = Signal()

    output = Signal(str)
    output_prefix = Signal(str)
    error_signal = Signal(str)
    auth_signal = Signal(str, str, str)  # host, nodename, user

    def __init__(self, masteruri, screen_name, nodename, user=None, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QWidget.__init__(self, parent)
        # load the UI file
        screen_dock_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'ui', 'logscreen', 'ScreenWidget.ui')
        loadUi(screen_dock_file, self)
        self.setObjectName("ScreenWidget")
        self.setWindowIcon(nm.settings().icon('crystal_clear_show_io.png'))
        # self.setFeatures(QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable)
        self.pauseButton.setIcon(nm.settings().icon('sekkyumu_pause.png'))
        self._valid = True
        self._logpath = ''
        self._lock = threading.RLock()
        self.finished = False
        self.qfile = None
        self.thread = None
        self._info = ''
        self._masteruri = ''
        self._nodename = nodename
        self._first_fill = True
        self._seek_start = -1
        self._seek_end = -1
        self._pause_read_end = False
        self._ssh_output_file = None
        self._ssh_error_file = None
        self._ssh_input_file = None
        self._on_pause = False
        self._char_format_end = None
        self.logframe.setVisible(False)
        self.loglevelButton.toggled.connect(self.on_toggle_loggers)
        self.logger_handler = None
        # connect to the button signals
        self.output.connect(self._on_output)
        self.output_prefix.connect(self._on_output_prefix)
        self.error_signal.connect(self._on_error)
        self.auth_signal.connect(self.on_request_pw)
        self.clearCloseButton.clicked.connect(self.clear)
        # self.pauseButton.clicked.connect(self.stop)
        self.pauseButton.toggled.connect(self.pause)
        self.clear_signal.connect(self.clear)
        self.loggerFilterInput.textChanged.connect(self.on_logger_filter_changed)
        self.textBrowser.verticalScrollBar().valueChanged.connect(self.on_scrollbar_position_changed)
        self.textBrowser.verticalScrollBar().rangeChanged.connect(self.on_scrollbar_range_changed)
        self.textBrowser.set_reader(self)
        self.tf = TerminalFormats()
        self.hl = ScreenHighlighter(self.textBrowser.document())
        self.searchFrame.setVisible(False)
        self.grepFrame.setVisible(False)
        self.grepLineEdit.textChanged.connect(self.on_grep_changed)
        self._shortcut_search = QShortcut(QKeySequence(self.tr("Ctrl+F", "Activate search")), self)
        self._shortcut_search.activated.connect(self.on_search)
        self._shortcut_grep = QShortcut(QKeySequence(self.tr("Ctrl+G", "Activate grep")), self)
        self._shortcut_grep.activated.connect(self.on_grep)
        self.searchLineEdit.editingFinished.connect(self.on_search_prev)
        self.searchNextButton.clicked.connect(self.on_search_next)
        self.searchPrevButton.clicked.connect(self.on_search_prev)
        # self.visibilityChanged.connect(self.stop)
        self._connect(masteruri, screen_name, nodename, user)

    def masteruri(self):
        return self._masteruri

    def name(self):
        return self._nodename

    def clear(self):
        '''
        Removes all messages and emit the `cleared_signal`.
        '''
        self.textBrowser.clear()
        self.infoLabel.setText('')
        self.cleared_signal.emit()

    def finish(self):
        self.finished = True
        self.output.disconnect()
        self.output_prefix.disconnect()
        self.close()

    def closeEvent(self, event):
        self.stop()
        QWidget.closeEvent(self, event)

    def hide(self):
        self.stop()
        QWidget.hide(self)

    def close(self):
        self.stop()
        QWidget.close(self)

    def on_search(self):
        self.searchFrame.setVisible(not self.searchFrame.isVisible())
        if self.searchFrame.isVisible():
            self.searchLineEdit.setFocus()
            self.searchLineEdit.selectAll()
        else:
            cursor = self.textBrowser.textCursor()
            cursor.clearSelection()
            self.textBrowser.setTextCursor(cursor)
            self.textBrowser.setFocus()

    def on_search_next(self):
        self._perform_search(forward=True)

    def on_search_prev(self):
        self._perform_search(forward=False)

    def _perform_search(self, forward=False):
        search_str = self.searchLineEdit.text()
        if search_str:
            cursor = self.textBrowser.textCursor()
            if forward:
                search_result = self.textBrowser.document().find(search_str, cursor)
            else:
                search_result = self.textBrowser.document().find(search_str, cursor, QTextDocument.FindBackward)
            if search_result.position() > -1:
                self.textBrowser.setTextCursor(search_result)
                self.searchLabel.setText('')
                # self.searchLabel.setText('%d' % search_result.position())
            else:
                self.searchLabel.setText('no results')
        else:
            self.searchLabel.setText('')

    def on_grep(self):
        self.grepFrame.setVisible(not self.grepFrame.isVisible())
        if self.grepFrame.isVisible():
            self.grepLineEdit.setFocus()
            self.on_grep_changed(self.grepLineEdit.text())
            self.hl.set_grep_text('')
            self.grepLineEdit.selectAll()
        else:
            self.on_grep_changed('')
            self.textBrowser.setFocus()

    def on_grep_changed(self, text):
        self.hl.set_grep_text(text)

    def stop(self):
        '''
        '''
        if self.finished:
            return
        if self.qfile is not None and self.qfile.isOpen():
            self.qfile.close()
            self.qfile = None
            self._seek_start = -1
            self._seek_end = -1
            self._pause_read_end = False
            # self.clear()
        try:
            self._ssh_output_file.close()
            self._ssh_error_file.close()
            # send Ctrl+C to remote process
            self._ssh_input_file.write('%s\n' % chr(3))
            self._ssh_input_file.close()
        except Exception:
            pass
        self.finished = True

    def pause(self, state):
        self._on_pause = state

    def valid(self):
        return self._valid

    def _connect(self, masteruri, screen_name, nodename, user=None):
        self._masteruri = masteruri
        if self.qfile is not None and self.qfile.isOpen():
            self.qfile.close()
            self.clear_signal.emit()
        host = get_hostname(masteruri)
        if nm.is_local(host):
            self.setWindowTitle(nodename)
            self._nodename = nodename
            if screen_name:
                screen_log = screen.get_logfile(node=nodename)
            else:
                screen_log = screen.get_ros_logfile(node=nodename)
            self._logpath = screen_log
            self.qfile = QFile(screen_log)
            if self.qfile.open(QIODevice.ReadOnly):
                self._first_fill = True
                self.qfile.seek(self.qfile.size()-1)
                # self.lread()
                self._info = "END"
                self.thread = threading.Thread(target=self._read_log, kwargs={"filename": screen_log})
                self.thread.setDaemon(True)
                self.thread.start()
            else:
                self._valid = False
        else:
            self._connect_ssh(host, nodename, user)
        if self._valid:
            self.logger_handler = LoggerHandler(nodename, masteruri=masteruri, layout=self.scrollAreaWidgetContents.layout())
            self.logger_handler.update()
        return False

    def _read_log(self, filename, lines=80):
        while self.qfile is not None and self.qfile.isOpen():
            with self._lock:
                if self._first_fill:
                    chars_count = self._seek_count_lines(lines)
                    self._seek_start = self.qfile.pos()
                    data = self.qfile.read(chars_count)
                    if sys.version_info > (3, 0):
                        data = data.decode('utf-8')
                    self.output.emit(data)
                    self._seek_end = self.qfile.pos()
                    self._first_fill = False
                else:
                    if self._seek_end != -1:
                        self.qfile.seek(self._seek_end)
                    if (not self._pause_read_end and self.qfile.bytesAvailable()):
                        start = self.qfile.pos()
                        data = self.qfile.readAll().data()
                        if sys.version_info > (3, 0):
                            data = data.decode('utf-8')
                        self.output.emit(data)
                        self._seek_end = self.qfile.pos()
                        self._info = "NEW: %d" % (self._seek_end - start)
            time.sleep(0.25)

    def reverse_read(self, lines=20):
        with self._lock:
            if self.qfile is not None and self.qfile.isOpen():
                if lines == -1:
                    self.qfile.seek(0)
                    chars_count = self._seek_start
                else:
                    self.qfile.seek(self._seek_start)
                    chars_count = self._seek_count_lines(lines)
                self._seek_start = self.qfile.pos()
                data = self.qfile.read(chars_count)
                if sys.version_info > (3, 0):
                    data = data.decode('utf-8')
                self.output_prefix.emit(data)

    def _seek_count_lines(self, lines=20):
        if self.qfile.pos() < 2:
            self.qfile.seek(0)
            return self.qfile.pos()
        count = 0
        chars_count = 2
        line_size = 0
        count_reached = False
        self.qfile.seek(self.qfile.pos() - 2)
        while (not count_reached) and (self.qfile.pos() > 0):
            ch = self.qfile.read(1)
            self.qfile.seek(self.qfile.pos() - 2)
            chars_count += 1
            line_size += 1
            if line_size > 120:
                count += 1
                line_size = 0
            if ch == b'\n':
                count += 1  
                line_size = 0
                if count >= lines:
                    count_reached = True
        return chars_count + 1

    def _on_output_prefix(self, msg):
        '''
        This text will be prepended
        '''
        if self.finished or self._on_pause:
            return
        if msg:
            cursor = QTextCursor(self.textBrowser.document())
            self.tf.insert_formated(cursor, msg.rstrip())
            self.textBrowser.setTextCursor(cursor)
            self.textBrowser.moveCursor(QTextCursor.Start)
            self._update_info_label()

    def _on_output(self, msg):
        '''
        This text will be appended.
        '''
        if self.finished or self._on_pause:
            return
        if msg:
            at_end = self.textBrowser.verticalScrollBar().value() > self.textBrowser.verticalScrollBar().maximum() - 20
            cursor_select = self.textBrowser.textCursor()
            # store selection and do not scroll to the appended text
            if not cursor_select.hasSelection():
                cursor_select = None
            cursor = self.textBrowser.textCursor()
            cursor.movePosition(QTextCursor.End)
            if self.hl.has_grep_text():
                # grep new text
                lines = msg.splitlines(True)
                for line in lines:
                    if self.hl.contains_grep_text(line):
                        self._char_format_end = self.tf.insert_formated(cursor, line, char_format=None)
            else:
                self._char_format_end = self.tf.insert_formated(cursor, msg, char_format=self._char_format_end)
            if cursor_select is not None:
                # restore selection
                self.textBrowser.setTextCursor(cursor_select)
            elif at_end:
                self.textBrowser.moveCursor(QTextCursor.End)
            self._update_info_label()
            if not self.finished:
                self.show()

    def on_scrollbar_position_changed(self, value):
        self._update_info_label()

    def on_scrollbar_range_changed(self, min, max):
        self._update_info_label()

    def _on_error(self, msg):
        self.textBrowser.append(msg)
        self._update_info_label('SSH ERROR')

    def _update_info_label(self, info=''):
        info_text = info
        vbar_value = self.textBrowser.verticalScrollBar().value()
        if not info_text:
            if vbar_value == 0:
                if self._seek_start == 0:
                    info_text = 'START'
                else:
                    info_text += "%d %%" % (self._seek_start * 100 / self._seek_end)
            elif vbar_value == self.textBrowser.verticalScrollBar().maximum():
                info_text = 'END'
            else:
                info_text = "%d / %d" % (vbar_value / 20, self.textBrowser.verticalScrollBar().maximum() / 20)
        seek_info = ''
        if self._seek_end > -1:
            seek_info = '\t%s / %s' % (sizeof_fmt(self._seek_end - self._seek_start), sizeof_fmt(self._seek_end))
        elif self._ssh_output_file is not None:
            seek_info = '\ttail via SSH'
        self.infoLabel.setText(info_text + seek_info)

    def _connect_ssh(self, host, nodename, user=None, pw=None):
        try:
            if user is not None:
                self.infoLabel.setText('connecting to %s@%s' % (user, host))
            else:
                self.infoLabel.setText('connecting to %s' % host)
            ok = False
            self._ssh_input_file, self._ssh_output_file, self._ssh_error_file, ok = nm.ssh().ssh_exec(host, [nm.settings().start_remote_script, '--tail_screen_log', nodename], user, pw, auto_pw_request=False, get_pty=True)
            if ok:
                thread = threading.Thread(target=self._read_ssh_output, args=((self._ssh_output_file,)))
                thread.setDaemon(True)
                thread.start()
                thread = threading.Thread(target=self._read_ssh_error, args=((self._ssh_error_file,)))
                thread.setDaemon(True)
                thread.start()
            elif self._ssh_output_file:
                self._ssh_output_file.close()
                self._ssh_error_file.close()
        except nm.AuthenticationRequest as e:
            self.auth_signal.emit(host, nodename, user)
        except Exception as e:
            self.error_signal.emit('%s\n' % e)

    def on_request_pw(self, host, nodename, user):
        res, user, pw = nm.ssh()._requestPW(user, host)
        if res:
            self._connect_ssh(host, nodename, user, pw)

    def _read_ssh_output(self, output_file):
        while not output_file.closed:
            text = output_file.readline()
            if text:
                self.output.emit(text.rstrip() + '\n')

    def _read_ssh_error(self, error_file):
        try:
            while not error_file.closed:
                text = error_file.readline()
                if text:
                    self.error_signal.emit(text.rstrip() + '\n')
        except Exception:
            pass

    def on_toggle_loggers(self, state):
        self.logframe.setVisible(state)
        if state:
            self.logger_handler.update()

    def on_logger_filter_changed(self, text):
        '''
        Filter the displayed loggers
        '''
        if self.logger_handler is not None:
            self.logger_handler.filter(text)

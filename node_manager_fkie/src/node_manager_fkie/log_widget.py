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
from datetime import datetime

from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import loadUi

from .rosout_listener import RosoutListener

class LogWidget(QtGui.QDockWidget):
  '''
  The collect the the warning log messages from rosout and print it in a text 
  browser.
  '''

  added_signal = QtCore.Signal(int, int, int, int)
  '''
  added_signal will be emitted on adding a new log entry. The parameter contains
  the current count of messages (INFO, WARN, ERROR, FATAL)
  '''

  cleared_signal = QtCore.Signal()

  def __init__(self, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QDockWidget.__init__(self, parent)
    self._log_info_count = 0
    self._log_warn_count = 0
    self._log_err_count = 0
    self._log_fatal_count = 0
    # load the UI file
    log_dock_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'LogDockWidget.ui')
    loadUi(log_dock_file, self)
    self.hide()
    # connect to the button signals
    self.clearCloseButton.clicked.connect(self._on_log_clear_close_clicked)
    self.closeButton.clicked.connect(self.hide)
    # initialize the listener to the rosout topic
    self._rosout_listener = RosoutListener()
    self._rosout_listener.roswarn_signal.connect(self._on_roslog_warn)
    self._rosout_listener.roserr_signal.connect(self._on_roslog_err)
    self._rosout_listener.rosfatal_signal.connect(self._on_roslog_fatal)
    self._rosout_listener.registerByROS()

  def count(self):
    '''
    Returns the count all current viewed log messages.
    '''
    return self._log_info_count + self._log_warn_count + self._log_err_count + self._log_fatal_count

  def clear(self):
    '''
    Removes all log messages and emit the `cleared_signal`.
    '''
    self._log_info_count = 0
    self._log_warn_count = 0
    self._log_err_count = 0
    self._log_fatal_count = 0
    self.textBrowser.clear()
    self.infoLabel.setText('')
    self.cleared_signal.emit()

  def stop(self):
    '''
    Unregister the listener thread from the `/rosout` topic. This method must be
    called at the exit!
    '''
    self._rosout_listener.stop()

  def _on_roslog_info(self, msg):
    self._log_info_count += 1
    text = ('<pre style="padding:10px;"><dt><font color="#000000">'
            '<b>[INFO]</b> %s (%s:%s:%s):'
            '<br>%s</font></dt></pre>'%(self._formated_ts(msg.header.stamp),
                                        msg.file, msg.function, msg.line,
                                        msg.msg))
    self.textBrowser.append(text)
    self._update_info_label()

  def _on_roslog_warn(self, msg):
    self._log_warn_count += 1
    text = ('<pre style="padding:10px;"><dt><font color="#FE9A2E">'
            '<b>[WARN]</b> %s (%s:%s:%s):'
            '<br>%s</font></dt></pre>'%(self._formated_ts(msg.header.stamp),
                                        msg.file, msg.function, msg.line,
                                        msg.msg))
    self.textBrowser.append(text)
    self._update_info_label()

  def _on_roslog_err(self, msg):
    self._log_err_count += 1
    text = ('<pre style="padding:10px;"><dt><font color="#DF0101">'
            '<b>[ERROR]</b> %s (%s:%s:%s):'
            '<br>%s</font></dt></pre>'%(self._formated_ts(msg.header.stamp),
                                        msg.file, msg.function, msg.line,
                                        msg.msg))
    self.textBrowser.append(text)
    self._update_info_label()

  def _on_roslog_fatal(self, msg):
    self._log_fatal_count += 1
    text = ('<pre style="padding:10px;"><dt><font color="#610B0B">'
            '<b>[FATAL]</b> %s (%s:%s:%s):'
            '<br>%s</font></dt></pre>'%(self._formated_ts(msg.header.stamp),
                                        msg.file, msg.function, msg.line,
                                        msg.msg))
    self.textBrowser.append(text)
    self._update_info_label()

  def _on_log_clear_close_clicked(self):
    self.clear()
    self.hide()

  def _update_info_label(self):
    info_text = ''
    if self._log_info_count > 0:
      info_text = '%s INFO: %d   '%(info_text, self._log_info_count)
    if self._log_warn_count > 0:
      info_text = '%s WARN: %d   '%(info_text, self._log_warn_count)
    if self._log_err_count > 0:
      info_text = '%s ERROR: %d   '%(info_text, self._log_err_count)
    if self._log_fatal_count > 0:
      info_text = '%s FATAL: %d'%(info_text, self._log_fatal_count)
    self.infoLabel.setText(info_text)
    self.added_signal.emit(self._log_info_count, self._log_warn_count,
                           self._log_err_count, self._log_fatal_count)

  def _formated_ts(self, stamp):
    ts = stamp.secs + stamp.secs / 1000000000.
    return datetime.fromtimestamp(ts).strftime("%d.%m.%Y %H:%M:%S.%f")

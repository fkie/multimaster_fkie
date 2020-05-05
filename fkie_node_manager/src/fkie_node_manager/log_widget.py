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
from python_qt_binding.QtCore import Signal
try:
    from python_qt_binding.QtGui import QDockWidget
except ImportError:
    from python_qt_binding.QtWidgets import QDockWidget
import os
import rospy
import sys
try:
    from roscpp.srv import GetLoggers, SetLoggerLevel, SetLoggerLevelRequest
except ImportError as err:
    sys.stderr.write("Cannot import GetLoggers service definition: %s" % err)

from fkie_node_manager_daemon.common import utf8
from .rosout_listener import RosoutListener
import fkie_node_manager as nm


class LogWidget(QDockWidget):
    '''
    The collect the the warning log messages from rosout and print it in a text
    browser.
    '''

    added_signal = Signal(int, int, int, int)
    '''
  added_signal will be emitted on adding a new log entry. The parameter contains
  the current count of messages (INFO, WARN, ERROR, FATAL)
  '''

    cleared_signal = Signal()

    def __init__(self, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QDockWidget.__init__(self, parent)
        self._log_debug_count = 0
        self._log_info_count = 0
        self._log_warn_count = 0
        self._log_err_count = 0
        self._log_fatal_count = 0
        # load the UI file
        log_dock_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'LogDockWidget.ui')
        loadUi(log_dock_file, self)
        self.setObjectName("LogWidget")
        self.closeButton.setIcon(nm.settings().icon('crystal_clear_button_close.png'))
        self.setFeatures(QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable)
        # connect to the button signals
        self.clearCloseButton.clicked.connect(self._on_log_clear_close_clicked)
        self.closeButton.clicked.connect(self.hide)
        # initialize the listener to the rosout topic
        self._rosout_listener = RosoutListener()
        self._rosout_listener.rosdebug_signal.connect(self._on_roslog_debug)
        self._rosout_listener.rosinfo_signal.connect(self._on_roslog_info)
        self._rosout_listener.roswarn_signal.connect(self._on_roslog_warn)
        self._rosout_listener.roserr_signal.connect(self._on_roslog_err)
        self._rosout_listener.rosfatal_signal.connect(self._on_roslog_fatal)
        self._rosout_listener.registerByROS()
        self._enable_info_on_start = False
        if self._enable_info_on_start:
            try:
                service_name = "%s/get_loggers" % rospy.get_name()
                log_level_srvs = rospy.ServiceProxy(service_name, GetLoggers)
                resp = log_level_srvs()
                for logger in resp.loggers:
                    if logger.name == 'rosout':
                        if logger.level == 'DEBUG':
                            self.checkBox_debug.setChecked(True)
                            self.checkBox_info.setChecked(True)
                        elif logger.level == 'INFO':
                            self.checkBox_info.setChecked(True)
                        break
            except rospy.ServiceException as e:
                err_msg = "Service call '%s' failed: %s" % (service_name, utf8(e))
                rospy.logwarn(err_msg)
        self.checkBox_debug.stateChanged.connect(self._on_checkbox_debug_state_changed)

    def _on_checkbox_debug_state_changed(self, state):
        try:
            service_name = "%s/set_logger_level" % rospy.get_name()
            log_level_srvs = rospy.ServiceProxy(service_name, SetLoggerLevel)
            msg = SetLoggerLevelRequest()
            msg.logger = 'rosout'
            msg.level = 'DEBUG' if state else 'INFO'
            _resp = log_level_srvs(msg)
        except rospy.ServiceException as e:
            err_msg = "Service call '%s' failed: %s" % (service_name, utf8(e))
            rospy.logwarn(err_msg)

    def count(self):
        '''
        Returns the count all current viewed log messages.
        '''
        return self._log_debug_count + self._log_info_count + self._log_warn_count + self._log_err_count + self._log_fatal_count

    def count_warn(self):
        '''
        Returns the count all current warning log messages.
        '''
        return self._log_warn_count + self._log_err_count + self._log_fatal_count

    def clear(self):
        '''
        Removes all log messages and emit the `cleared_signal`.
        '''
        self._log_debug_count = 0
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

    def _on_roslog_debug(self, msg):
        if self.checkBox_debug.isChecked():
            self._log_debug_count += 1
            text = ('<pre style="padding:10px;"><dt><font color="#5EA02E">'
                    '[DEBUG] %s (%s:%s:%s:%s):\t'
                    '<b>%s</b></font></dt></pre>' % (self._formated_ts(msg.header.stamp),
                                                     msg.name, msg.file, msg.function, msg.line,
                                                     msg.msg))
            self.textBrowser.append(text)
            self._update_info_label()

    def _on_roslog_info(self, msg):
        if self.checkBox_info.isChecked():
            self._log_info_count += 1
            text = ('<pre style="padding:10px;"><dt><font color="#000000">'
                    '[INFO] %s (%s:%s:%s:%s): '
                    '<b>%s</b></font></dt></pre>' % (self._formated_ts(msg.header.stamp),
                                                     msg.name, msg.file, msg.function, msg.line,
                                                     msg.msg))
            self.textBrowser.append(text)
            self._update_info_label()

    def _on_roslog_warn(self, msg):
        self._log_warn_count += 1
        text = ('<pre style="padding:10px;"><dt><font color="#FE9A2E">'
                '[WARN] %s (%s:%s:%s:%s): '
                '<b>%s</b></font></dt></pre>' % (self._formated_ts(msg.header.stamp),
                                                 msg.name, msg.file, msg.function, msg.line,
                                                 msg.msg))
        self.textBrowser.append(text)
        self._update_info_label()

    def _on_roslog_err(self, msg):
        self._log_err_count += 1
        text = ('<pre style="padding:10px;"><dt><font color="#DF0101">'
                '[ERROR] %s (%s:%s:%s:%s): '
                '<b>%s</b></font></dt></pre>' % (self._formated_ts(msg.header.stamp),
                                                 msg.name, msg.file, msg.function, msg.line,
                                                 msg.msg))
        self.textBrowser.append(text)
        self._update_info_label()

    def _on_roslog_fatal(self, msg):
        self._log_fatal_count += 1
        text = ('<pre style="padding:10px;"><dt><font color="#610B0B">'
                '[FATAL] %s (%s:%s:%s:%s): '
                '<b>%s</b></font></dt></pre>' % (self._formated_ts(msg.header.stamp),
                                                 msg.name, msg.file, msg.function, msg.line,
                                                 msg.msg))
        self.textBrowser.append(text)
        self._update_info_label()

    def _on_log_clear_close_clicked(self):
        self.clear()
        # self.hide()

    def _update_info_label(self):
        info_text = ''
        if self._log_debug_count > 0:
            info_text = '%s DEBUG: %d   ' % (info_text, self._log_debug_count)
        if self._log_info_count > 0:
            info_text = '%s INFO: %d   ' % (info_text, self._log_info_count)
        if self._log_warn_count > 0:
            info_text = '%s WARN: %d   ' % (info_text, self._log_warn_count)
        if self._log_err_count > 0:
            info_text = '%s ERROR: %d   ' % (info_text, self._log_err_count)
        if self._log_fatal_count > 0:
            info_text = '%s FATAL: %d' % (info_text, self._log_fatal_count)
        self.infoLabel.setText(info_text)
        self.added_signal.emit(self._log_info_count, self._log_warn_count,
                               self._log_err_count, self._log_fatal_count)

    def _formated_ts(self, stamp):
        ts = stamp.secs + stamp.secs / 1000000000.
        return datetime.fromtimestamp(ts).strftime("%d.%m.%Y %H:%M:%S.%f")

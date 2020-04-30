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

from __future__ import division, absolute_import, print_function, unicode_literals

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor
try:
    from python_qt_binding.QtGui import QFrame, QHBoxLayout, QLabel, QRadioButton
except ImportError:
    from python_qt_binding.QtWidgets import QFrame, QHBoxLayout, QLabel, QRadioButton
    import rospy

import threading
import sys
try:
    from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest
except ImportError as err:
    sys.stderr.write("Cannot import SetLoggerLevel service definition: %s" % err)


class LoggerItem(QFrame):
    '''
    Represents one ROS logger and offers methods to change the logger level.
    '''

    def __init__(self, nodename, loggername, level='INFO', parent=None):
        '''
        Creates a new item.
        '''
        QFrame.__init__(self, parent)
        self.setObjectName("LoggerItem")
        self.nodename = nodename
        self.loggername = loggername
        layout = QHBoxLayout(self)
        layout.setContentsMargins(1, 1, 1, 1)
        self.debug = QRadioButton()
        self.debug.setStyleSheet("QRadioButton{ background-color: #2ECC40;}")
        self.debug.toggled.connect(self.toggled_debug)
        layout.addWidget(self.debug)
        self.info = QRadioButton()
        self.info.setStyleSheet("QRadioButton{ background-color: #CACFD2;}")
        self.info.toggled.connect(self.toggled_info)
        layout.addWidget(self.info)
        self.warn = QRadioButton()
        self.warn.setStyleSheet("QRadioButton{ background-color: #FF851B;}")
        self.warn.toggled.connect(self.toggled_warn)
        layout.addWidget(self.warn)
        self.error = QRadioButton()
        self.error.setStyleSheet("QRadioButton{ background-color: #FF4136;}")
        self.error.toggled.connect(self.toggled_error)
        layout.addWidget(self.error)
        self.fatal = QRadioButton()
        self.fatal.setStyleSheet("QRadioButton{ background-color: #FF0000;}")
        self.fatal.toggled.connect(self.toggled_fatal)
        layout.addWidget(self.fatal)
        self.label = QLabel(loggername)
        layout.addWidget(self.label)
        layout.addStretch()
        self._callback = None
        self._current_level = None
        self.set_level(level)

    def set_callback(self, callback):
        self._callback = callback

    def toggled_debug(self, state):
        if state:
            self.set_level('DEBUG')

    def toggled_info(self, state):
        if state:
            self.set_level('INFO')

    def toggled_warn(self, state):
        if state:
            self.set_level('WARN')

    def toggled_error(self, state):
        if state:
            self.set_level('ERROR')

    def toggled_fatal(self, state):
        if state:
            self.set_level('FATAL')

    def set_level(self, level):
        if level.upper() == 'DEBUG':
            self.debug.setChecked(True)
        elif level.upper() == 'INFO':
            self.info.setChecked(True)
        elif level.upper() == 'WARN':
            self.warn.setChecked(True)
        elif level.upper() == 'ERROR':
            self.error.setChecked(True)
        elif level.upper() == 'FATAL':
            self.fatal.setChecked(True)
        elif level:
            rospy.logwarn("loglevel not found '%s'" % (level))
        if self._current_level is not None:
            if self._callback is not None:
                self._callback(level)
            else:
                # call set loglevel service
                thread = threading.Thread(target=self._set_level, kwargs={'level': level})
                thread.setDaemon(True)
                thread.start()
                pass
        self._current_level = level

    def _set_level(self, level):
        try:
            set_logger_level = rospy.ServiceProxy('%s/set_logger_level' % self.nodename, SetLoggerLevel)
            msg = SetLoggerLevelRequest()
            msg.logger = self.loggername
            msg.level = level
            _resp = set_logger_level(msg)
        except rospy.ServiceException, e:
            rospy.logwarn("Set logger %s for %s to %s failed: %s" % (self.loggername, self.nodename, level, e))

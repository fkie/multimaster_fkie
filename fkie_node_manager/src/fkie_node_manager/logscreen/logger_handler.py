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

from datetime import datetime
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, Qt, Signal
from python_qt_binding.QtGui import QColor
try:
    from python_qt_binding.QtGui import QFrame, QHBoxLayout, QLabel, QRadioButton
except ImportError:
    from python_qt_binding.QtWidgets import QFrame, QHBoxLayout, QLabel, QRadioButton

import os
import rospy
import threading
import sys
try:
    from roscpp.srv import GetLoggers
except ImportError as err:
    sys.stderr.write("Cannot import GetLoggers service definition: %s" % err)

import rospy

from .logger_item import LoggerItem

class LoggerHandler(QObject):
    '''
    Handles ROS logger requests
    '''

    loggers_signal = Signal(list)

    def __init__(self, nodename, layout, parent=None):
        '''
        Creates a new item.
        '''
        QObject.__init__(self, parent)
        self.setObjectName("LoggerHandler")
        self.nodename = nodename
        self.layout = layout
        self.loggers_signal.connect(self._handle_loggers)
        self._thread_update = None

    def update(self):
        if self._thread_update is None:
            self._thread_update = threading.Thread(target=self._update_loggers)
            self._thread_update.setDaemon(True)
            self._thread_update.start()

    def _update_loggers(self):
        try:
            get_loggers = rospy.ServiceProxy('%s/get_loggers' % self.nodename, GetLoggers)
            resp = get_loggers()
            self.loggers_signal.emit(resp.loggers)
        except rospy.ServiceException, e:
            rospy.logwarn("Get loggers for %s failed: %s" % (self.nodename, e))
        self._thread_update = None

    def _handle_loggers(self, loggers):
        while self.layout.count() > 1:
            item = self.layout.takeAt(0)
            del item
        all_item = LoggerItem(self.nodename, 'all', '')
        all_item.set_callback(self.change_all)
        self.layout.insertWidget(0, all_item)
        index = 1
        for logger in loggers:
            self.layout.insertWidget(index, LoggerItem(self.nodename, logger.name, logger.level))
            index += 1

    def change_all(self, loglevel):
        index = 1
        while index < self.layout.count():
            item = self.layout.itemAt(index).widget()
            if isinstance(item, LoggerItem):
                item.set_level(loglevel)
            index += 1

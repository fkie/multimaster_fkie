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



from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor
try:
    from python_qt_binding.QtGui import QFrame, QHBoxLayout, QLabel, QRadioButton
except ImportError:
    from python_qt_binding.QtWidgets import QFrame, QHBoxLayout, QLabel, QRadioButton
    import rospy
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient
import threading
import sys
try:
    from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest
except ImportError as err:
    sys.stderr.write("Cannot import SetLoggerLevel service definition: %s" % err)

import fkie_node_manager as nm


class LoggerItem(QFrame):
    '''
    Represents one ROS logger and offers methods to change the logger level.
    '''

    success_signal = Signal(str)
    error_signal = Signal(str)

    def __init__(self, nodename, masteruri, loggername, level='INFO', parent=None):
        '''
        Creates a new item.
        '''
        QFrame.__init__(self, parent)
        self.setObjectName("LoggerItem")
        self.nodename = nodename
        self.masteruri = masteruri
        self.loggername = loggername
        self.current_level = None
        layout = QHBoxLayout(self)
        layout.setContentsMargins(1, 1, 1, 1)
        self.debug = QRadioButton()
        self.debug.setStyleSheet("QRadioButton{ background-color: #39B54A;}")  # QColor(57, 181, 74)
        self.debug.toggled.connect(self.toggled_debug)
        layout.addWidget(self.debug)
        self.info = QRadioButton()
        self.info.setStyleSheet("QRadioButton{ background-color: #FFFAFA;}")
        self.info.toggled.connect(self.toggled_info)
        layout.addWidget(self.info)
        self.warn = QRadioButton()
        self.warn.setStyleSheet("QRadioButton{ background-color: #FFC706;}")  # QColor(255, 199, 6)
        self.warn.toggled.connect(self.toggled_warn)
        layout.addWidget(self.warn)
        self.error = QRadioButton()
        self.error.setStyleSheet("QRadioButton{ background-color: #DE382B;}")  # QColor(222, 56, 43)
        self.error.toggled.connect(self.toggled_error)
        layout.addWidget(self.error)
        self.fatal = QRadioButton()
        self.fatal.setStyleSheet("QRadioButton{ background-color: #FF0000;}")
        self.fatal.toggled.connect(self.toggled_fatal)
        layout.addWidget(self.fatal)
        self.label = QLabel(loggername)
        layout.addWidget(self.label)
        layout.addStretch()
        self._callback = None  # used to set all logger
        self.success_signal.connect(self.on_succes_update)
        self.error_signal.connect(self.on_error_update)
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

    def on_succes_update(self, level):
        button = None
        if level.upper() == 'DEBUG':
            button = self.debug
        elif level.upper() == 'INFO':
            button = self.info
        elif level.upper() == 'WARN':
            button = self.warn
        elif level.upper() == 'ERROR':
            button = self.error
        elif level.upper() == 'FATAL':
            button = self.fatal
        elif level:
            rospy.logwarn("loglevel not found '%s'" % (level))
            return
        if button is not None:
            checked = self._callback is None
            if not checked:
                button.setAutoExclusive(False)
            button.setChecked(checked)
            if not checked:
                button.setAutoExclusive(True)
        self.current_level = level

    def on_error_update(self, level):
        self.on_succes_update(level)

    def set_level(self, level, force=False):
        if self.current_level is not None or force:
            if self._callback is not None:
                self._callback(level)
            else:
                # call set loglevel service
                thread = threading.Thread(target=self._set_level, kwargs={'level': level, 'current_level': self.current_level})
                thread.setDaemon(True)
                thread.start()
                pass
        else:
            self.on_succes_update(level)

    def _set_level(self, level, current_level):
        try:
            backup_level = current_level
            service_name = '%s/set_logger_level' % self.nodename
            # get service URI from ROS-Master
            master = xmlrpcclient.ServerProxy(self.masteruri)
            code, _, serviceuri = master.lookupService(rospy.get_name(), service_name)
            if code == 1:
                self.call_service_set_level(serviceuri, service_name, self.loggername, level)
                self.success_signal.emit(level)
        except rospy.ServiceException as e:
            rospy.logwarn("Set logger %s for %s to %s failed: %s" % (self.loggername, self.nodename, level, e))
            if backup_level is not None:
                self.error_signal.emit(backup_level)

    @classmethod
    def call_service_set_level(cls, serviceuri, servicename, loggername, level):
        _req, _resp = nm.starter().callService(serviceuri, servicename, SetLoggerLevel, service_args=[loggername, level])

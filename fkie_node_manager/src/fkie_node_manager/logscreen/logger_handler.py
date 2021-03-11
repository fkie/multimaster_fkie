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
from python_qt_binding.QtCore import QObject, QRegExp, Qt, Signal
from python_qt_binding.QtGui import QColor
try:
    from python_qt_binding.QtGui import QFrame, QHBoxLayout, QLabel, QRadioButton
except ImportError:
    from python_qt_binding.QtWidgets import QFrame, QHBoxLayout, QLabel, QRadioButton

try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

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
import fkie_node_manager as nm


class LoggerHandler(QObject):
    '''
    Handles ROS logger requests
    '''

    loggers_signal = Signal(list)
    level_changed_signal = Signal(list)

    def __init__(self, nodename, masteruri, layout, parent=None):
        '''
        Creates a new item.
        '''
        QObject.__init__(self, parent)
        self.setObjectName("LoggerHandler")
        self.nodename = nodename
        self.masteruri = masteruri
        self._filter = QRegExp('', Qt.CaseInsensitive, QRegExp.Wildcard)
        self._logger_items = {}  # logger name: LoggerItem
        self.layout = layout
        self._change_all_cancel = False
        self._stored_values = {}
        self.loggers_signal.connect(self._handle_loggers)
        self._thread_update = None
        self._thread_set_all = None
        self._all_item = LoggerItem(self.nodename, self.masteruri, 'all', '')
        self._all_item.set_callback(self.change_all)

    def update(self):
        if self._thread_update is None:
            self._thread_update = threading.Thread(target=self._update_loggers)
            self._thread_update.setDaemon(True)
            self._thread_update.start()

    def _update_loggers(self):
        try:
            service_name = '%s/get_loggers' % self.nodename
            master = xmlrpcclient.ServerProxy(self.masteruri)
            code, _, serviceuri = master.lookupService(rospy.get_name(), service_name)
            if code == 1:
                _req, resp = nm.starter().callService(serviceuri, service_name, GetLoggers, service_args=[])
                self.loggers_signal.emit(resp.loggers)
        except (rospy.ServiceException, nm.StartException) as e:
            rospy.logwarn("Get loggers for %s failed: %s" % (self.nodename, e))
        except IOError as err:
            rospy.logwarn("Get loggers for %s failed; cannot get service URI from %s: %s" % (self.nodename, self.masteruri, err))
        self._thread_update = None

    def _handle_loggers(self, loggers):
        new_logger = {}
        for logger in loggers:
            new_logger[logger.name] = logger.level
            self._stored_values[logger.name] = logger.level
        while self.layout.count() > 1:
            item = self.layout.takeAt(0)
            wd = item.widget()
            if wd.current_level is not None and wd.loggername != 'all':
                self._stored_values[wd.loggername] = wd.current_level
            wd.setParent(None)
        self._logger_items.clear()
        index = 0
        if not 'all' in self._stored_values:
            self.layout.insertWidget(0, self._all_item)
            index += 1
        for logger_name, logger_level in sorted(self._stored_values.items()):
            item = LoggerItem(self.nodename, self.masteruri, logger_name, logger_level)
            self._logger_items[logger_name] = item
            if (not logger_name in new_logger) or new_logger[logger_name] != logger_level:
                item.set_level(logger_level, True)
            self.layout.insertWidget(index, item)
            index += 1
            if self._filter.indexIn(logger.name) == -1:
                item.setVisible(False)

    def change_all(self, loglevel, ignore=['ros.roscpp.roscpp_internal',
                                           'ros.roscpp.roscpp_internal.connections',
                                           'ros.roscpp.superdebug',
                                           'rospy.tcpros']):
        '''
        Change the log level of all logger in a new thread.
        '''
        if self._thread_set_all is not None:
            self._thread_set_all.cancel()
            self._thread_set_all.success_signal.disconnect()
            self._thread_set_all.error_signal.disconnect()
        index = 1
        itemlist = []
        while index < self.layout.count():
            item = self.layout.itemAt(index).widget()
            if isinstance(item, LoggerItem) and item.loggername not in ignore:
                itemlist.append((item.loggername, item.current_level))
            index += 1
        self._thread_set_all = SetAllThread(self.nodename, self.masteruri, itemlist, loglevel)
        self._thread_set_all.success_signal.connect(self.on_success_set)
        self._thread_set_all.error_signal.connect(self.on_error_set)
        self._thread_set_all.setDaemon(True)
        self._thread_set_all.start()

    def on_success_set(self, nodename, logger, level):
        if logger in self._logger_items:
            self._logger_items[logger].on_succes_update(level)
        elif logger == 'all':
            self._all_item.on_succes_update(level)

    def on_error_set(self, nodename, logger, level):
        if logger in self._logger_items:
            self._logger_items[logger].on_error_update(level)
        elif logger == 'all':
            self._all_item.on_error_update(level)

    def filter(self, text):
        self._filter = QRegExp(text, Qt.CaseInsensitive, QRegExp.Wildcard)
        for name, item in self._logger_items.items():
            hidden = self._filter.indexIn(name) != -1
            item.setVisible(hidden)


class SetAllThread(QObject, threading.Thread):
    '''
    A thread to set the level of all loggers and publish 
    the new level it be sending a QT signal.
    '''
    success_signal = Signal(str, str, str)
    error_signal = Signal(str, str, str)

    def __init__(self, nodename, masteruri, loggers, newlevel):
        '''
        :param str nodename: the name of the node
        :param str masteruri: the master where the service is registered
        :param list logger: list with tuple of (logger and current level)
        :param str newlevel: new log level
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._nodename = nodename
        self._masteruri = masteruri
        self._loggers = loggers
        self._newlevel = newlevel
        self._cancel = False
        self.setDaemon(True)

    def run(self):
        '''
        '''
        service_name = '%s/set_logger_level' % self._nodename
        master = xmlrpcclient.ServerProxy(self._masteruri)
        code, _, serviceuri = master.lookupService(rospy.get_name(), service_name)
        if code == 1:
            for logger, level in self._loggers:
                try:
                    if not self._cancel:
                        LoggerItem.call_service_set_level(serviceuri, service_name, logger, self._newlevel)
                        self.success_signal.emit(self._nodename, logger, self._newlevel)
                except Exception as err:
                    rospy.logwarn("Set logger %s for %s to %s failed: %s" % (logger, self._nodename, self._newlevel, err))
                    if level is not None:
                        self.error_signal.emit(self._nodename, logger, level)
        self.success_signal.emit(self._nodename, 'all', self._newlevel)

    def cancel(self):
        self._cancel = True
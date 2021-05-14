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
from python_qt_binding.QtCore import Qt, Signal
try:
    from python_qt_binding.QtGui import QDockWidget, QTabWidget
except ImportError:
    from python_qt_binding.QtWidgets import QDockWidget, QTabWidget

import os
import rospy
import sys
try:
    from roscpp.srv import GetLoggers, SetLoggerLevel, SetLoggerLevelRequest
except ImportError as err:
    sys.stderr.write("Cannot import GetLoggers service definition: %s" % err)

from fkie_node_manager_daemon.host import get_hostname
from .detachable_tab_dock import DetachableTabDock
from .screen_widget import ScreenWidget


class ScreenDock(DetachableTabDock):
    '''
    Extend detachable dock with connect method to create ScreenWidget.
    The connect() can be called from different thread.
    '''

    connect_signal = Signal(str, str, str, str)  # host, screen_name, nodename, user
    closed_signal = Signal(DetachableTabDock)

    def __init__(self, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        DetachableTabDock.__init__(self, parent)
        self.setObjectName("ScreenDock")
        self.setWindowTitle("Screens")
        self.setFeatures(QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable)
        self.connect_signal.connect(self._on_connect)
        self.tab_widget.close_tab_request_signal.connect(self.close_tab_requested)
        self.tab_widget.tab_removed_signal.connect(self.tab_removed)
        self._nodes = {}  # tuple of (host, nodename) : ScreenWidget

    def connect(self, masteruri, screen_name, nodename, user=''):
        self.connect_signal.emit(masteruri, screen_name, nodename, user)

    def _on_connect(self, masteruri, screen_name, nodename, user=''):
        if (masteruri, nodename) not in self._nodes:
            sw = ScreenWidget(masteruri, screen_name, nodename, str(user))
            if sw.valid():
                tab_index = self.tab_widget.addTab(sw, nodename + ('' if screen_name else ' ROSLOG'))
                self.tab_widget.setCurrentIndex(tab_index)
                self._nodes[(masteruri, nodename)] = sw
            else:
                rospy.logwarn('ros log file %s not found!' % sw._logpath)
                sw.close()
        else:
            index = self.tab_widget.indexOf(self._nodes[(masteruri, nodename)])
            if index >= 0:
                self.tab_widget.setCurrentIndex(index)
            else:
                for dia in self._open_dialogs:
                    index = dia.tab_widget.indexOf(self._nodes[(masteruri, nodename)])
                    if index >= 0:
                        dia.tab_widget.setCurrentIndex(index)
                        dia.raise_()
                        dia.activateWindow()
                        break
        self.show()

    def update_node(self, node):
        '''
        Updates the logger state of the given node.

        :param node: Node object
        :type node: node_tree_model.NodeItem
        '''
        # if not node.isLocal:
        #     # update only notifications from 'local' nodes. While sync we avoid multiple updates
        #     return
        name = node.name
        if (node.masteruri, name) in self._nodes:
            self._nodes[(node.masteruri, name)].logger_handler.update()

    def close_tab_requested(self, tab_widget, index):
        tab_widget.removeTab(index)

    def tab_removed(self, widget):
        try:
            del self._nodes[(widget.masteruri(), widget.name())]
        except Exception:
            pass
        widget.close()

    def finish(self):
        self.tab_widget.clear()
        for od in self._open_dialogs:
            od.tab_widget.clear()
        self._nodes.clear()

    def closeEvent(self, event):
        # close tabs on hide
        self.tab_widget.clear()
        self.closed_signal.emit(self)
        DetachableTabDock.closeEvent(self, event)

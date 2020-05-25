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


from .detachable_tab_dialog import DetachableTabDialog
from .detachable_tab_widget import DetachableTabWidget


class DetachableTabDock(QDockWidget):
    '''
    The main widget where tabs are open.
    '''
    closed_signal = Signal(QDockWidget)

    def __init__(self, parent=None):
        '''
        Creates the window, connects the signals and init the class.
        '''
        QDockWidget.__init__(self, parent)
        self.setObjectName("ScreenDock")
        self.setWindowTitle("Screens")
        self.setFeatures(QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetMovable)  # | QDockWidget.DockWidgetClosable)
        self._parent_dock = None
        self._open_dialogs = []
        self.tab_widget = DetachableTabWidget(self)
        self.tab_widget.currentChanged.connect(self.on_tab_changed)
        self.tab_widget.detach_signal.connect(self.on_detach)
        self.setWidget(self.tab_widget)

    def on_tab_changed(self, index):
        pass

    def on_detach_from_dialog(self, name, widget, point, geometry, by_double_click):
        attached = False
        for dia in self._open_dialogs:
            if dia.tab_widget.prepared_for_drop():
                dia.tab_widget.attach_tab(widget, name)
                attached = True
                break
        if not attached:
            if self.tab_widget.prepared_for_drop() or by_double_click:
                self.tab_widget.attach_tab(widget, widget.name())
                self.show()
            else:
                # create a new dock widget
                dt = DetachableTabDock(self.parentWidget())
                dt.setWindowTitle('SubScreen')
                dt.setFeatures(QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable)
                dt._parent_dock = self
                dt.tab_widget.attach_tab(widget, name)
                dt.tab_widget.empty_tabbar_signal.connect(dt._close_if_empty)
                dt.tab_widget.tab_removed_signal.connect(self._forward_tab_removed)
                dt.tab_widget.close_tab_request_signal.connect(self._forward_close_tab_requested)
                dt.closed_signal.connect(self._on_dialog_closed)
                self._open_dialogs.append(dt)
                self.parentWidget().addDockWidget(Qt.BottomDockWidgetArea, dt)

    def on_detach(self, name, widget, point, geometry, by_double_click):
        if self._parent_dock is not None:
            # lets handle on_detach by parent dock
            self._parent_dock.on_detach(name, widget, point, geometry, by_double_click)
            return
        attached = False
        if self.tab_widget.prepared_for_drop():
            self.tab_widget.attach_tab(widget, widget.name())
            self.show()
            attached = True
        for dia in self._open_dialogs:
            if dia.tab_widget.prepared_for_drop():
                dia.tab_widget.attach_tab(widget, name)
                attached = True
                break
        if not attached:
            # Create a new detached tab window
            detached_tab = DetachableTabDialog(widget, self)
            detached_tab.setWindowModality(Qt.NonModal)
            detached_tab.setWindowTitle(name)
            # detached_tab.setWindowIcon(icon)
            detached_tab.setObjectName(name)
            detached_tab.setGeometry(geometry)
            detached_tab.tab_widget.tab_removed_signal.connect(self._forward_tab_removed)
            detached_tab.tab_widget.close_tab_request_signal.connect(self._forward_close_tab_requested)
            detached_tab.tab_widget.detach_signal.connect(self.on_detach_from_dialog)
            detached_tab.move(point)
            detached_tab.closed_signal.connect(self._on_dialog_closed)
            detached_tab.show()
            self._open_dialogs.append(detached_tab)

    def _forward_close_tab_requested(self, widget, index):
        self.tab_widget.tab_removed_signal.emit(widget)

    def _forward_tab_removed(self, widget):
        self.tab_widget.tab_removed_signal.emit(widget)

    def _on_dialog_closed(self, dialog):
        try:
            self._open_dialogs.remove(dialog)
        except Exception:
            import traceback
            print(traceback.format_exc())

        self.tab_widget.empty_tabbar_signal.connect(self._close_if_empty)

    def _close_if_empty(self):
        '''
        Close this dialog if not tabs are inside.
        '''
        if self.tab_widget.count() == 0:
            self.close()

    def closeEvent(self, event):
        # close tabs on hide
        self.tab_widget.clear()
        self.closed_signal.emit(self)
        QDockWidget.closeEvent(self, event)

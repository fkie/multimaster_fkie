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



from python_qt_binding.QtCore import QPoint, QRect, Qt, Signal
import os

import rospy
import fkie_node_manager as nm

try:
    from python_qt_binding.QtGui import QTabWidget, QWidget
except Exception:
    from python_qt_binding.QtWidgets import QTabWidget, QWidget

from .detachable_tab_bar import DetachableTabBar


class DetachableTabWidget(QTabWidget):
    '''
    This class was overloaded to close tabs on middle mouse click
    '''
    detach_signal = Signal(str, QWidget, QPoint, QRect, bool)  # bool: True if double click
    close_tab_request_signal = Signal(QTabWidget, int)
    tab_removed_signal = Signal(QWidget)
    empty_tabbar_signal = Signal()

    def __init__(self, parent=None):
        QTabWidget.__init__(self, parent)
        tab_bar = DetachableTabBar(self)
        tab_bar.detach_tab_signal.connect(self.detach_tab)
        tab_bar.move_tab_signal.connect(self.move_tab)
        tab_bar.empty_signal.connect(self._on_empty_tabbar)
        self.tabCloseRequested.connect(self._close_requested)
        self.setTabBar(tab_bar)
        self.setAcceptDrops(True)
        self.setTabPosition(QTabWidget.North)
        # self.setDocumentMode(True)
        self.setTabsClosable(True)
        self.setObjectName("tabWidget")

    def removeTab(self, index, detach=False):
        # inform about tab was removed and forward contained page to subscriber
        widget = self.widget(index)
        QTabWidget.removeTab(self, index)
        if not detach:
            self.tab_removed_signal.emit(widget)

    def _on_empty_tabbar(self):
        self.empty_tabbar_signal.emit()

    def _close_requested(self, index):
        self.close_tab_request_signal.emit(self, index)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MidButton:
            close_index = self.tabBar().tabAt(event.pos())
            if close_index > -1:
                self.tabCloseRequested.emit(close_index)
                event.setAccepted(True)
        if not event.isAccepted():
            QTabWidget.mouseReleaseEvent(event)

    def setMovable(self, movable):
        '''
        The default movable functionality of QTabWidget must remain disabled
        so as not to conflict with the added features
        '''
        pass

    def prepared_for_drop(self):
        return self.tabBar().prepared_for_drop()

    def move_tab(self, fromIndex, toIndex):
        '''
        Move a tab from one position (index) to another

        :param fromIndex:    the original index location of the tab
        :param toIndex:      the new index location of the tab
        '''
        if fromIndex != toIndex:
            widget = self.widget(fromIndex)
            icon = self.tabIcon(fromIndex)
            text = self.tabText(fromIndex)

            self.removeTab(fromIndex, detach=True)
            self.insertTab(toIndex, widget, icon, text)
            self.setCurrentIndex(toIndex)

    def detach_tab(self, index, point, by_double_click):
        '''
        Detach the tab by removing it's contents and placing them in
        a DetachedTab dialog

        :param index:    the index location of the tab to be detached
        :param point:    the screen position for creating the new DetachedTab dialog
        :param by_double_click:  True if detach comes from double click
        '''
        content_widget = self.widget(index)
        if content_widget is not None:
            self.detach_signal.emit(self.tabText(index), content_widget, point, content_widget.frameGeometry(), by_double_click)

    # def event(self, event):

    #     # If the event type is QEvent.NonClientAreaMouseButtonDblClick then
    #     # close the dialog
    #     if event.type() == 176:
    #         event.accept()
    #         self.close()

    #     return QDialog.event(self, event)

    def closeEvent(self, event):
        self.clear()
        QTabWidget.closeEvent(self, event)

    def attach_tab(self, widget, name, icon=None):
        '''
        Re-attach the tab by removing the content from the DetachedTab dialog,
        closing it, and placing the content back into the DetachableTabWidget

        :param QWidget widget: the content widget from the DetachedTab dialog
        :param str name: the name of the detached tab
        :param QIcon icon: the window icon for the detached tab
        '''
        # Make the content widget a child of this widget
        widget.setParent(self)
        if icon is None:
            index = self.addTab(widget, name)
        else:
            index = self.addTab(widget, icon, name)
        # Make this tab the current tab
        if index > -1:
            self.setCurrentIndex(index)

    def clear(self):
        '''
        Remove all tabs. On remove a tab_removed_signal() is emited to inform subscriber about closed tabs.
        '''
        while (self.count() > 0):
            self.removeTab(0)




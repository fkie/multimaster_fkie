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



import fkie_node_manager as nm

from python_qt_binding.QtCore import QPoint, Qt, Signal, QMimeData, QEvent
from python_qt_binding.QtGui import QCursor, QDrag, QMouseEvent, QPainter, QPixmap

try:
    from python_qt_binding.QtGui import QApplication, QTabBar
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QTabBar


class DetachableTabBar(QTabBar):
    '''
    The TabBar class re-implements some of the functionality of the QTabBar widget
    '''
    detach_tab_signal = Signal(int, QPoint, bool)  # tab at pos, mouse cursor position, by double click
    move_tab_signal = Signal(int, int)
    empty_signal = Signal()

    def __init__(self, parent=None):
        QTabBar.__init__(self, parent)

        self.setAcceptDrops(True)
        self.setElideMode(Qt.ElideRight)
        self.setSelectionBehaviorOnRemove(QTabBar.SelectLeftTab)

        self.drag_start_pos = QPoint()
        self.drag_droped_pos = QPoint()
        self.mouse_cursor = QCursor()
        self.drag_initiated = False

    def mouseDoubleClickEvent(self, event):
        '''
        Send the detach_tab_signal when a tab is double clicked
        '''
        event.accept()
        self.detach_tab_signal.emit(self.tabAt(event.pos()), self.mouse_cursor.pos(), True)

    def mousePressEvent(self, event):
        '''
        Set the starting position for a drag event when the mouse button is pressed
        '''
        self.drag_droped_pos = QPoint(0, 0)
        self.drag_initiated = False
        if event.button() == Qt.LeftButton:
            self.drag_start_pos = event.pos()
        QTabBar.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        '''
        Determine if the current movement is a drag.  If it is, convert it into a QDrag.  If the
        drag ends inside the tab bar, emit an move_tab_signal.  If the drag ends outside the tab
        bar, emit an detach_tab_signal.
        '''
        # Determine if the current movement is detected as a drag
        if not self.drag_start_pos.isNull() and ((event.pos() - self.drag_start_pos).manhattanLength() > QApplication.startDragDistance()):
            self.drag_initiated = True

        # If the current movement is a drag initiated by the left button
        if ((event.buttons() & Qt.LeftButton)) and self.drag_initiated:

            # Stop the move event
            finishMoveEvent = QMouseEvent(QEvent.MouseMove, event.pos(), Qt.NoButton, Qt.NoButton, Qt.NoModifier)
            QTabBar.mouseMoveEvent(self, finishMoveEvent)

            # Convert the move event into a drag
            drag = QDrag(self)
            mime_data = QMimeData()
            mime_data.setData('action', b'application/tab-detach')
            drag.setMimeData(mime_data)

            # Create the appearance of dragging the tab content
            # tab_index = self.tabAt(self.drag_start_pos)
            pixmap = self.parentWidget().grab()
            targetPixmap = QPixmap(pixmap.size())
            targetPixmap.fill(Qt.transparent)
            painter = QPainter(targetPixmap)
            painter.setOpacity(0.85)
            painter.drawPixmap(0, 0, pixmap)
            painter.end()
            drag.setPixmap(targetPixmap)

            # Initiate the drag
            dropAction = drag.exec_(Qt.MoveAction | Qt.CopyAction)

            # If the drag completed outside of the tab bar, detach the tab and move
            # the content to the current cursor position
            if dropAction == Qt.IgnoreAction:
                event.accept()
                self.detach_tab_signal.emit(self.tabAt(self.drag_start_pos), self.mouse_cursor.pos(), False)
            elif dropAction == Qt.MoveAction:
                # else if the drag completed inside the tab bar, move the selected tab to the new position
                if not self.drag_droped_pos.isNull():
                    self.move_tab_signal.emit(self.tabAt(self.drag_start_pos), self.tabAt(self.drag_droped_pos))
                else:
                    # else if the drag completed inside the tab bar new TabBar, move the selected tab to the new TabBar
                    self.detach_tab_signal.emit(self.tabAt(self.drag_start_pos), self.mouse_cursor.pos(), False)
                event.accept()
        else:
            QTabBar.mouseMoveEvent(self, event)

    def dragEnterEvent(self, event):
        '''
        Determine if the drag has entered a tab position from another tab position
        '''
        self.drag_droped_pos = QPoint(0, 0)
        mime_data = event.mimeData()
        formats = mime_data.formats()
        if 'action' in formats and mime_data.data('action') == 'application/tab-detach':
            event.acceptProposedAction()
        QTabBar.dragMoveEvent(self, event)

    def dropEvent(self, event):
        '''
        Get the position of the end of the drag
        '''
        self.drag_droped_pos = event.pos()
        mime_data = event.mimeData()
        formats = mime_data.formats()
        if 'action' in formats and mime_data.data('action') == 'application/tab-detach':
            event.acceptProposedAction()
        QTabBar.dropEvent(self, event)
    
    def prepared_for_drop(self):
        return not self.drag_droped_pos.isNull()

    def tabRemoved(self, index):
        QTabBar.tabRemoved(self, index)
        if self.count() == 0:
            self.empty_signal.emit()

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



import rospy
import fkie_node_manager as nm

from python_qt_binding.QtCore import QPoint, Qt, Signal, QMimeData, QEvent
from python_qt_binding.QtGui import QIcon

try:
    from python_qt_binding.QtGui import QDialog, QWidget, QVBoxLayout
except Exception:
    from python_qt_binding.QtWidgets import QDialog, QWidget, QVBoxLayout

from .detachable_tab_widget import DetachableTabWidget


class DetachableTabDialog(QDialog):
    '''
    When a tab is detached, the contents are placed into this QDialog.  The tab
    can be re-attached by closing the dialog or by double clicking on its
    window frame.
    '''
    closed_signal = Signal(QDialog)

    def __init__(self, content_widget, parent=None):
        QDialog.__init__(self, parent)
        self.tab_widget = DetachableTabWidget(self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.addWidget(self.tab_widget)
        self.setWindowFlags(Qt.Window)
        tab_index = self.tab_widget.addTab(content_widget, content_widget.name())
        self.tab_widget.setCurrentIndex(tab_index)
        self.tab_widget.empty_tabbar_signal.connect(self._close_if_empty)

    def _close_if_empty(self):
        '''
        Close this dialog if not tabs are inside.
        '''
        if self.tab_widget.count() == 0:
            self.close()

    def closeEvent(self, event):
        '''
        Close TabWidget to remove all tabs.
        '''
        self.tab_widget.clear()
        self.closed_signal.emit(self)
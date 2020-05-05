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



from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QLineEdit, QToolButton, QStyle
except Exception:
    from python_qt_binding.QtWidgets import QLineEdit, QToolButton, QStyle
import fkie_node_manager as nm

class EnhancedLineEdit(QLineEdit):

    stop_signal = Signal()
    ''' stop button was pressed '''

    refresh_signal = Signal(str)
    ''' sends a refresh signal with current text. This signal is emmited if text was changed or button was pressed '''

    def __init__(self, parent=None):
        QLineEdit.__init__(self, parent=None)
        self.process_active = False
        # create a reload button with icon
        self.button_reload = button_reload = QToolButton(self)
        icon = QIcon.fromTheme("view-refresh", nm.settings().icon('oxygen_view_refresh.png'))
        button_reload.setIcon(icon)
        button_reload.setCursor(Qt.ArrowCursor)
        button_reload.setStyleSheet("QToolButton { border: none; padding: 0px; }")

        # create a stop button with icon
        self.button_stop = button_stop = QToolButton(self)
        icon = QIcon.fromTheme("process-stop", nm.settings().icon('oxygen_view_refresh.png'))
        button_stop.setIcon(icon)
        button_stop.setCursor(Qt.ArrowCursor)
        button_stop.setStyleSheet("QToolButton { border: none; padding: 0px; }")
        button_stop.hide()

        # signals, clear lineEdit if btn pressed; change btn visibility on input
        button_reload.clicked.connect(self._emit_refresh_text)
        self.textChanged[str].connect(self.update_close_button)
        button_stop.clicked.connect(self._process_stop)

        frameWidth = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        self.setStyleSheet("QLineEdit { padding-right: %dpx; } " % (button_reload.sizeHint().width() + frameWidth + 1))
        msz = self.minimumSizeHint()
        self.setMinimumSize(max(msz.width(), button_reload.sizeHint().height() + frameWidth * 2 + 2),
                            max(msz.height(), button_reload.sizeHint().height() + frameWidth * 2 + 2))
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._emit_refresh_text)

    def resizeEvent(self, event):
        sz = self.button_reload.sizeHint()
        frameWidth = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        self.button_reload.move(self.rect().right() - frameWidth - sz.width(),
                                (self.rect().bottom() + 1 - sz.height()) / 2)
        self.button_stop.move(self.rect().right() - frameWidth - sz.width(),
                              (self.rect().bottom() + 1 - sz.height()) / 2)

    def update_close_button(self, text):
        self._timer.stop()
        self._timer.start()
        # self.button_reload.setVisible(True if text else False)

    def set_process_active(self, state):
        if self.process_active != state:
            self.process_active = state
            self.button_reload.setVisible(not state)
            self.button_stop.setVisible(state)

    def _process_stop(self):
        self.stop_signal.emit()

    def _emit_refresh_text(self):
        self.set_process_active(True)
        self.refresh_signal.emit(self.text())

    def keyPressEvent(self, event):
        '''
        Enable the ESC handling
        '''
        if event.key() == Qt.Key_Escape and self.text():
            self.setText('')
            self._timer.stop()
        elif event.key() in [Qt.Key_Return, Qt.Key_Enter]:
            self._timer.stop()
            self._emit_refresh_text()
        else:
            event.accept()
            QLineEdit.keyPressEvent(self, event)

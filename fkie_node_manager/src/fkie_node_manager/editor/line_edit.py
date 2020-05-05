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



from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QLineEdit, QToolButton, QStyle
except Exception:
    from python_qt_binding.QtWidgets import QLineEdit, QToolButton, QStyle

import fkie_node_manager as nm


class EnhancedLineEdit(QLineEdit):

    def __init__(self, parent=None):
        QLineEdit.__init__(self, parent)
        # Create a clear button with icon
        self.clearBtn = clearBtn = QToolButton(self)
        icon = QIcon.fromTheme("edit-clear", nm.settings().icon('crystal_clear_button_close.png'))
        clearBtn.setIcon(icon)
        clearBtn.setCursor(Qt.ArrowCursor)
        clearBtn.setStyleSheet("QToolButton { border: none; padding: 0px; }")
        clearBtn.hide()

        # signals, clear lineEdit if btn pressed; change btn visibility on input
        clearBtn.clicked.connect(self.clear)
        self.textChanged[str].connect(self.update_close_button)

        frameWidth = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        self.setStyleSheet("QLineEdit { padding-right: %dpx; } " % (clearBtn.sizeHint().width() + frameWidth + 1))
        msz = self.minimumSizeHint()
        self.setMinimumSize(max(msz.width(), clearBtn.sizeHint().height() + frameWidth * 2 + 2),
                            max(msz.height(), clearBtn.sizeHint().height() + frameWidth * 2 + 2))

    def resizeEvent(self, event):
        sz = self.clearBtn.sizeHint()
        frameWidth = self.style().pixelMetric(QStyle.PM_DefaultFrameWidth)
        self.clearBtn.move(self.rect().right() - frameWidth - sz.width(),
                           (self.rect().bottom() + 1 - sz.height()) / 2)

    def update_close_button(self, text):
        self.clearBtn.setVisible(True if text else False)

    def keyPressEvent(self, event):
        '''
        Enable the ESC handling
        '''
        if event.key() == Qt.Key_Escape and self.text():
            self.setText('')
        else:
            event.accept()
            QLineEdit.keyPressEvent(self, event)

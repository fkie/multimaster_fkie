from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QLineEdit, QToolButton, QStyle
except:
    from python_qt_binding.QtWidgets import QLineEdit, QToolButton, QStyle


class EnchancedLineEdit(QLineEdit):

    def __init__(self, parent=None):
        QLineEdit.__init__(self, parent)
        # Create a clear button with icon
        self.clearBtn = clearBtn = QToolButton(self)
        icon = QIcon.fromTheme("edit-clear", QIcon(":/icons/crystal_clear_button_close.png"))
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

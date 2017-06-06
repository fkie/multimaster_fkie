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

try:
    from python_qt_binding.QtGui import QMessageBox, QPushButton, QSpacerItem, QSizePolicy, QTextEdit
except:
    from python_qt_binding.QtWidgets import QMessageBox, QPushButton, QSpacerItem, QSizePolicy, QTextEdit


class DetailedError(Exception):
    ''' '''

    def __init__(self, title, text, detailed_text=""):
        self.title = title
        self.value = text
        self.detailed_text = detailed_text

    def __str__(self):
        return repr(self.text) + ":::" + self.detailed_text


class WarningMessageBox(QMessageBox):

    def __init__(self, icon, title, text, detailed_text="", buttons=QMessageBox.Ok):
        QMessageBox.__init__(self, icon, title, text, buttons)
        self.textEdit = None
        if detailed_text:
            self.setDetailedText(detailed_text)
            self.textEdit = textEdit = self.findChild(QTextEdit)
            if textEdit is not None:
                textEdit.setMinimumHeight(0)
                textEdit.setMaximumHeight(16777215)
                textEdit.setMinimumWidth(0)
                textEdit.setMaximumWidth(16777215)
                textEdit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            # horizontalSpacer = QSpacerItem(480, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)
            # layout = self.layout()
            # layout.addItem(horizontalSpacer, layout.rowCount(), 0, 1, layout.columnCount())
        if QMessageBox.Abort & buttons:
            self.setEscapeButton(QMessageBox.Abort)
        elif QMessageBox.Ignore & buttons:
            self.setEscapeButton(QMessageBox.Ignore)
        elif QMessageBox.Cancel & buttons:
            self.setEscapeButton(QMessageBox.Cancel)
        else:
            self.setEscapeButton(buttons)
        self.ignore_all_btn = QPushButton('Don\'t display again')
        self.addButton(self.ignore_all_btn, QMessageBox.HelpRole)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.ignore_all_btn.setVisible(False)

    def paintEvent(self, event):
        QMessageBox.paintEvent(self, event)
        if self.textEdit is not None:
            if self.textEdit.isVisible():
                if not self.ignore_all_btn.isVisible():
                    self.ignore_all_btn.setVisible(True)
                    self.setSizeGripEnabled(True)
                self.setMaximumHeight(16777215)
                self.setMaximumWidth(16777215)
            elif not self.textEdit.isVisible():
                if self.ignore_all_btn.isVisible():
                    self.ignore_all_btn.setVisible(False)
                self.setSizeGripEnabled(False)

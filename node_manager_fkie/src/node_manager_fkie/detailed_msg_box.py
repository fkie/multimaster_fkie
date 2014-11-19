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

from python_qt_binding import QtGui

class DetailedError(Exception):
  ''' '''

  def __init__(self, title, text, detailed_text=""):
    self.title = title
    self.value = text
    self.detailed_text = detailed_text

  def __str__(self):
    return repr(self.text) + ":::" + self.detailed_text


class WarningMessageBox(QtGui.QMessageBox):

  def __init__(self, icon, title, text, detailed_text="", buttons=QtGui.QMessageBox.Ok):
    QtGui.QMessageBox.__init__(self, icon, title, text, buttons)
    if detailed_text:
      self.setDetailedText(detailed_text)
#            self.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
#            self.setSizeGripEnabled(True)
      horizontalSpacer = QtGui.QSpacerItem(480, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
      layout = self.layout()
      layout.addItem(horizontalSpacer, layout.rowCount(), 0, 1, layout.columnCount())

    if QtGui.QMessageBox.Abort & buttons:
      self.setEscapeButton(QtGui.QMessageBox.Abort)
    elif QtGui.QMessageBox.Ignore & buttons:
      self.setEscapeButton(QtGui.QMessageBox.Ignore)
    else:
      self.setEscapeButton(buttons)

    self.textEdit = textEdit = self.findChild(QtGui.QTextEdit)
    if textEdit != None :
      textEdit.setMinimumHeight(0)
      textEdit.setMaximumHeight(600)
      textEdit.setMinimumWidth(0)
      textEdit.setMaximumWidth(600)
      textEdit.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

    self.ignore_all_btn = QtGui.QPushButton('Don\'t display again')
    self.addButton(self.ignore_all_btn, QtGui.QMessageBox.HelpRole)

  def paintEvent (self, event):
    QtGui.QMessageBox.paintEvent(self, event)
    self.ignore_all_btn.setVisible(self.textEdit.isVisible() if self.textEdit else False)

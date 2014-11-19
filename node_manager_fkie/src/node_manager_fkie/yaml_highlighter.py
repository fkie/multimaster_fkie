# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# based on code of Timo Roehling
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
from python_qt_binding import QtCore

class YamlHighlighter(QtGui.QSyntaxHighlighter):
  '''
  Enabled the syntax highlightning for the yaml files.
  '''
  
  def __init__(self, parent=None):
    QtGui.QSyntaxHighlighter.__init__(self, parent)
    self.rules = []
    self.commentStart = QtCore.QRegExp("#")
    self.commentEnd = QtCore.QRegExp("\n|\r")
    self.commentFormat = QtGui.QTextCharFormat()
    self.commentFormat.setFontItalic(True)
    self.commentFormat.setForeground(QtCore.Qt.darkGray)

    f = QtGui.QTextCharFormat()
    r = QtCore.QRegExp()
    r.setMinimal(True)
    f.setFontWeight(QtGui.QFont.Normal)
    f.setForeground (QtCore.Qt.blue)
    tagList = ["\\btrue\\b", "\\bfalse\\b"]
    for tag in tagList:
      r.setPattern(tag)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtGui.QColor(127,64,127))
    r.setPattern ("\\d+")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.darkBlue)
    r.setPattern("^\s*[_.\w]*\s*:")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.darkBlue)
    r.setPattern(":\s*:[_\.\w]*$|:\s*\@[_\.\w]*$")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setFontWeight(QtGui.QFont.Bold)
    f.setForeground(QtCore.Qt.darkRed)
    r.setPattern("^\s*-")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.darkRed)
    r.setPattern("^---$")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.darkGreen)
    r.setPattern("[\[\]\{\}\,]")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setFontWeight(QtGui.QFont.Normal)
    f.setForeground(QtCore.Qt.magenta)
    r.setPattern("\".*\"|\'.*\'")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtGui.QColor(127,64,127))
    r.setPattern ("\\$\\(.*\\)")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground (QtCore.Qt.lightGray)
    r.setPattern ("<!DOCTYPE.*>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    r.setPattern ("<\\?xml.*\\?>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))


  def highlightBlock(self, text):
    for pattern, form in self.rules:
      index = pattern.indexIn(text)
      while index >= 0:
        length = pattern.matchedLength()
        self.setFormat(index, length, form)
        index = pattern.indexIn(text, index + length)

    # mark comment blocks
    self.setCurrentBlockState(0)
    startIndex = 0
    if self.previousBlockState() != 1:
      startIndex = self.commentStart.indexIn(text)
      if startIndex >= 0:
        commentLength = len(text) - startIndex
        self.setFormat(startIndex, commentLength, self.commentFormat)

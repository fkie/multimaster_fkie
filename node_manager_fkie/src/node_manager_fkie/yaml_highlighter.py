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

from python_qt_binding.QtCore import QRegExp, Qt
from python_qt_binding.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat


class YamlHighlighter(QSyntaxHighlighter):
  '''
  Enabled the syntax highlightning for the yaml files.
  '''

  def __init__(self, parent=None):
    QSyntaxHighlighter.__init__(self, parent)
    self.rules = []
    self.commentStart = QRegExp("#")
    self.commentEnd = QRegExp("\n|\r")
    self.default_format = QTextCharFormat()
    self.default_format.setForeground(QColor(24, 24, 24))
    self.commentFormat = QTextCharFormat()
    self.commentFormat.setFontItalic(True)
    self.commentFormat.setForeground(Qt.darkGray)

    f = QTextCharFormat()
    r = QRegExp()
    r.setMinimal(True)
    f.setFontWeight(QFont.Normal)
    f.setForeground(Qt.blue)
    tagList = ["\\btrue\\b", "\\bfalse\\b"]
    for tag in tagList:
      r.setPattern(tag)
      self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(QColor(127, 64, 127))
    r.setPattern("\\d+")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(Qt.darkBlue)
    r.setPattern("^\s*[_.\w]*\s*:")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(Qt.darkBlue)
    r.setPattern(":\s*:[_\.\w]*$|:\s*\@[_\.\w]*$")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setFontWeight(QFont.Bold)
    f.setForeground(Qt.darkRed)
    r.setPattern("^\s*-")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(Qt.darkRed)
    r.setPattern("^---$")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(Qt.darkGreen)
    r.setPattern("[\[\]\{\}\,]")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setFontWeight(QFont.Normal)
    f.setForeground(Qt.magenta)
    r.setPattern("\".*\"|\'.*\'")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(QColor(127, 64, 127))
    r.setPattern("\\$\\(.*\\)")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

    f.setForeground(Qt.lightGray)
    r.setPattern("<!DOCTYPE.*>")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))
    r.setPattern("<\\?xml.*\\?>")
    self.rules.append((QRegExp(r), QTextCharFormat(f)))

  def highlightBlock(self, text):
    self.setFormat(0, len(text), self.default_format)
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

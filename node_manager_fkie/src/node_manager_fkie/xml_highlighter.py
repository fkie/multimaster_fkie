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

class XmlHighlighter(QtGui.QSyntaxHighlighter):
  '''
  Enabled the syntax highlightning for the ROS launch files.
  '''
  
  def __init__(self, parent=None):
    QtGui.QSyntaxHighlighter.__init__(self, parent)
    self.rules = []
    self.commentStart = QtCore.QRegExp("<!--")
    self.commentEnd = QtCore.QRegExp("-->")
    self.commentFormat = QtGui.QTextCharFormat()
    f = QtGui.QTextCharFormat()
    r = QtCore.QRegExp()
    r.setMinimal(True)
    f.setFontWeight(QtGui.QFont.Normal)
    f.setForeground (QtCore.Qt.darkBlue)
    tagList = ["\\blaunch\\b", "\\bnode\\b", "\\bmachine\\b", "\\binclude\\b",
               "\\bremap\\b", "\\benv-loader\\b", "\\bparam\\b", "\\brosparam\\b",
               "\\bgroup\\b", "\\btest\\b", "\\barg\\b"]
    for tag in tagList:
      r.setPattern(tag)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.darkGreen)
    attrList = ["\\bdeprecated=", "\\bpkg=", "\\btype=", "\\bname=",
                "\\bargs=", "\\bmachine=", "\\brespawn=", "\\brequired=",
                "\\bns=", "\\bclear_params=", "\\boutput=", "\\bcwd=",
                "\\blaunch-prefix=", "\\baddress=", "\\benv-loader=", "\\bdefault=",
                "\\buser=", "\\bpassword=", "\\btimeout=", "\\bros-root=",
                "\\bros-package-path=", "\\bfile=", "\\bclear_params=", "\\bfrom=",
                "\\bto=", "\\bvalue=", "\\btextfile=", "\\bbinfile=",
                "\\bcommand=", "\\btest-name=", "\\btime-limit=", "\\bretry=",
                "\\if=", "\\unless="]
    for attr in attrList:
      r.setPattern(attr)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtCore.Qt.magenta)
    r.setPattern("\".*\"")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground(QtGui.QColor(127,64,127))
    r.setPattern ("\\$\\(.*\\)")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    f.setForeground (QtCore.Qt.lightGray)
    r.setPattern ("<!DOCTYPE.*>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    r.setPattern ("<\\?xml.*\\?>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    self.commentFormat.setFontItalic(True)
    self.commentFormat.setForeground(QtCore.Qt.darkGray)

  
  def highlightBlock(self, text):
    for pattern, format in self.rules:
      index = pattern.indexIn(text)
      while index >= 0:
        length = pattern.matchedLength()
        self.setFormat(index, length, format)
        index = pattern.indexIn(text, index + length)
  
    self.setCurrentBlockState(0)
    startIndex = 0
    if self.previousBlockState() != 1:
      startIndex = self.commentStart.indexIn(text)
    while startIndex >= 0:
      endIndex = self.commentEnd.indexIn(text, startIndex)
      commentLength = 0
      if endIndex == -1:
        self.setCurrentBlockState(1)
        commentLength = len(text) - startIndex
      else:
        commentLength = endIndex - startIndex + self.commentEnd.matchedLength()
      self.setFormat(startIndex, commentLength, self.commentFormat)
      startIndex = self.commentStart.indexIn(text, startIndex + commentLength)

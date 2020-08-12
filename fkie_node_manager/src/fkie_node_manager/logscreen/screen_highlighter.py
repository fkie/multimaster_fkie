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
from python_qt_binding.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QTextCursor


class ScreenHighlighter(QSyntaxHighlighter):
    '''
    Enabled the syntax highlightning for the ROS log files.
    '''
    def __init__(self, parent=None):
        QSyntaxHighlighter.__init__(self, parent)
        self._grep_format = QTextCharFormat()
        self._grep_rule = None
        self.rules = []
        self.rules.append((self._create_regexp(r'.*\[DEBUG\].', syntax=QRegExp.RegExp), self._create_format(QColor(57, 181, 74))))
        self.rules.append((self._create_regexp(r'.*\[INFO\].', syntax=QRegExp.RegExp), self._create_format(QColor('#FFFAFA'))))
        self.rules.append((self._create_regexp(r'.*\[WARN\].', syntax=QRegExp.RegExp), self._create_format(QColor(255, 199, 6))))
        self.rules.append((self._create_regexp(r'.*WARNING.', syntax=QRegExp.RegExp), self._create_format(QColor(255, 199, 6))))
        self.rules.append((self._create_regexp(r'.*\[ERROR\].', syntax=QRegExp.RegExp), self._create_format(QColor(222, 56, 43))))
        self.rules.append((self._create_regexp(r'.*\[FATAL\].', syntax=QRegExp.RegExp), self._create_format(QColor(255, 0, 0))))  #red

    def _create_format(self, color, style=''):
        _format = QTextCharFormat()
        _format.setForeground(color)
        if 'bold' in style:
            _format.setFontWeight(QFont.Bold)
        else:
            _format.setFontWeight(QFont.Normal)
        if 'italic' in style:
            _format.setFontItalic(True)
        return _format

    def highlightBlock(self, text):
        for pattern, frmt in self.rules:
            index = pattern.indexIn(text[:80])
            if index >= 0:
                self.setFormat(0, len(text), frmt)
                break
        if self._grep_rule is not None:
            index = self._grep_rule.indexIn(text)
            while index >= 0:
                length = self._grep_rule.matchedLength()
                self.setFormat(index, length, self._grep_format)
                index = self._grep_rule.indexIn(text, index + length)

    def has_grep_text(self):
        return self._grep_rule is not None

    def set_grep_text(self, text):
        if text:
            self._grep_rule = self._create_regexp(text)
            self._grep_format.setBackground(Qt.darkGreen)
        else:
            self._grep_format = QTextCharFormat()
            self._grep_rule = None

    def contains_grep_text(self, text):
        if self._grep_rule is not None:
            return self._grep_rule.indexIn(text) >= 0

    def _create_regexp(self, pattern='', cs=Qt.CaseInsensitive, syntax=QRegExp.Wildcard, minimal=False):
        _regexp = QRegExp(pattern, cs, syntax)
        _regexp.setMinimal(minimal)
        return _regexp

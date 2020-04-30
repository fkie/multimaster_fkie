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

from __future__ import division, absolute_import, print_function, unicode_literals

from python_qt_binding.QtCore import QRegExp, Qt
from python_qt_binding.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QTextCursor


class ScreenHighlighter(QSyntaxHighlighter):
    '''
    Enabled the syntax highlightning for the ROS log files.
    '''

    def __init__(self, parent=None):
        QSyntaxHighlighter.__init__(self, parent)
        self.format_default = self._create_format(QColor('#FFFAFA'))  #Snow  https://www.w3schools.com/colors/colors_names.asp
        # self.format_default.setForeground(QColor(24, 24, 24))
        self.rules = []
        self.format_warn = self._create_format(QColor('#FFA500'))  #Orange orange from https://clrs.cc/
        #self.rules.append((self._create_regexp(r"\[DEBUG\].*$"), self._create_format(QColor('#2ECC40'))))  #2ECC40 green from https://clrs.cc/
        #self.rules.append((self._create_regexp(r"\[INFO\].*$"), self._create_format(QColor('#CACFD2'))))  #CACFD2
        #self.rules.append((self._create_regexp(r"\[WARN\].*$"), self.format_warn))
        #self.rules.append((self._create_regexp(r"WARNING:.*$"), self._create_format(QColor('#FF851B'))))  #FF851B orange from https://clrs.cc/
        #self.rules.append((self._create_regexp(r"\[ERROR\].*$"), self._create_format(QColor('#FF4136'))))  #FF4136 red from https://clrs.cc/
        #self.rules.append((self._create_regexp(r"\[FATAL\].*$"), self._create_format(QColor('#FF0000'))))  #FF0000 red

    def _create_regexp(self, pattern=''):
        _regexp = QRegExp()
        _regexp.setMinimal(True)
        _regexp.setPattern(pattern)
        return _regexp

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
        for pattern, form in self.rules:
            index = pattern.indexIn(text)
            while index >= 0:
                length = pattern.matchedLength()
                self.setFormat(index, length, form)
                index = pattern.indexIn(text, index + length)

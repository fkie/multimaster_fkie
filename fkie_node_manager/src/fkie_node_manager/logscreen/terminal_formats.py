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

from python_qt_binding.QtCore import QRegExp, Qt, QObject
from python_qt_binding.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QTextCursor

import re


class TerminalFormats(QObject):
    '''
    Defines qt formats for terminal colors.
    Use ubuntu colors from https://en.wikipedia.org/wiki/ANSI_escape_code
    '''

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        self._default_color = QColor('#FFFAFA')
        self._default_bg = QColor('#010101')
        self.current_format = self._create_format()
        self.formats = {}
        self.formats['0'] = {}
        self.formats['2'] = {'style': 'bold', 'style_value': True}
        self.formats['3'] = {'style': 'italic', 'style_value': True}
        self.formats['4'] = {'style': 'underline', 'style_value': True}
        self.formats['22'] = {'style': 'bold', 'style_value': False}
        self.formats['23'] = {'style': 'italic', 'style_value': False}
        self.formats['24'] = {'style': 'underline', 'style_value': False}
        # foreground colors
        self.formats['30'] = {'color': QColor(1, 1, 1)}  # Black
        self.formats['31'] = {'color': QColor(222, 56, 43)}  # Red
        self.formats['32'] = {'color': QColor(57, 181, 74)}  # Green
        self.formats['33'] = {'color': QColor(255, 199, 6)}  # Yellow
        self.formats['34'] = {'color': QColor(0, 111, 184)}  # Blue
        self.formats['35'] = {'color': QColor(118, 38, 113)}  # Megenta
        self.formats['36'] = {'color': QColor(44, 181, 233)}  # Cyan
        self.formats['37'] = {'color': QColor(204, 204, 204)}  # White
        self.formats['39'] = {'color': self._default_color}  # Default foreground color
        self.formats['90'] = {'color': QColor(128, 128, 128)}  # Bright Black
        self.formats['91'] = {'color': QColor(255, 0, 0)}  # Bright Red
        self.formats['92'] = {'color': QColor(0, 255, 0)}  # Bright Green
        self.formats['93'] = {'color': QColor(255, 255, 0)}  # Bright Yellow
        self.formats['94'] = {'color': QColor(0, 0, 255)}  # Bright Blue
        self.formats['95'] = {'color': QColor(255, 0, 255)}  # Bright Magenta
        self.formats['96'] = {'color': QColor(0, 255, 255)}  # Bright Cyan
        self.formats['97'] = {'color': QColor(255, 255, 255)}  # Bright White

        # background colors
        self.formats['40'] = {'color': QColor(1, 1, 1)}  # Black
        self.formats['41'] = {'color': QColor(222, 56, 43)}  # Red
        self.formats['42'] = {'color': QColor(57, 181, 74)}  # Green
        self.formats['43'] = {'color': QColor(255, 199, 6)}  # Yellow
        self.formats['44'] = {'color': QColor(0, 111, 184)}  # Blue
        self.formats['45'] = {'color': QColor(118, 38, 113)}  # Megenta
        self.formats['46'] = {'color': QColor(44, 181, 233)}  # Cyan
        self.formats['47'] = {'color': QColor(204, 204, 204)}  # White
        self.formats['49'] = {'color': self._default_bg}  # Default background color
        self.formats['100'] = {'color': QColor(128, 128, 128)}  # Bright Black
        self.formats['101'] = {'color': QColor(255, 0, 0)}  # Bright Red
        self.formats['102'] = {'color': QColor(0, 255, 0)}  # Bright Green
        self.formats['103'] = {'color': QColor(255, 255, 0)}  # Bright Yellow
        self.formats['104'] = {'color': QColor(0, 0, 255)}  # Bright Blue
        self.formats['105'] = {'color': QColor(255, 0, 255)}  # Bright Magenta
        self.formats['106'] = {'color': QColor(0, 255, 255)}  # Bright Cyan
        self.formats['107'] = {'color': QColor(255, 255, 255)}  # Bright White

    def _create_format(self, color=QColor('#FFFAFA'), style=''):
        _format = QTextCharFormat()
        _format.setForeground(color)
        if 'bold' in style:
            _format.setFontWeight(QFont.Bold)
        else:
            _format.setFontWeight(QFont.Normal)
        if 'italic' in style:
            _format.setFontItalic(True)
        return _format

    def update_format(self, fmt, updates={}):
        if not updates:
            fmt.setForeground(self._default_color)
            fmt.setBackground(self._default_bg)
            fmt.setFontWeight(QFont.Normal)
            fmt.setFontItalic(False)
            fmt.setFontUnderline(False)
        if 'color' in updates:
            fmt.setForeground(updates['color'])
        if 'background' in updates:
            fmt.setBackground(updates['background'])
        if 'style' in updates:
            if updates['style'] == 'bold':
                fmt.setFontWeight(QFont.Bold)
            elif updates['style'] == 'normal':
                fmt.setFontWeight(QFont.Normal)
            elif updates['style'] == 'italic':
                fmt.setFontItalic(updates['style_value'])
            elif updates['style'] == 'underline':
                fmt.setFontUnderline(updates['style_value'])

    def get_formats(self, line):
        parts = line.split('\u001B')
        if len(parts) == 1:
            return [(self.current_format, parts[0])]
        re_code = re.compile(r"\[(?P<code>.*?)m")
        result = []
        new_line = ''
        for part in parts:
            match = re_code.match(part)
            if match:
                if match.group(1) in self.formats:
                    # we change now the format; check first if we have already changes
                    if new_line:
                        result.append((self.current_format, new_line))
                        new_line = ''
                    updates = self.formats[match.group(1)]
                    if not updates:
                        # it is a reset command, create new format
                        self.current_format = self._create_format()
                    else:
                        self.update_format(self.current_format, updates)
                    new_line += part[match.end():]
                    if new_line:
                        result.append((self.current_format, new_line))
                        new_line = ''
                else:
                    # we print out unknown commands
                    new_line += '\u001B%s' % part
            else:
                new_line += part
                if new_line:
                    result.append((self.current_format, new_line))
                    new_line = ''
        if new_line:
            result.append((self.current_format, new_line))
        return result

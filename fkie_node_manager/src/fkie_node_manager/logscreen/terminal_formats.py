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
import rospy


class TerminalFormats(QObject):
    '''
    Defines qt formats for terminal colors.
    Use ubuntu colors from https://en.wikipedia.org/wiki/ANSI_escape_code
    '''

    default_color = QColor('#FFFAFA')
    default_bg = QColor('#010101')
    re_code = re.compile(r"\x1B\[(?P<code>.*?)m")

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        self.current_format = QTextCharFormat()
        self.current_format.setForeground(self.default_color)
        self.formats = {}
        self.formats[0] = {'setForeground': self.default_color,
                           'setBackground': self.default_bg,
                           'setFontWeight': QFont.Normal,
                           'setFontItalic': False,
                           'setFontUnderline': False}  # Normal/Default (reset all attributes)
        self.formats[1] = {'setFontWeight': QFont.Bold}  # Bold/Bright (bold or increased intensity)
        self.formats[2] = {'setFontWeight': QFont.Light}  # Dim/Faint (decreased intensity)
        self.formats[3] = {'setFontItalic': True}  # Italicized (italic on)
        self.formats[4] = {'setFontUnderline': True, 'setUnderlineStyle': QTextCharFormat.SingleUnderline}  # Underscore (single underlined)
        self.formats[5] = {'setFontWeight': QFont.Bold}  # Blink (slow, appears as Bold)
        self.formats[6] = {'setFontWeight': QFont.Black}  # Blink (rapid, appears as very Bold)
        self.formats[22] = {'setFontWeight': QFont.Normal}
        self.formats[23] = {'setFontItalic': False}
        self.formats[24] = {'setFontUnderline': False}
        # foreground colors
        self.formats[30] = {'setForeground': QColor(1, 1, 1)}  # Black
        self.formats[31] = {'setForeground': QColor(222, 56, 43)}  # Red
        self.formats[32] = {'setForeground': QColor(57, 181, 74)}  # Green
        self.formats[33] = {'setForeground': QColor(255, 199, 6)}  # Yellow
        self.formats[34] = {'setForeground': QColor(0, 111, 184)}  # Blue
        self.formats[35] = {'setForeground': QColor(118, 38, 113)}  # Megenta
        self.formats[36] = {'setForeground': QColor(44, 181, 233)}  # Cyan
        self.formats[37] = {'setForeground': QColor(204, 204, 204)}  # White
        self.formats[39] = {'setForeground': self.default_color}  # Default foreground color
        self.formats[90] = {'setForeground': QColor(128, 128, 128)}  # Bright Black
        self.formats[91] = {'setForeground': QColor(255, 0, 0)}  # Bright Red
        self.formats[92] = {'setForeground': QColor(0, 255, 0)}  # Bright Green
        self.formats[93] = {'setForeground': QColor(255, 255, 0)}  # Bright Yellow
        self.formats[94] = {'setForeground': QColor(0, 0, 255)}  # Bright Blue
        self.formats[95] = {'setForeground': QColor(255, 0, 255)}  # Bright Magenta
        self.formats[96] = {'setForeground': QColor(0, 255, 255)}  # Bright Cyan
        self.formats[97] = {'setForeground': QColor(255, 255, 255)}  # Bright White
        # background colors
        self.formats[40] = {'setBackground': QColor(1, 1, 1)}  # Black
        self.formats[41] = {'setBackground': QColor(222, 56, 43)}  # Red
        self.formats[42] = {'setBackground': QColor(57, 181, 74)}  # Green
        self.formats[43] = {'setBackground': QColor(255, 199, 6)}  # Yellow
        self.formats[44] = {'setBackground': QColor(0, 111, 184)}  # Blue
        self.formats[45] = {'setBackground': QColor(118, 38, 113)}  # Megenta
        self.formats[46] = {'setBackground': QColor(44, 181, 233)}  # Cyan
        self.formats[47] = {'setBackground': QColor(204, 204, 204)}  # White
        self.formats[49] = {'setBackground': self.default_bg}  # Default background color
        self.formats[100] = {'setBackground': QColor(128, 128, 128)}  # Bright Black
        self.formats[101] = {'setBackground': QColor(255, 0, 0)}  # Bright Red
        self.formats[102] = {'setBackground': QColor(0, 255, 0)}  # Bright Green
        self.formats[103] = {'setBackground': QColor(255, 255, 0)}  # Bright Yellow
        self.formats[104] = {'setBackground': QColor(0, 0, 255)}  # Bright Blue
        self.formats[105] = {'setBackground': QColor(255, 0, 255)}  # Bright Magenta
        self.formats[106] = {'setBackground': QColor(0, 255, 255)}  # Bright Cyan
        self.formats[107] = {'setBackground': QColor(255, 255, 255)}  # Bright White

    def _update_format(self, fmt, updates={}):
        for attr, args in updates.items():
            try:
                if isinstance(args, list):
                    getattr(fmt, attr)(*args)
                else:
                    getattr(fmt, attr)(args)
            except AttributeError:
                pass

    def insert_formated(self, cursor, text, char_format=None):
        cursor.beginEditBlock()
        current_char_format = char_format
        if current_char_format is None:
            current_char_format = cursor.charFormat()
        cidx = 0
        for match in self.re_code.finditer(text):
            code = -1
            try:
                code = int(match.group(1))
            except Exception:
                pass
            cursor.insertText(text[cidx:match.start()], current_char_format)
            if code in self.formats:
                try:
                    self._update_format(current_char_format, self.formats[code])
                except Exception as err:
                    rospy.logwarn("Failed update format for ANSI_escape_code %d: %s" % (code, err))
            cidx = match.end()
        cursor.insertText(text[cidx:], current_char_format)
        cursor.movePosition(QTextCursor.End)
        cursor.setCharFormat(current_char_format)
        cursor.endEditBlock()
        return current_char_format


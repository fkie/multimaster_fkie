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



from python_qt_binding.QtCore import QPoint, QSize
from python_qt_binding.QtGui import QAbstractTextDocumentLayout, QFontMetrics, QPalette, QTextDocument
try:
    from python_qt_binding.QtGui import QApplication, QStyledItemDelegate, QStyle
    from python_qt_binding.QtGui import QStyleOptionViewItemV4 as QStyleOptionViewItem
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QStyledItemDelegate, QStyle
    from python_qt_binding.QtWidgets import QStyleOptionViewItem

from rosgraph.names import is_legal_name


class HTMLDelegate(QStyledItemDelegate):
    '''
    A class to display the HTML text in QTreeView.
    '''
    def __init__(self, parent=None, check_for_ros_names=True, dec_ascent=False, is_node=False, palette=None):
        QStyledItemDelegate.__init__(self, parent)
        self._check_for_ros_names = check_for_ros_names
        self._cached_size = None
        self._red_ascent = 4 if not dec_ascent else 2
        self._dec_ascent = dec_ascent
        self._is_node = is_node
        self._palette = palette

    def paint(self, painter, option, index):
        '''
        Use the QTextDokument to represent the HTML text.
        @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
        '''
        options = QStyleOptionViewItem(option)
        self.initStyleOption(options, index)

        style = QApplication.style() if options.widget is None else options.widget.style()

        doc = QTextDocument()
        doc.setHtml(self.toHTML(options.text, self._check_for_ros_names, self._is_node, self._palette))

        options.text = ''
        style.drawControl(QStyle.CE_ItemViewItem, options, painter)

        ctx = QAbstractTextDocumentLayout.PaintContext()

        # Highlighting text if item is selected
        # if (optionV4.state and QStyle::State_Selected):
        #  ctx.palette.setColor(QPalette::Text, optionV4.palette.color(QPalette::Active, QPalette::HighlightedText));

        textRect = style.subElementRect(QStyle.SE_ItemViewItemText, options, options.widget)
        if textRect.width() < 10:
            textRect.setWidth(options.rect.width())
            textRect.setHeight(options.rect.height())
        painter.save()
        red = self._red_ascent if not self._dec_ascent else self._red_ascent / 2
        painter.translate(QPoint(textRect.topLeft().x(), textRect.topLeft().y() - red))
        painter.setClipRect(textRect.translated(-textRect.topLeft()))
        doc.documentLayout().draw(painter, ctx)

        painter.restore()

    def sizeHint(self, option, index):
        '''
        Determines and returns the size of the text after the format.
        @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
        '''
        if self._cached_size is not None:
            return self._cached_size
        options = QStyleOptionViewItem(option)
        self.initStyleOption(options, index)
        doc = QTextDocument()
        doc.setHtml(options.text)
        doc.setTextWidth(options.rect.width())
        metric = QFontMetrics(doc.defaultFont())
        self._red_ascent = abs(metric.height() - metric.ascent()) + 1
        self._cached_size = QSize(doc.idealWidth(), metric.height() + self._red_ascent)
        return self._cached_size

    @classmethod
    def toHTML(cls, text, check_for_ros_names=True, is_node=False, palette=None):
        '''
        Creates a HTML representation of the given text. It could be a node, topic service or group name.
        :param str text: a name with ROS representation
        :return: the HTML representation of the given name
        :rtype: str
        '''
        if text.rfind('@') > 0:  # handle host names
            name, sep, host = text.rpartition('@')
            result = ''
            if sep:
                result = '%s<span style="color:%s;">%s%s</span>' % (name, cls.color_name(palette, QPalette.ButtonText), sep, host)
            else:
                result = text
        elif text.find('{') > -1:  # handle group names
            text = text.strip('{}')
            ns, sep, name = text.rpartition('/')
            result = ''
            if sep:
                result = '{<span style="color:%s;">%s%s</span>%s}' % (cls.color_name(palette, QPalette.ButtonText), ns, sep, name)
            else:
                result = '<span style="color:%s;">{%s}</span>' % (cls.color_name(palette, QPalette.ButtonText), name)
#                result = '<b>{</b><span style="color:gray;">%s</span><b>}</b>' % (name)
#                result = '<b>{%s}</b>' % (name)
        elif text.find('[') > -1:
            start_idx = text.find('[')
            end_idx = text.find(']', start_idx)
            nr_idx = text.find(':')
            last_part = ""
            color = cls.color_name(palette, QPalette.ButtonText)
            if end_idx + 1 < len(text):
                last_part = text[end_idx + 1:]
            if nr_idx > -1 and nr_idx < start_idx:
                result = '%s<b>%s</b><span style="color:%s;">%s</span><b>%s</b>' % (text[0:nr_idx + 1], text[nr_idx + 1:start_idx], color, text[start_idx:end_idx + 1], last_part)
            else:
                result = '<b>%s</b><span style="color:%s;">%s</span><b>%s</b>' % (text[0:start_idx], color, text[start_idx:end_idx + 1], last_part)
        elif text.startswith('<arg_not_set>'):
            result = '<span style="color:#0000FF;">%s</span>' % (text.replace('<arg_not_set>', ''))
        elif text.startswith('<arg>'):
            result = text.replace('<arg>', '')
        elif check_for_ros_names and not is_legal_name(text):  # handle all invalid names (used space in the name)
            ns, sep, name = text.rpartition('/')
            result = ''
            if sep:
                result = '<span style="color:#FF6600;">%s%s<b>%s</b></span' % (ns, sep, name)
            else:
                result = '<span style="color:#FF6600;">%s</span>' % (name)
        else:  # handle all ROS names
            ns, sep, name = text.rpartition('/')
            result = ''
            if sep:
                result = '<span style="color:%s;">%s%s</span><b>%s</b>' % (cls.color_name(palette, QPalette.ButtonText), ns, sep, name)
            elif is_node:
                result = '<b>%s</b>' % name
            else:
                result = name
        return result

    @classmethod
    def color_name(cls, palette, color_type=QPalette.Text):
        if palette is not None:
            return palette.color(color_type).name()
        elif color_type == QPalette.ButtonText:
            return 'gray'
#            return '#3c3c3c'

# The MIT License
#
# Copyright (c) 2009 John Schember <john@nachtimwald.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE



from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPainter
try:
    from python_qt_binding.QtGui import QWidget, QFrame, QHBoxLayout
except Exception:
    from python_qt_binding.QtWidgets import QWidget, QFrame, QHBoxLayout


class LineNumberWidget(QFrame):

    class NumberBar(QWidget):

        def __init__(self, *args):
            QWidget.__init__(self, *args)
            self.edit = None
            # it is the highest line that is currently visible.
            self.highest_line = 0

        def set_text_edit(self, edit):
            self.edit = edit

        def update(self, *args):
            # the +4 is used to compensate for the current line being bold.
            width = self.fontMetrics().width(str(self.highest_line)) + 4
            if self.width() != width:
                self.setFixedWidth(width)
            QWidget.update(self, *args)

        def paintEvent(self, event):
            contents_y = self.edit.verticalScrollBar().value()
            page_bottom = contents_y + self.edit.viewport().height()
            font_metrics = self.fontMetrics()
            current_block = self.edit.document().findBlock(self.edit.textCursor().position())
            painter = QPainter(self)
            painter.setPen(Qt.darkGray)
            line_count = 0
            first_pos = -1
            posdiff = 0
            # Iterate over all text blocks in the document.
            block = self.edit.document().begin()
            while block.isValid():
                line_count += 1
                # the top left position of the block in the document
                if posdiff == 0:
                    position_y = self.edit.document().documentLayout().blockBoundingRect(block).topLeft().y()
                    if first_pos == -1:
                        first_pos = position_y
                    else:
                        posdiff = position_y - first_pos
                else:
                    position_y += posdiff
                # check if the position of the block is out side of visible area
                if position_y > page_bottom:
                    break
                # we want the line number for the selected line to be bold.
                bold = False
                if block == current_block:
                    bold = True
                    font = painter.font()
                    font.setBold(True)
                    painter.setFont(font)
                    painter.setPen(Qt.black)
                # Draw the line number right justified at the y position of the
                # line. 3 is the magic padding number. drawText(x, y, text)
                painter.drawText(self.width() - font_metrics.width(str(line_count)) - 3, position_y - contents_y + font_metrics.ascent(), str(line_count))
                if bold:
                    font = painter.font()
                    font.setBold(False)
                    painter.setFont(font)
                    painter.setPen(Qt.darkGray)

                block = block.next()

            self.highest_line = line_count
            painter.end()
            QWidget.paintEvent(self, event)

    def __init__(self, editor, *args):
        QFrame.__init__(self, *args)

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.edit = editor

        self.number_bar = self.NumberBar()
        self.number_bar.set_text_edit(self.edit)

        hbox = QHBoxLayout(self)
        hbox.setSpacing(0)
        hbox.setContentsMargins(0, 0, 0, 0)
        # hbox.setMargin(0) # removed: it is not supported by Qt5
        hbox.addWidget(self.number_bar)
        hbox.addWidget(self.edit)

        self.edit.installEventFilter(self)
        self.edit.viewport().installEventFilter(self)

    def eventFilter(self, obj, event):
        # Update the line numbers for all events on the text edit and the viewport.
        # This is easier than connecting all necessary signals.
        try:
            if obj in (self.edit, self.edit.viewport()):
                self.number_bar.update()
                return False
            return QFrame.eventFilter(obj, event)
        except Exception:
            pass

    def get_text_edit(self):
        return self.edit

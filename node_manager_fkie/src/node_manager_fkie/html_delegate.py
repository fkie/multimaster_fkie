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
from python_qt_binding import QtCore

class HTMLDelegate(QtGui.QStyledItemDelegate):
  '''
  A class to display the HTML text in QTreeView.
  '''
  
  def paint(self, painter, option, index):
    '''
    Use the QTextDokument to represent the HTML text.
    @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
    '''
    options = QtGui.QStyleOptionViewItemV4(option)
    self.initStyleOption(options, index)

    style = QtGui.QApplication.style() if options.widget is None else options.widget.style()

    doc = QtGui.QTextDocument()
    doc.setHtml(self.toHTML(options.text))
    doc.setTextWidth(option.rect.width())

    options.text = ''
    style.drawControl(QtGui.QStyle.CE_ItemViewItem, options, painter);

    ctx = QtGui.QAbstractTextDocumentLayout.PaintContext()

    #Highlighting text if item is selected
    #if (optionV4.state and QStyle::State_Selected):
    #  ctx.palette.setColor(QPalette::Text, optionV4.palette.color(QPalette::Active, QPalette::HighlightedText));

    textRect = style.subElementRect(QtGui.QStyle.SE_ItemViewItemText, options, options.widget)
    painter.save()
    painter.translate(QtCore.QPoint(textRect.topLeft().x(), textRect.topLeft().y()-3))
    painter.setClipRect(textRect.translated(-textRect.topLeft()))
    doc.documentLayout().draw(painter, ctx)

    painter.restore()

  def sizeHint(self, option, index):
    '''
    Determines and returns the size of the text after the format.
    @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
    '''
    options = QtGui.QStyleOptionViewItemV4(option)
    self.initStyleOption(options,index)

    doc = QtGui.QTextDocument()
    doc.setHtml(options.text)
    doc.setTextWidth(options.rect.width())
    metric = QtGui.QFontMetrics(doc.defaultFont())
    return QtCore.QSize(doc.idealWidth(), metric.height()+4)

  @classmethod
  def toHTML(cls, text):
    '''
    Creates a HTML representation of the topic name.
    @param topic_name: the topic name
    @type topic_name: C{str}
    @return: the HTML representation of the topic name
    @rtype: C{str}
    '''
    if text.rfind('@') > 0: # handle host names
      name, sep, host = text.rpartition('@')
      result = ''
      if sep:
        result = ''.join(['<div>', name, '<span style="color:gray;">', sep, host, '</span></div>'])
      else:
        result = group_name
    elif text.find('{') > -1: # handle group names 
      text = text.strip('{}')
      ns, sep, name = text.rpartition('/')
      result = ''
      if sep:
        result = ''.join(['<div>', '<b>{</b><span style="color:gray;">', ns, sep, '</span><b>', name, '}</b></div>'])
      else:
        result = ''.join(['<div>', '<b>{', name, '}</b></div>'])
    elif text.find(' ') > -1: # handle all invalid names (used space in the name)
      ns, sep, name = text.rpartition('/')
      result = ''
      if sep:
        result = ''.join(['<div>', '<span style="color:red;">', str(ns), sep, '<b>', name, '</b></span></div>'])
      else:
        result = ''.join(['<div>', '<span style="color:red;">', name, '</span></div>'])
    else: # handle all ROS names
      ns, sep, name = text.rpartition('/')
      result = ''
      if sep:
        result = ''.join(['<div>', '<span style="color:gray;">', str(ns), sep, '</span><b>', name, '</b></div>'])
      else:
        result = name
    return result

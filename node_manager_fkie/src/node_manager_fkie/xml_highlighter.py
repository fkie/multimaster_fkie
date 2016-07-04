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

from python_qt_binding import QtCore
from python_qt_binding import QtGui


class XmlHighlighter(QtGui.QSyntaxHighlighter):
  '''
  Enabled the syntax highlightning for the ROS launch files.
  '''

  LAUNCH_LAUNCH_CHILDS = ['group', 'node', 'test', 'env', 'remap', 'rosparam', 'param', 'machine', 'include', 'arg']
  LAUNCH_LAUNCH_ATTR = {'deprecated='  : '"message"'}
  LAUNCH_GROUP_CHILDS = ['node', 'test', 'env', 'remap', 'rosparam', 'param', 'machine', 'include', 'arg']
  LAUNCH_GROUP_ATTR = {'ns='           : '"foo"',
                       'clear_params=' : '"true|false"'
                       }
  LAUNCH_MACHINE_CHILDS = ['env']
  LAUNCH_MACHINE_ATTR = {'name='       : '"machine-name"',
                         'address='    : '"blah.willowgarage.com"',
                         'env-loader=' : '"/opt/ros/fuerte/env.sh"',
                         'default='    : '"true|false|never"',
                         'user='       : '"username"',
                         'password='   : '"passwhat"',
                         'timeout='    : '"10.0"'
                       }
  LAUNCH_NODE_CHILDS = ['env', 'remap', 'rosparam', 'param']
  LAUNCH_NODE_ATTR = {'pkg='          : '"mypackage"',
                      'type='         : '"nodetype"',
                      'name='         : '"nodename"',
                      'args='         : '"arg1"',
                      'machine='      : '"machine-name"',
                      'respawn='      : '"true"',
                      'required='     : '"true"',
                      'ns='           : '"foo"',
                      'clear_params=' : '"true|false"',
                      'output='       : '"log|screen"',
                      'cwd='          : '"ROS_HOME|node"',
                      'launch-prefix=': '"prefix arguments"'
                      }
  LAUNCH_INCLUDE_CHILDS = ['env', 'arg']
  LAUNCH_INCLUDE_ATTR = {'file='         : '"$(find pkg-name)/path/filename.xml"',
                         'ns='           : '"foo"',
                         'clear_params=' : '"true|false"'
                         }

  LAUNCH_REMAP_ATTR = {'from=' : '"originalname"',
                       'to='   : '"newname"'
                       }
  LAUNCH_ENV_ATTR = {'name='  : '"name"',
                     'value=' : '"value"'
                     }
  LAUNCH_PARAM_ATTR = {'name='     : '"namespace/name"',
                       'value='    : '"value"',
                       'type='     : '"str|int|double|bool"',
                       'textfile=' : '"$(find pkg-name)/path/file.txt"',
                       'binfile='  : '"$(find pkg-name)/path/file"',
                       'command='  : '"$(find pkg-name)/exe \'$(find pkg-name)/arg.txt\'"'
                       }

  LAUNCH_ROSPARAM_ATTR = {'command=' : '"load|dump|delete"',
                          'file='    : '"$(find pkg-name)/path/foo.yaml"',
                          'param='   : '"name"',
                          'ns='      : '"foo"'
                          }
  LAUNCH_ARG_ATTR = {'name='     : '"name"',
                     'value='    : '"bar"',
                     'default='  : '"defbar"'
                     }
  LAUNCH_TEST_CHILDS = ['env', 'remap', 'rosparam', 'param']
  LAUNCH_TEST_ATTR = {'pkg='          : '"mypackage"',
                      'type='         : '"nodetype"',
                      'name='         : '"nodename"',
                      'test-name='    : '"test_name"',
                      'args='         : '"arg1"',
                      'ns='           : '"foo"',
                      'clear_params=' : '"true|false"',
                      'retry='        : '"0"',
                      'cwd='          : '"ROS_HOME|node"',
                      'launch-prefix=': '"prefix arguments"',
                      'time-limit='   : '"60.0"'
                      }

  LAUNCH_CHILDS = {'launch'   : LAUNCH_LAUNCH_CHILDS,
                   'group'    : LAUNCH_GROUP_CHILDS,
                   'machine'  : LAUNCH_MACHINE_CHILDS,
                   'node'     : LAUNCH_NODE_CHILDS,
                   'include'  : LAUNCH_INCLUDE_CHILDS,
                   'remap'    : [],
                   'env'      : [],
                   'param'    : [],
                   'rosparam' : [],
                   'arg'      : [],
                   'test'     : LAUNCH_TEST_CHILDS
                   }

  LAUNCH_ATT_GLOBAL = {'if=': '""', 'unless=': '""'}
  LAUNCH_LAUNCH_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_GROUP_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_MACHINE_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_NODE_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_INCLUDE_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_REMAP_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_ENV_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_PARAM_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_ROSPARAM_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_ARG_ATTR.update(LAUNCH_ATT_GLOBAL)
  LAUNCH_TEST_ATTR.update(LAUNCH_ATT_GLOBAL)

  LAUNCH_ATTR = {'launch'   : LAUNCH_LAUNCH_ATTR,
                   'group'    : LAUNCH_GROUP_ATTR,
                   'machine'  : LAUNCH_MACHINE_ATTR,
                   'node'     : LAUNCH_NODE_ATTR,
                   'include'  : LAUNCH_INCLUDE_ATTR,
                   'remap'    : LAUNCH_REMAP_ATTR,
                   'env'      : LAUNCH_ENV_ATTR,
                   'param'    : LAUNCH_PARAM_ATTR,
                   'rosparam' : LAUNCH_ROSPARAM_ATTR,
                   'arg'      : LAUNCH_ARG_ATTR,
                   'test'     : LAUNCH_TEST_ATTR,
                   }

  def __init__(self, parent=None):
    QtGui.QSyntaxHighlighter.__init__(self, parent)
    self.rules = []
    self.commentStart = QtCore.QRegExp("<!--")
    self.commentEnd = QtCore.QRegExp("-->")
    self.default_format = QtGui.QTextCharFormat()
    self.default_format.setForeground(QtGui.QColor(24, 24, 24))
    self.mark_background = QtGui.QBrush(QtGui.QColor(251, 247, 222))
    self.commentFormat = QtGui.QTextCharFormat()
    f = QtGui.QTextCharFormat()
    r = QtCore.QRegExp()
    r.setMinimal(True)
    f.setFontWeight(QtGui.QFont.Normal)
    f.setForeground(QtCore.Qt.darkBlue)
    # create patterns for TAG
    tagList = ["\\b%s\\b" % t for t in self.LAUNCH_CHILDS.keys()]
    for tag in tagList:
      r.setPattern(tag)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for ATTRIBUTES
    f.setForeground(QtCore.Qt.darkGreen)
    attrList = set(["\\b%s" % attr for v in self.LAUNCH_ATTR.values() for attr in v.keys()])
    for attr in attrList:
      r.setPattern(attr)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for strings
    f.setForeground(QtCore.Qt.magenta)
    r.setPattern("\".*\"")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for substitutions
    f.setForeground(QtGui.QColor(127, 64, 127))
    r.setPattern("\\$\\(.*\\)")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for DOCTYPE
    f.setForeground (QtCore.Qt.lightGray)
    r.setPattern("<!DOCTYPE.*>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    r.setPattern("<\\?xml.*\\?>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    self.commentFormat.setFontItalic(True)
    self.commentFormat.setForeground(QtCore.Qt.darkGray)

    # part to select an XML block
    self._current_mark_range = (-1, -1)  # absolute (start, end) positions

  def highlightBlock(self, text):
    self._format(0, len(text), self.default_format)
    for pattern, form in self.rules:
      index = pattern.indexIn(text)
      while index >= 0:
        length = pattern.matchedLength()
        self._format(index, index + length, form)
        index = pattern.indexIn(text, index + length)
    # detection for comments
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
      self._format(startIndex, startIndex + commentLength, self.commentFormat)
      startIndex = self.commentStart.indexIn(text, startIndex + commentLength)

  def _format(self, start, end, frmt):
    start += self.currentBlock().position()
    end += self.currentBlock().position()
    mark = {}
    do_mark = self._inrange(start, self._current_mark_range)
    for val in range(start + 1, end):
      res = self._inrange(val, self._current_mark_range)
      if do_mark != res:
        mark[val] = res
        do_mark = res
    do_end_mark = do_mark
    last_pos = start
    for pos, do_mark in mark.items():
      if not do_mark:
        frmt.setBackground(self.mark_background)
      else:
        frmt.clearBackground()
      self.setFormat(last_pos - self.currentBlock().position(), pos - last_pos, frmt)
      last_pos = pos
    if do_end_mark:
      frmt.setBackground(self.mark_background)
    else:
      frmt.clearBackground()
    self.setFormat(last_pos - self.currentBlock().position(), end - last_pos, frmt)
    return frmt

  def _inrange(self, value, (start, end)):
    return value >= start and value <= end

  def mark_tag_block(self, position):
    '''
    Select an XML block, depending on the cursor position.
    '''
    text = self.document().toPlainText()
    search_tag, open_pos, close_pos = self.get_tag_of_current_block(text, position)
    force_clear = not search_tag and close_pos != -1
    # now let mark the block
    new_block = open_pos != -1 and close_pos != -1
    if self._current_mark_range[1] != -1 and (new_block or force_clear):
      # demark old block
      first_block = self.document().findBlock(self._current_mark_range[0])
      end_block = self.document().findBlock(self._current_mark_range[1])
      self._current_mark_range = (-1, -1)
      for blocknr in range(first_block.blockNumber(), end_block.blockNumber() + 1):
        block = self.document().findBlockByNumber(blocknr)
        self.rehighlightBlock(block)
    if new_block and search_tag != 'launch':
      # mark the block
      self._current_mark_range = (open_pos, close_pos)
      first_block = self.document().findBlock(open_pos)
      end_block = self.document().findBlock(close_pos)
      for blocknr in range(first_block.blockNumber(), end_block.blockNumber() + 1):
        block = self.document().findBlockByNumber(blocknr)
        self.rehighlightBlock(block)

  def get_tag_of_current_block(self, text, position):
    pos = position
    if len(text) <= pos:
      return '', -1, -1
    open_pos = -1
    close_pos = -1
    search_tag = ''
    # find the start position of the start/end tag
    closed_tags = 0
    has_close_brace = False
    try:
      while text[pos] not in ['<'] or closed_tags > 0:
        if text[pos - 2:pos] in ['/>', '->']:
          closed_tags += 1
          pos -= 2
        elif text[pos - 2:pos] in ['</'] and has_close_brace:
          # handle the case the cursor is in the name of closed tag
          has_close_brace = False
          closed_tags += 1
          pos -= 2
        elif text[pos] == '<':
          closed_tags -= 1
        elif text[pos] == '>':
          has_close_brace = True
        pos -= 1
      if text[pos] == '<':
        open_pos = pos
      elif text[pos] == '>':
        close_pos = pos
      else:
        pos = position
    except:
      pass
    if open_pos > -1:
      # the start position was found, determine the tag and end of the group
      try:
        while text[pos] not in [' ', '!', '>']:
          pos += 1
        if text[pos] == '!':
          # skip comment start
          pass
        elif text[pos] == '>' and text[open_pos + 1] == '/':
          # it is an end tag: determine the tag name and searches for start
          search_tag = text[open_pos + 2:pos]
          open_tag_pos = text.rfind('<%s' % search_tag, 0, open_pos)
          if open_tag_pos > -1:
            close_pos = pos + 1
            open_pos = open_tag_pos
        elif text[pos] == ' ' or text[pos] == '>':
          # it is a start tag: determine the name and search for end
          search_tag = text[open_pos + 1:pos]
          while (text[pos + 1:pos + 3] not in ['/>']) and ('>' not in text[pos:pos + 2]):
            pos += 1
          if text[pos] == '>' or text[pos + 1] == '>':
            close_tag_pos = text.find('</%s>' % search_tag, pos)
            if close_tag_pos > -1:
              close_pos = close_tag_pos + len(search_tag) + 3
          else:
            # it is closed by />
            close_pos = pos + 2
      except:
        pass
    return search_tag, open_pos, close_pos

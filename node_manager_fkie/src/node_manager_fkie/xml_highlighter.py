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

  LAUNCH_ATT_GLOBAL = {'if=' : '""', 'unless=' : '""'}
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

  LAUNCH_ATTR  =  {'launch'   : LAUNCH_LAUNCH_ATTR,
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
    self.commentFormat = QtGui.QTextCharFormat()
    f = QtGui.QTextCharFormat()
    r = QtCore.QRegExp()
    r.setMinimal(True)
    f.setFontWeight(QtGui.QFont.Normal)
    f.setForeground (QtCore.Qt.darkBlue)
    # create patterns for TAG
    tagList = ["\\b%s\\b"%t for t in self.LAUNCH_CHILDS.keys()]
    for tag in tagList:
      r.setPattern(tag)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for ATTRIBUTES
    f.setForeground(QtCore.Qt.darkGreen)
    attrList = set(["\\b%s"%attr for v in self.LAUNCH_ATTR.values() for attr in v.keys()])
    for attr in attrList:
      r.setPattern(attr)
      self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for strings
    f.setForeground(QtCore.Qt.magenta)
    r.setPattern("\".*\"")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for substitutions
    f.setForeground(QtGui.QColor(127,64,127))
    r.setPattern ("\\$\\(.*\\)")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    # create patterns for DOCTYPE
    f.setForeground (QtCore.Qt.lightGray)
    r.setPattern ("<!DOCTYPE.*>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))
    r.setPattern ("<\\?xml.*\\?>")
    self.rules.append((QtCore.QRegExp(r), QtGui.QTextCharFormat(f)))

    self.commentFormat.setFontItalic(True)
    self.commentFormat.setForeground(QtCore.Qt.darkGray)


  def highlightBlock(self, text):
    for pattern, form in self.rules:
      index = pattern.indexIn(text)
      while index >= 0:
        length = pattern.matchedLength()
        self.setFormat(index, length, form)
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

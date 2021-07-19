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
from python_qt_binding.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat


class XmlHighlighter(QSyntaxHighlighter):
    '''
    Enabled the syntax highlightning for the ROS launch files.
    '''

    LAUNCH_LAUNCH_CHILDS = ['group', 'node', 'test', 'env', 'remap', 'rosparam', 'param', 'machine', 'include', 'arg']
    LAUNCH_LAUNCH_ATTR = {'deprecated=': '"message"'}
    LAUNCH_GROUP_CHILDS = ['node', 'test', 'env', 'remap', 'rosparam', 'param', 'machine', 'include', 'arg']
    LAUNCH_GROUP_ATTR = {'ns=': '"foo"',
                         'clear_params=': '"true|false"'
                         }
    LAUNCH_MACHINE_CHILDS = ['env']
    LAUNCH_MACHINE_ATTR = {'name=': '"machine-name"',
                           'address=': '"blah.willowgarage.com"',
                           'env-loader=': '"/opt/ros/fuerte/env.sh"',
                           'default=': '"true|false|never"',
                           'user=': '"username"',
                           'password=': '"passwhat"',
                           'timeout=': '"10.0"'
                           }
    LAUNCH_NODE_CHILDS = ['env', 'remap', 'rosparam', 'param']
    LAUNCH_NODE_ATTR = {'pkg=': '"mypackage"',
                        'type=': '"nodetype"',
                        'name=': '"nodename"',
                        'args=': '"arg1"',
                        'machine=': '"machine-name"',
                        'respawn=': '"true"',
                        'required=': '"true"',
                        'ns=': '"foo"',
                        'clear_params=': '"true|false"',
                        'output=': '"log|screen"',
                        'cwd=': '"ROS_HOME|node"',
                        'launch-prefix=': '"prefix arguments"'
                        }
    LAUNCH_INCLUDE_CHILDS = ['env', 'arg']
    LAUNCH_INCLUDE_ATTR = {'file=': '"$(find pkg-name)/path/filename.xml"',
                           'ns=': '"foo"',
                           'clear_params=': '"true|false"',
                           'pass_all_args=': '"true|false"'
                           }

    LAUNCH_REMAP_ATTR = {'from=': '"originalname"',
                         'to=': '"newname"'
                         }
    LAUNCH_ENV_ATTR = {'name=': '"name"',
                       'value=': '"value"'
                       }
    LAUNCH_PARAM_ATTR = {'name=': '"namespace/name"',
                         'value=': '"value"',
                         'type=': '"str|int|double|bool"',
                         'textfile=': '"$(find pkg-name)/path/file.txt"',
                         'binfile=': '"$(find pkg-name)/path/file"',
                         'command=': '"$(find pkg-name)/exe \'$(find pkg-name)/arg.txt\'"'
                         }

    LAUNCH_ROSPARAM_ATTR = {'command=': '"load|dump|delete"',
                            'file=': '"$(find pkg-name)/path/foo.yaml"',
                            'param=': '"name"',
                            'ns=': '"foo"',
                            'subst_value=': '"true|false"'
                            }
    LAUNCH_ARG_ATTR = {'name=': '"name"',
                       'value=': '"bar"',
                       'default=': '"defbar"'
                       }
    LAUNCH_TEST_CHILDS = ['env', 'remap', 'rosparam', 'param']
    LAUNCH_TEST_ATTR = {'pkg=': '"mypackage"',
                        'type=': '"nodetype"',
                        'name=': '"nodename"',
                        'test-name=': '"test_name"',
                        'args=': '"arg1"',
                        'ns=': '"foo"',
                        'clear_params=': '"true|false"',
                        'retry=': '"0"',
                        'cwd=': '"ROS_HOME|node"',
                        'launch-prefix=': '"prefix arguments"',
                        'time-limit=': '"60.0"'
                        }

    LAUNCH_CHILDS = {'launch': LAUNCH_LAUNCH_CHILDS,
                     'group': LAUNCH_GROUP_CHILDS,
                     'machine': LAUNCH_MACHINE_CHILDS,
                     'node': LAUNCH_NODE_CHILDS,
                     'include': LAUNCH_INCLUDE_CHILDS,
                     'remap': [],
                     'env': [],
                     'param': [],
                     'rosparam': [],
                     'arg': [],
                     'test': LAUNCH_TEST_CHILDS
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

    LAUNCH_ATTR = {'launch': LAUNCH_LAUNCH_ATTR,
                   'group': LAUNCH_GROUP_ATTR,
                   'machine': LAUNCH_MACHINE_ATTR,
                   'node': LAUNCH_NODE_ATTR,
                   'include': LAUNCH_INCLUDE_ATTR,
                   'remap': LAUNCH_REMAP_ATTR,
                   'env': LAUNCH_ENV_ATTR,
                   'param': LAUNCH_PARAM_ATTR,
                   'rosparam': LAUNCH_ROSPARAM_ATTR,
                   'arg': LAUNCH_ARG_ATTR,
                   'test': LAUNCH_TEST_ATTR,
                   }

    DEPRECATED_PARAMETER = {'associations': 'nm/associations',
                            'kill_on_stop': 'nm/kill_on_stop',
                            }
    STATE_COMMENT = 2
    STATE_STRING = 4

    def __init__(self, parent=None, is_launch=True):
        QSyntaxHighlighter.__init__(self, parent)
        self._is_launch = is_launch
        self.rules = []
        self.comment_start = QRegExp("<!--")
        self.comment_end = QRegExp("-->")
        self.comment_format = self._create_format(Qt.darkGray, 'italic')
#        self.mark_background = QBrush(QColor(251, 247, 222))
        # create patterns for braces
        self.rules.append((self._create_regexp("</?|/?>"), self._create_format(QColor(24, 24, 24))))
        # create patterns for TAG
        if self._is_launch:
            tag_list = '|'.join(["\\b%s\\b" % t for t in self.LAUNCH_CHILDS.keys()])
            self.rules.append((self._create_regexp(tag_list), self._create_format(Qt.darkRed)))
        else:
            self.rules.append((self._create_regexp(">|/>|<[/.\w:]*[\s\t>]|<[/.\w:]*$"), self._create_format(Qt.darkRed)))
        # create patterns for ATTRIBUTES
        if self._is_launch:
            attr_list = '|'.join(set(["\\b%s" % attr for v in self.LAUNCH_ATTR.values() for attr in v.keys()]))
            self.rules.append((self._create_regexp(attr_list), self._create_format(QColor(0, 100, 0))))  # darkGreen
        else:
            self.rules.append((self._create_regexp("[_.\w]*="), self._create_format(QColor(0, 100, 0))))  # darkGreen
        # create patterns for substitutions
        self.rule_arg = (self._create_regexp("\\$\\(.*\\)"), self._create_format(QColor(77, 0, 38)))
        # create patterns for DOCTYPE
        self.rules.append((self._create_regexp("<!DOCTYPE.*>"), self._create_format(Qt.lightGray)))
        self.rules.append((self._create_regexp("<\\?xml.*\\?>"), self._create_format(Qt.lightGray)))
        # create patterns for yaml parameter inside
        self.rules.append((self._create_regexp("[_.\w]*\s*:"), self._create_format(Qt.darkBlue)))
        # create patterns for yaml oneline strings inside
        self.rules.append((self._create_regexp("'.*'"), self._create_format(Qt.blue)))
        # create pattern for list signes
        self.rules.append((self._create_regexp("^\s*-"), self._create_format(Qt.darkRed, 'bold')))
        # create pattern for digits
        self.rules.append((self._create_regexp("\\d+"), self._create_format(QColor(127, 64, 127))))
        self.yaml_comment_rule = (self._create_regexp("#[.]*"), self._create_format(Qt.darkGray))
        # create deprecated
        self.dep_pattern = []
        if self.DEPRECATED_PARAMETER:
            attr_list = '|'.join(set([r'name="%s"' % attr for attr in self.DEPRECATED_PARAMETER.keys()]))
            # print(attr_list)
            self.dep_pattern.append((self._create_regexp(attr_list), self._create_format(QColor(250, 0, 0), 'bold')))  # red
        # create patterns for strings
        self.string_pattern = QRegExp("\"")
        self.string_format = self._create_format(Qt.blue)
        # part to select an XML block
        self._tag_hl_range = []  # list with puples (start, length)
        self._tag_hl_last = []  # set with blocks of last highlighted tags
        self._color_hl_tag = QColor(255, 128, 0)

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
                frmt = form
                if self._in_hl_range(index, self._tag_hl_range):
                    frmt = QTextCharFormat(form)
                    if not self._end_tag_found:
                        frmt.setForeground(Qt.red)
                    else:
                        frmt.setForeground(self._color_hl_tag)
                    frmt.setFontWeight(QFont.Bold)
                self.setFormat(index, length, frmt)
                index = pattern.indexIn(text, index + length)
        # search for YAML comments
        index = self.yaml_comment_rule[0].indexIn(text)
        if index >= 0:
            self.setFormat(index, len(text) - index, self.yaml_comment_rule[1])
        self._tag_hl_range = []
        self.setCurrentBlockState(0)
        # detection for XML comments
        self._comments_idx = []
        idx_start_cmt = 0
        comment_length = 0
        if self.previousBlockState() == -1 or not self.previousBlockState() & self.STATE_COMMENT:
            idx_start_cmt = self.comment_start.indexIn(text)
        while idx_start_cmt >= 0:
            idx_end = self.comment_end.indexIn(text, idx_start_cmt)
            comment_length = 0
            if idx_end == -1:
                self.setCurrentBlockState(self.STATE_COMMENT)
                comment_length = len(text) - idx_start_cmt
            else:
                comment_length = idx_end - idx_start_cmt + self.comment_end.matchedLength()
            self._comments_idx.append((idx_start_cmt, comment_length))
            self.setFormat(idx_start_cmt, comment_length, self.comment_format)
            idx_start_cmt = self.comment_start.indexIn(text, idx_start_cmt + comment_length)
        # format string and detection for multiline string
        idx_start = self.string_pattern.indexIn(text)
        if self.previousBlockState() != -1 and self.previousBlockState() & self.STATE_STRING:
            strlen = idx_start + self.string_pattern.matchedLength()
            if idx_start == -1:
                strlen = len(text)
                self.setCurrentBlockState(self.currentBlockState() + self.STATE_STRING)
            self.setFormat(0, strlen, self.string_format)
            idx_start = self.string_pattern.indexIn(text, strlen)
        idx_search = idx_start + 1
        while idx_start >= 0:
            # skip the strings which are in the comments
            if not self._in_hl_range(idx_search, self._comments_idx):
                idx_end = self.string_pattern.indexIn(text, idx_search)
                strlen = 0
                if not self._in_hl_range(idx_end, self._comments_idx):
                    if idx_end == -1:
                        self.setCurrentBlockState(self.currentBlockState() + self.STATE_STRING)
                        strlen = len(text) - idx_start
                    else:
                        strlen = idx_end - idx_start + self.string_pattern.matchedLength()
                    idx_search = idx_start + strlen
                    self.setFormat(idx_start, strlen, self.string_format)
                    idx_start = self.string_pattern.indexIn(text, idx_search)
                    idx_search = idx_start + 1
                else:
                    idx_search = idx_end + 1
            else:
                idx_start = self.string_pattern.indexIn(text, idx_search)
                idx_search = idx_start + 1
        # mark arguments
        index = self.rule_arg[0].indexIn(text)
        while index >= 0:
            if not self._in_hl_range(index, self._comments_idx):
                length = self.rule_arg[0].matchedLength()
                self.setFormat(index, length, self.rule_arg[1])
            index = self.rule_arg[0].indexIn(text, index + length)
        # mark deprecated parameter
        for pattern, form in self.dep_pattern:
            index = pattern.indexIn(text)
            while index >= 0:
                length = pattern.matchedLength()
                frmt = form
                if self._in_hl_range(index, self._tag_hl_range):
                    frmt = QTextCharFormat(form)
                    if not self._end_tag_found:
                        frmt.setForeground(Qt.red)
                    else:
                        frmt.setForeground(self._color_hl_tag)
                    frmt.setFontWeight(QFont.Bold)
                self.setFormat(index, length, frmt)
                index = pattern.indexIn(text, index + length)

    def mark_block(self, block, position):
        text = block.text()
        word, idx_word = self._get_current_word(text, position)
        for hlblock in self._tag_hl_last:
            self.rehighlightBlock(hlblock)
        del self._tag_hl_last[:]
        self._tag_hl_range = [(idx_word, len(word))]
        next_block = block
        open_braces = 0
        closed_braces = 0
        idx_search = idx_word
        rindex = -1
        loop = 0
        tag_len = 0
        if self._isclosetag(word):
            # we are at the close tag: search for the open tag
            opentag = '<%s' % self._get_tag(word)
            tag_len = len(opentag)
            while rindex == -1 and next_block.isValid():
                rindex = text.rfind(opentag, 0, idx_search)
                obr, cbr = self._get_braces_count(text[rindex if rindex != -1 else 0:idx_search])
                open_braces += obr
                closed_braces += cbr
                loop += 1
                if loop > 50000:
                    rindex = -1
                    break
                if rindex == -1:
                    next_block = next_block.previous()
                    text = next_block.text()
                    idx_search = len(text)
                elif open_braces <= closed_braces:
                    idx_search = rindex
                    rindex = -1
        elif self._isopentag(word):
            # we are at the open tag: search for the close tag
            closetag = QRegExp("</%s>|/>" % self._get_tag(word))
            closetag.setMinimal(True)
            while rindex == -1 and next_block.isValid():
                rindex = closetag.indexIn(text, idx_search)
                max_search_idx = rindex + closetag.matchedLength() if rindex != -1 else len(text)
                obr, cbr = self._get_braces_count(text[idx_search:max_search_idx])
                open_braces += obr
                closed_braces += cbr
                loop += 1
                if loop > 50000:
                    rindex = -1
                    break
                if rindex == -1:
                    next_block = next_block.next()
                    text = next_block.text()
                    idx_search = 0
                elif open_braces > closed_braces:
                    idx_search = rindex + closetag.matchedLength()
                    rindex = -1
                tag_len = closetag.matchedLength()
        else:
            self._tag_hl_range = []
        self._end_tag_found = rindex != -1
        if self._tag_hl_range and block != next_block:
            self.rehighlightBlock(block)
            self._tag_hl_last.append(block)
        if rindex != -1:
            self._tag_hl_range.append((rindex, tag_len))
            self.rehighlightBlock(next_block)
            self._tag_hl_last.append(next_block)

    def _get_braces_count(self, text):
        closed_short = text.count('/>')
        closed_long = text.count('</')
        cmnt_long = text.count('<!')
        openbr = text.count('<') - closed_long - cmnt_long
        return openbr, closed_short + closed_long

    def _isopentag(self, word):
        return word.startswith('<') and '/' not in word

    def _isclosetag(self, word):
        return '/>' == word or word.startswith('</')

    def _get_tag(self, word):
        return word.strip('</>')

    def _get_current_word(self, text, position):
        word = ''
        idx_start = position
        for i in reversed(range(0, position)):
            if text[i] in [' ', '\n', '=', '"']:
                break
            else:
                word = "%s%s" % (text[i], word)
                idx_start = i
        for i in range(position, len(text)):
            if text[i] in [' ', '\n', '=', '"']:
                break
            else:
                word += text[i]
        return word, idx_start

    def _in_hl_range(self, value, ranges):
        for (start, length) in ranges:
            if value >= start and value <= start + length:
                return True
        return False

    def get_tag_of_current_block(self, block, position):
        text = block.text()
        next_block = block
        idx_search = position
        rindex = -1
        loop = 0
        # we are at the close tag: search for the open tag
        opentag = '<'
        while rindex == -1 and next_block.isValid():
            rindex = text.rfind(opentag, 0, idx_search)
            loop += 1
            if loop > 100:
                rindex = -1
                break
            if rindex == -1:
                next_block = next_block.previous()
                text = next_block.text()
                idx_search = len(text)
        tag = ''
        if rindex != -1:
            for i in range(rindex + 1, len(text)):
                if text[i] in [' ', '\n', '=', '"', '>']:
                    break
                else:
                    tag += text[i]
        return tag

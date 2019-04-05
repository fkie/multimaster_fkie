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

import ruamel.yaml


class YamlFormatter(ruamel.yaml.YAML):

    def __init__(self, indent=''):
        self.indent_data = indent
        ruamel.yaml.YAML.__init__(self)

    def format_string(self, data):
        code = ruamel.yaml.load(data.encode('utf-8'), Loader=ruamel.yaml.RoundTripLoader)
        buf = ruamel.yaml.compat.StringIO()
        ruamel.yaml.dump(code, buf, Dumper=ruamel.yaml.RoundTripDumper, encoding='utf-8')
        result = buf.getvalue()  # self.dump(code)
        if self.indent_data:
            lines = result.splitlines()
            result = ''
            last_idx = 0
            for line in lines:
                len_line = len(line)
                idx = len_line - len(line.lstrip())
                indent_ = self.indent_data
                try:
                    # handle comments
                    if idx == line.index('#'):
                        indent_ = ' ' * (last_idx + len(indent_))
                        line = line.lstrip()
                except ValueError:
                    pass
                if idx < len_line:
                    result += '\n%s%s' % (indent_, line)
                else:
                    result += line
                last_idx = idx
        return result

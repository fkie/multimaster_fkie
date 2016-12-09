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

from python_qt_binding.QtCore import QObject, Signal
from python_qt_binding.QtCore import QRegExp, QFile
import os
import rospy
import threading

import node_manager_fkie as nm

from .parser_functions import interpret_path


class TextSearchThread(QObject, threading.Thread):
    '''
    A thread to search recursive for a text in files.
    '''
    search_result_signal = Signal(str, bool, str, int, int, str)
    ''' @ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, position in text, line number, line text)
        for each result a signal will be emitted.
    '''
    warning_signal = Signal(str)

    def __init__(self, search_text, path, is_regex=False, path_text={}, recursive=False):
        '''
        :param search_text: text to search for
        :type search_text: str
        :param path: initial file path
        :type path: str
        .param is_regex: is the search_text a regular expressions
        :type is_regex: bool
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._search_text = search_text
        self._path = path
        self._path_text = path_text
        self._recursive = recursive
        self._isrunning = True
        self.setDaemon(True)

    def stop(self):
        self._isrunning = False

    def run(self):
        '''
        '''
        try:
            self.search(self._search_text, self._path, self._recursive)
        except:
            import traceback
            # print traceback.print_exc()
            formatted_lines = traceback.format_exc(1).splitlines()
            rospy.logwarn("Error while search for '%s' in '%s': %s" % (self._search_text, self._path, formatted_lines[-1]))
            self.warning_signal.emit(formatted_lines[-1])
        finally:
            if self._isrunning:
                self.search_result_signal.emit(self._search_text, False, '', -1, -1, '')

    def search(self, search_text, path, recursive=False):
        '''
        Searches for given text in this document and all included files.
        @param search_text: text to find
        @type search_text: C{str}
        @return: the list with all files contain the text
        @rtype: C{[str, ...]}
        '''
        data = self._get_text(path)
        pos = data.find(search_text)
        slen = len(search_text)
        while pos != -1 and self._isrunning:
            if self._isrunning:
                self.search_result_signal.emit(search_text, True, path, pos, data.count('\n', 0, pos) + 1, self._strip_text(data, pos))
            pos += slen
            pos = data.find(search_text, pos)
        if self._isrunning:
            inc_files = self._included_files(path)
            for incf in inc_files:
                if not self._isrunning:
                    break
                if recursive:
                    self.search(search_text, incf)

    def _get_text(self, path):
        if path in self._path_text:
            return self._path_text[path]
        with open(path, 'r') as f:
            data = f.read()
            return data
        return ''

    def _included_files(self, path):
        '''
        Returns all included files in the given file.
        '''
        result = []
        with open(path, 'r') as f:
            data = f.read()
            reg = QRegExp("=[\s\t]*\".*\"")
            reg.setMinimal(True)
            pos = reg.indexIn(data)
            while pos != -1 and self._isrunning:
                try:
                    pp = interpret_path(reg.cap(0).strip('"'))
                    f = QFile(pp)
                    ext = os.path.splitext(pp)
                    if f.exists() and ext[1] in nm.settings().SEARCH_IN_EXT:
                        result.append(pp)
                except Exception as exp:
                    parsed_text = pp
                    try:
                        parsed_text = reg.cap(0).strip('"')
                    except:
                        pass
                    self.warning_signal.emit("Error while parse '%s': %s" % (parsed_text, exp))
                pos += reg.matchedLength()
                pos = reg.indexIn(data, pos)
        return result

    def _strip_text(self, data, pos):
        start = pos
        end = pos
        while start > 0 and data[start - 1] not in ['\n', '\0', '\r']:
            start -= 1
        while end < len(data) and data[end] not in ['\n', '\0', '\r']:
            end += 1
        return data[start:end].strip()

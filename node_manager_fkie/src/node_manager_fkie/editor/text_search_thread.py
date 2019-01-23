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
import rospy
import threading

import node_manager_fkie as nm

from node_manager_daemon_fkie.common import replace_internal_args
from node_manager_fkie.common import utf8


class TextSearchThread(QObject, threading.Thread):
    '''
    A thread to search recursive for a text in files.
    '''
    search_result_signal = Signal(str, bool, str, int, int, str)
    ''' :ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, position in text, line number, line text)
        for each result a signal will be emitted.
    '''
    warning_signal = Signal(str)

    def __init__(self, search_text, path, is_regex=False, path_text={}, recursive=False, only_launch=False):
        '''
        :param str search_text: text to search for
        :param str path: initial file path
        :param bool is_regex: is the search_text a regular expressions
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._search_text = search_text
        self._path = path
        self._path_text = path_text
        self._recursive = recursive
        self._only_launch = only_launch
        self._isrunning = True
        self.setDaemon(True)

    def stop(self):
        self._isrunning = False

    def run(self):
        '''
        '''
        try:
            self.search(self._search_text, self._path, self._recursive)
        except Exception:
            import traceback
            # formatted_lines = traceback.format_exc(1).splitlines()
            rospy.logwarn("Error while search for '%s' in '%s': %s" % (self._search_text, self._path, traceback.print_exc()))
            self.warning_signal.emit(traceback.print_exc())
        finally:
            if self._isrunning:
                self.search_result_signal.emit(self._search_text, False, '', -1, -1, '')

    def search(self, search_text, path, recursive=False, args={}, content='', count=1):
        '''
        Searches for given text in this document and all included files.
        :param str search_text: text to find
        :return: the list with all files contain the text
        :rtype: [str, ...]
        '''
        data = content
        if not data:
            data = self._get_text(path)
        pos = data.find(search_text)
        slen = len(search_text)
        found = False
        while pos != -1 and self._isrunning:
            found = True
            if self._isrunning:
                doemit = True
                if self._only_launch:
                    doemit = path.endswith('.launch')
                if doemit:
                    self.search_result_signal.emit(search_text, True, path, pos, data.count('\n', 0, pos) + 1, self._strip_text(data, pos))
            pos += slen
            pos = data.find(search_text, pos)
        if self._isrunning:
            # try to replace the arguments and search again
            resolve_args = args
            if not resolve_args:
                resolve_args = nm.nmd().launch_args(path)
            if not found and not content:
                new_data = data
                if search_text.startswith('name="'):
                    replaced, new_data, _internal_args = replace_internal_args(data, resolve_args)
                if replaced:
                    self.search(search_text, path, False, resolve_args, new_data, count + 1)
            if recursive:
                # TODO: test search string for 'name=' and skip search in not launch files
                queue = []
                inc_files = nm.nmd().get_included_files(path, False)
                # read first all included files in curret file
                for _linenr, inc_path, exists, include_args in inc_files:
                    if not self._isrunning:
                        break
                    if exists:
                        queue.append((search_text, inc_path, recursive, include_args))
                # search in all files
                for search_text, inc_path, recursive, include_args in queue:
                    new_dict = dict(resolve_args)
                    new_dict.update(include_args)
                    self.search(search_text, inc_path, recursive, new_dict, '', count + 1)

    def _get_text(self, path):
        result = ''
        try:
            result = self._path_text[path]
        except KeyError:
            try:
                _, _, data = nm.nmd().get_file_content(path)
                result = utf8(data)
            except Exception as err:
                rospy.logwarn("can't get content: %s" % (utf8(err)))
        return result

    def _strip_text(self, data, pos):
        start = pos
        end = pos
        while start > 0 and data[start - 1] not in ['\n', '\0', '\r']:
            start -= 1
        while end < len(data) and data[end] not in ['\n', '\0', '\r']:
            end += 1
        return data[start:end].strip()

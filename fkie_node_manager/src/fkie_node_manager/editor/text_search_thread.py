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
import re
import rospy
import os
import threading
from xml.dom import minidom

import fkie_node_manager as nm

from fkie_node_manager_daemon import exceptions
from fkie_node_manager_daemon.common import replace_arg, utf8


class TextSearchThread(QObject, threading.Thread):
    '''
    A thread to search recursive for a text in files.
    '''
    search_result_signal = Signal(str, bool, str, int, int, int, str)
    ''' :ivar: A signal emitted after search_threaded was started.
        (search text, found or not, file, first position in text, last position in text, line number, line text)
        for each result a signal will be emitted.
    '''
    warning_signal = Signal(str)

    def __init__(self, search_text, path, is_regex=False, path_text={}, recursive=False, only_launch=False, count_results=0):
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
        self._found = 0
        self._count_results = count_results
        self._isrunning = True
        self.setDaemon(True)

    def stop(self):
        self._isrunning = False

    def run(self):
        '''
        '''
        try:
            if self._search_text.startswith('name="'):
                self.search_for_node(self._search_text, self._path, self._recursive)
            else:
                self.search(self._search_text, self._path, self._recursive)
        except exceptions.GrpcTimeout as tout:
            self.warning_signal.emit("Search in launch failed! Daemon not responded within %.2f seconds while"
                                     " get configuration file: %s\nYou can try to increase"
                                     " the timeout for GRPC requests in node manager settings." % (nm.settings().timeout_grpc, tout.remote))
        except Exception:
            import traceback
            # formatted_lines = traceback.format_exc(1).splitlines()
            msg = "Error while search for '%s' in '%s': %s" % (self._search_text, self._path, traceback.print_exc())
            rospy.logwarn(msg)
            self.warning_signal.emit(msg)
        finally:
            if self._isrunning:
                self.search_result_signal.emit(self._search_text, False, '', -1, -1, -1, '')

    def search(self, search_text, path, recursive=False, args={}, count=1):
        '''
        Searches for given text in this document and all included files.
        :param str search_text: text to find
        :return: the list with all files contain the text
        :rtype: [str, ...]
        '''
        if not self._isrunning:
            return
        data = self._get_text(path)
        pos = data.find(search_text)
        slen = len(search_text)
        while pos != -1 and self._isrunning:
            if self._isrunning:
                doemit = True
                line = self._strip_text(data, pos)
                if self._only_launch:
                    doemit = path.endswith('.launch') or path.find('.launch.') > 0
                if doemit:
                    self._found += 1
                    self.search_result_signal.emit(search_text, True, path, pos, pos + len(search_text), data.count('\n', 0, pos) + 1, line)
            if self._count_results > 0 and self._count_results < self._found:
                # break search if found the requested count of occurrences
                return
            pos += slen
            pos = data.find(search_text, pos)
        if self._isrunning:
            if recursive:
                queue = []
                inc_files = nm.nmd().launch.get_included_files(path, False, include_args=args)
                # read first all included files in current file
                for inc_file in inc_files:
                    if not self._isrunning:
                        return
                    if inc_file.exists:
                        queue.append((search_text, inc_file.inc_path, recursive, inc_file.args))
                # search in all files
                for search_text, inc_path, recursive, include_args in queue:
                    new_dict = dict(args)
                    new_dict.update(include_args)
                    # test search string for 'name=' and skip search in not launch files
                    if not self._only_launch or inc_path.endswith('.launch') or path.find('.launch.') > 0:
                        self.search(search_text, inc_path, recursive, new_dict, count + 1)
        if self._path == path and self._found == 0:
            self.warning_signal.emit("not found '%s' in %s (%srecursive)" % (search_text, path, '' if recursive else 'not '))

    def search_for_node(self, search_text, path, recursive=False, args={}, count=1):
        '''
        Searches for node in this document and all included files.
        :param str search_text: node to find with 'name="' prefix
        :return: the list with all files contain the text
        :rtype: [str, ...]
        '''
        if path and not (path.endswith('.launch') or path.find('.launch.') > 0):
            return
        if not self._isrunning:
            return
        data = self._get_text(path)
        launch_node = self._get_launch_element(data, path)
        if launch_node is None:
            # something goes wrong while parse XML content
            # backup solution to search without resolve arguments
            self.search(search_text, path, recursive, args, count)
        else:
            # read XML content and update the arguments
            rospy.logdebug("search for node '%s' in %s with args: %s, recursive: %s" % (search_text, path, args, recursive))
            resolve_args = dict(args)
            if not resolve_args:
                resolve_args.update(nm.nmd().launch.launch_args(path))
            my_resolved_args = self._resolve_args(launch_node, resolve_args, path)
            # replace arguments and search for node in data
            search_for_name = search_text.replace('name="', '').replace('"', '')
            occur_idx = 0
            for aname, _rname, span in self._next_node_name(data, search_for_name, my_resolved_args, path):
                # found, now test in XML for if and unless statements
                if self._check_node_conditions(launch_node, search_for_name, occur_idx, my_resolved_args, path):
                    self._found += 1
                    self.search_result_signal.emit(search_text, True, path, span[0], span[1], data.count('\n', 0, span[0]) + 1, aname)
                else:
                    self.warning_signal.emit("%s in %s ignored because of conditions." % (search_text, path))
                occur_idx += 1
            if self._isrunning and recursive:
                queue = []
                inc_files = nm.nmd().launch.get_included_files(path, False, include_args=args, search_in_ext=['.launch', '.xml'])
                # read first all included files in current file
                for inc_file in inc_files:
                    if not self._isrunning:
                        return
                    if inc_file.exists:
                        queue.append((search_text, inc_file.inc_path, recursive, inc_file.args))
                    elif inc_file.inc_path.endswith('.launch') or inc_file.inc_path.find('.launch.') > 0:
                        rospy.logwarn("skip parsing of not existing included file: %s" % inc_file.inc_path)
                # search in all files
                for search_text, inc_path, recursive, include_args in queue:
                    new_dict = dict(my_resolved_args)
                    new_dict.update(include_args)
                    # skip search in not launch files
                    if inc_path.endswith('.launch') or inc_path.find('.launch.') > 0:
                        self.search_for_node(search_text, inc_path, recursive, new_dict, count + 1)
        if self._path == path and self._found == 0:
            self.warning_signal.emit("not found '%s' in %s (%srecursive)" % (search_text, path, '' if recursive else 'not '))

    def _get_launch_element(self, content, path):
        result = None
        try:
            xml_nodes = minidom.parseString(content.encode('utf-8')).getElementsByTagName('launch')
            if xml_nodes:
                result = xml_nodes[-1]
        except Exception as err:
            msg = "%s in %s" % (utf8(err), path)
            self.warning_signal.emit(msg)
            rospy.logwarn(msg)
        return result

    def _resolve_args(self, launch_node, resolve_args, path):
        '''
        Load the content with xml parser, search for arg-nodes.
        :return: Dictionary with argument names and values
        :rtype: {name: value}
        '''
        resolve_args_intern = dict(resolve_args)
        try:
            for child in launch_node.childNodes:
                if child.localName == 'arg' and child.hasAttributes():
                    aname = ''
                    aval = ''
                    for argi in range(child.attributes.length):
                        arg_attr = child.attributes.item(argi)
                        if arg_attr.localName == 'name':
                            aname = arg_attr.value
                        elif arg_attr.localName in ['value', 'default']:
                            aval = arg_attr.value
                    if aname and aname not in resolve_args_intern:
                        for arg_key, args_val in resolve_args_intern.items():
                            aval = aval.replace('$(arg %s)' % arg_key, args_val)
                        resolve_args_intern[aname] = aval
        except Exception as err:
            rospy.logwarn("%s in %s" % (utf8(err), path))
        return resolve_args_intern

    def _next_node_name(self, content, node_name, resolve_args={}, path=''):
        '''
        Load the content with xml parser, search for arg-nodes and replace the arguments in node-statements.
        note:: We do not use xml parser to be able to find the position of the name in file.

        :return: True if something was replaced, node name, line number
        :rtype: (bool, str, line number)
        '''
        re_nodes = re.compile(r"<node[\w\W\S\s]*?name=\"(?P<name>.*?)\"")
        for groups in re_nodes.finditer(content):
            aname = groups.group("name")
            rospy.logdebug("  check node name '%s' for '%s' in group %s" % (aname, node_name, groups.span("name")))
            if aname == node_name:
                yield node_name, aname, groups.span("name")
            else:
                rname = aname
                for arg_key, args_val in resolve_args.items():
                    rname = rname.replace('$(arg %s)' % arg_key, args_val)
                if rname == node_name:
                    yield aname, rname, groups.span("name")

    def _check_node_conditions(self, launch_node, node_name, node_idx, resolve_args, path):
        try:
            idx = 0
            nodes = launch_node.getElementsByTagName('node')
            for node in nodes:
                if node.getAttribute('name') == node_name:
                    if idx == node_idx:
                        for attridx in range(node.attributes.length):
                            attr = node.attributes.item(attridx)
                            if attr.localName == 'if':
                                val = replace_arg(attr.value, resolve_args)
                                return val in ['true', '1']
                            elif attr.localName == 'unless':
                                val = replace_arg(attr.value, resolve_args)
                                return val in ['false', '0']
                    idx += 1
        except Exception as err:
            rospy.logwarn("%s in %s" % (utf8(err), path))
        return True

    def _get_text(self, path):
        result = ''
        try:
            result = self._path_text[path]
        except KeyError:
            try:
                _, _, data = nm.nmd().file.get_file_content(path)
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

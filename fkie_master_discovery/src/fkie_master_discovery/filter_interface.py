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

import re
import sys

import rospy

from .common import create_pattern, gen_pattern, is_empty_pattern, EMPTY_PATTERN
from .common import get_ros_param, resolve_url, read_interface


class FilterInterface(object):
    '''
    The class represents a filter used by sync node to filter nodes, topics or
    services. The filter is based on regular expressions. The filter object can
    be converted to a tuple of strings and then passed to the XML-RPC method to
    get a filtered ROS master state.
    After creation of the filter object you must call
    :mod:`fkie_master_discovery.filter_interface.FilterInterface.load()` or
    :mod:`fkie_master_discovery.filter_interface.FilterInterface.from_list()`.
    Otherwise the object is invalid and the test methods return always `False`.
    '''

    def __init__(self):
        self.is_valid = False
        self._re_do_not_sync = EMPTY_PATTERN
        self._re_do_not_sync_from_list = EMPTY_PATTERN
        self._re_hide_nodes = EMPTY_PATTERN
        self._re_hide_topics = EMPTY_PATTERN
        self._re_hide_services = EMPTY_PATTERN

    def load(self, mastername='',
             ignore_nodes=[], sync_nodes=[],
             ignore_topics=[], sync_topics=[],
             ignore_srv=[], sync_srv=[],
             ignore_type=[],
             ignore_publishers=[], ignore_subscribers=[],
             do_not_sync=[]):
        '''
        Reads the parameter and creates the pattern using :mod:`fkie_master_discovery.common.create_pattern()`
        '''
        self.__interface_file = interface_file = resolve_url(get_ros_param('~interface_url', ''))
        self.__mastername = mastername
        self.__data = data = read_interface(interface_file) if interface_file else {}
        # set the pattern for ignore and sync lists
        self._re_ignore_nodes = create_pattern('ignore_nodes', data, interface_file,
                                               ignore_nodes, mastername)
        self._re_sync_nodes = create_pattern('sync_nodes', data, interface_file,
                                             sync_nodes, mastername)
        self._re_ignore_topics = create_pattern('ignore_topics', data, interface_file,
                                                ignore_topics, mastername)
        self._re_sync_topics = create_pattern('sync_topics', data, interface_file,
                                              sync_topics, mastername)
        self._re_ignore_services = create_pattern('ignore_services', data, interface_file,
                                                  ignore_srv, mastername)
        self._re_sync_services = create_pattern('sync_services', data, interface_file,
                                                sync_srv, mastername)
        self._re_ignore_type = create_pattern('ignore_type', data, interface_file,
                                              ignore_type, mastername)
        self._re_ignore_publishers = create_pattern('ignore_publishers', data, interface_file,
                                                    ignore_publishers, mastername)
        self._re_ignore_subscribers = create_pattern('ignore_subscribers', data, interface_file,
                                                     ignore_subscribers, mastername)
        self._sync_remote_nodes = False
        if interface_file:
            if 'sync_remote_nodes' in data:
                self._sync_remote_nodes = data['sync_remote_nodes']
        else:
            self._sync_remote_nodes = get_ros_param('~sync_remote_nodes', self._sync_remote_nodes)
        if do_not_sync:
            self._re_do_not_sync = gen_pattern(do_not_sync, 'do_not_sync')
        else:
            self.read_do_not_sync()
        self.is_valid = True

    def set_hide_pattern(self, re_hide_nodes=EMPTY_PATTERN, re_hide_topics=EMPTY_PATTERN, re_hide_services=EMPTY_PATTERN):
        self._re_hide_nodes = re_hide_nodes
        self._re_hide_topics = re_hide_topics
        self._re_hide_services = re_hide_services

    def read_do_not_sync(self):
        _do_not_sync = get_ros_param('do_not_sync', [])
        if sys.version_info[0] <= 2:
            import types
            string_types = types.StringTypes
        else:
            string_types = (str,)
        if isinstance(_do_not_sync, string_types):
            # create a list from string
            _do_not_sync = _do_not_sync.strip('[').rstrip(']').replace(' ', ',').split(',')
            # remove empty values
            _do_not_sync = [val for val in _do_not_sync if val]
        self._re_do_not_sync = gen_pattern(_do_not_sync, 'do_not_sync', print_info=False)

    def update_sync_topics_pattern(self, topics=[]):
        '''
        Updates the sync topics pattern.

        :param topics: the list of topics to sync.

        :type topics: list of strings
        '''
        self._re_sync_topics = create_pattern('sync_topics', self.__data, self.__interface_file, topics, self.__mastername)

    def sync_remote_nodes(self):
        '''
        Returns the value stored in `sync_remote_nodes` parameter.
        '''
        if not self.is_valid:
            return False
        return self._sync_remote_nodes

    def is_ignored_node(self, node):
        '''
        Searches the given node in `ignore_nodes` and `sync_nodes` lists.

        :param node: the name of the node to test.

        :type node: str

        :return: `True`, if the node was found in the `ignore_nodes` list or the
          `sync_nodes` is not empty.

        :note: If the filter object is not initialized by load() or from_list() the
              returned value is `False`
        '''
        if not self.is_valid:
            return False
        if self._re_hide_nodes.match(node):
            return True
        if self.do_not_sync(node):
            return True
        if self._re_sync_nodes.match(node):
            return False
        # there are no sync nodes defined => return False
        return not is_empty_pattern(self._re_sync_nodes)

    def is_ignored_topic(self, node, topic, topictype):
        '''
        NOTE: This function is deprecated. Please use `is_ignored_subscriber` and
        `is_ignored_publisher` instead

        Searches firstly in ignore lists `ignore_type`, `ignore_nodes` and `ignore_topics`.
        Then in `sync_nodes` or `sync_topics`.

        :param node: the name of the node published or subscribed the topic.

        :type node: str

        :param topic: the name of the topic to test.

        :type topic: str

        :param topictype: the type of the topic. (e.g. the synchronization of bond/Status terminate the nodelets)

        :type topictype: str

        :return: `True`, if the values are found in `ignore_type`, `ignore_nodes`
          or `ignore_topics`. If `sync_nodes` or `sync_topics` is empty `True` will
          be returned, too.
        :note: If the filter object is not initialized by load() or from_list() the
              returned value is `False`
        '''
        rospy.logwarn("Call to deprecated method 'is_ignored_topic'. Please use"
                      "'is_ignored_subscriber' and 'is_ignored_publisher' instead")
        self._is_ignored_topic(node, topic, topictype)

    def _is_ignored_topic(self, node, topic, topictype):
        if not self.is_valid:
            return False
        if self.do_not_sync([node, topic, topictype]):
            return True
        # do not sync the bond message of the nodelets!!
        if self._re_ignore_type.match(topictype):
            return True
        if self._re_ignore_nodes.match(node):
            return True
        if self._re_ignore_topics.match(topic):
            return True
        if self._re_sync_nodes.match(node) or self._re_sync_topics.match(topic):
            return False
        # there are no sync nodes and topic lists defined => return False (=>sync the given topic)
        return not is_empty_pattern(self._re_sync_nodes) or not is_empty_pattern(self._re_sync_topics)

    def is_ignored_subscriber(self, node, topic, topictype):
        '''
        Searches first in the `ignore_subscribers` ignore list
        Then in ignore lists `ignore_type`, `ignore_nodes` and `ignore_topics`.
        Finally in `sync_nodes` or `sync_topics`.

        :param node: the name of the node published or subscribed the topic.

        :type node: str

        :param topic: the name of the topic to test.

        :type topic: str

        :param topictype: the type of the topic. (e.g. the synchronization of bond/Status terminate the nodelets)

        :type topictype: str

        :return: `True`, if the values are found in `ignore_type`, `ignore_nodes`
          or `ignore_topics`. If `sync_nodes` or `sync_topics` is empty `True` will
          be returned, too.
        :note: If the filter object is not initialized by load() or from_list() the
              returned value is `False`
        '''
        if self._re_hide_nodes.match(node):
            return True
        if self._re_hide_topics.match(topic):
            return True
        if self.do_not_sync([node, topic, topictype]):
            return True
        return self._re_ignore_subscribers.match(topic) or self._is_ignored_topic(node, topic, topictype)

    def is_ignored_publisher(self, node, topic, topictype):
        '''
        Searches first in the `ignore_publishers` ignore list
        Then in ignore lists `ignore_type`, `ignore_nodes` and `ignore_topics`.
        Finally in `sync_nodes` or `sync_topics`.

        :param node: the name of the node published or subscribed the topic.

        :type node: str

        :param topic: the name of the topic to test.

        :type topic: str

        :param topictype: the type of the topic. (e.g. the synchronization of bond/Status terminate the nodelets)

        :type topictype: str

        :return: `True`, if the values are found in `ignore_type`, `ignore_nodes`
          or `ignore_topics`. If `sync_nodes` or `sync_topics` is empty `True` will
          be returned, too.
        :note: If the filter object is not initialized by load() or from_list() the
              returned value is `False`
        '''
        if self._re_hide_nodes.match(node):
            return True
        if self._re_hide_topics.match(topic):
            return True
        if self.do_not_sync([node, topic, topictype]):
            return True
        return self._re_ignore_publishers.match(topic) or self._is_ignored_topic(node, topic, topictype)

    def is_ignored_service(self, node, service):
        '''
        Searches firstly in ignore lists `ignore_nodes` and `ignore_services`.
        Then in `sync_nodes` or `sync_services`.

        :param node: the name of the node provided the service.

        :type node: str

        :param topic: the name of the service to test.

        :type topic: str

        :return: `True`, if the values are found in `ignore_nodes` or `ignore_services`.
                If `sync_nodes` or `sync_services` is empty `True` will be returned, too.

        :note: If the filter object is not initialized by load() or from_list() the
              returned value is `False`
        '''
        if not self.is_valid:
            return False
        if self._re_hide_nodes.match(node):
            return True
        if self._re_hide_services.match(service):
            return True
        if self.do_not_sync([node, service]):
            return True
        if self._re_ignore_nodes.match(node):
            return True
        if self._re_ignore_services.match(service.strip()):
            return True
        if self._re_sync_nodes.match(node) or self._re_sync_services.match(service):
            return False
        return not is_empty_pattern(self._re_sync_nodes) or not is_empty_pattern(self._re_sync_services)

    def do_not_sync(self, name):
        if isinstance(name, list):
            for nval in name:
                if self._re_do_not_sync.match(nval) or self._re_do_not_sync_from_list.match(nval):
                    return True
        elif self._re_do_not_sync.match(name) or self._re_do_not_sync_from_list.match(name):
            return True
        return False

    def to_list(self):
        '''
        :returns: the tuple of the all filter patterns.
          ::

            (sync_remote_nodes,
            ignore_nodes, sync_nodes,
            ignore_topics, sync_topics,
            ignore_services, sync_services,
            ignore_type,
            ignore_publishers, ignore_subscribers,
            do_not_sync)

        :rtype: `(bool, str, str, str, str, str, str, str, str, str, str)`
        '''
        if not self.is_valid:
            return (False, '', '', '', '', '', '', '', '', '', '')
        return (self._sync_remote_nodes,
                _to_str(self._re_ignore_nodes),
                _to_str(self._re_sync_nodes),
                _to_str(self._re_ignore_topics),
                _to_str(self._re_sync_topics),
                _to_str(self._re_ignore_services),
                _to_str(self._re_sync_services),
                _to_str(self._re_ignore_type),
                _to_str(self._re_ignore_publishers),
                _to_str(self._re_ignore_subscribers),
                _to_str(self._re_do_not_sync))

    @staticmethod
    def from_list(l=None):
        '''
        Reads the tuple of
          ::

            (sync_remote_nodes,
            ignore_nodes, sync_nodes,
            ignore_topics, sync_topics,
            ignore_services, sync_services,
            ignore_type,
            ignore_publishers, ignore_subscribers,
            do_not_sync)`

        with types
        `(bool, str, str, str, str, str, str, str, str)`

        and creates the `FilterInterface` object.

        :return: `FilterInterface` object or `None` on failure
        '''
        try:
            result = FilterInterface()
            if l is None:
                l = (False, '', '', '', '', '', '', '', '', '', '')
            else:
                result.read_do_not_sync()
            result._sync_remote_nodes = bool(l[0])
            result._re_ignore_nodes = _from_str(l[1])
            result._re_sync_nodes = _from_str(l[2])
            result._re_ignore_topics = _from_str(l[3])
            result._re_sync_topics = _from_str(l[4])
            result._re_ignore_services = _from_str(l[5])
            result._re_sync_services = _from_str(l[6])
            result._re_ignore_type = _from_str(l[7])
            result._re_ignore_publishers = _from_str(l[8] if len(l) > 8 else '')
            result._re_ignore_subscribers = _from_str(l[9] if len(l) > 9 else '')
            result._re_do_not_sync_from_list = _from_str(l[10] if len(l) > 10 else '')
            result.is_valid = True
            return result
        except:
            import traceback
            print(traceback.format_exc())
        return None


def _to_str(re_object):
    if is_empty_pattern(re_object):
        return ''
    return re_object.pattern


def _from_str(msg):
    if msg:
        return re.compile(msg, re.I)
    else:
        return EMPTY_PATTERN

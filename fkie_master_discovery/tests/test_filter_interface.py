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

import os
import unittest

from fkie_master_discovery.filter_interface import FilterInterface

PKG = 'fkie_master_discovery'


class TestFilterInterface(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def test_do_not_sync(self):
        fi = FilterInterface()
        
        fi.load(mastername='testmaster',
                ignore_nodes=[], sync_nodes=['/node_one', '/node_two/topic'],
                ignore_topics=[], sync_topics=['/test_topic'],
                ignore_srv=[], sync_srv=[],
                ignore_type=[],
                ignore_publishers=[], ignore_subscribers=[],
                do_not_sync=[])
        ignore_by_do_no_sync = fi.do_not_sync(['/some_node', '/test_topic', 'SomeType'])
        self.assertFalse(ignore_by_do_no_sync, "/test_topic is in sync_topic, but ignored by do not sync")
        ignore = fi.is_ignored_publisher('/some_node', '/test_topic', '')
        self.assertFalse(ignore, "/test_topic is in sync_topic, but ignored by filter interface")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFilterInterface)


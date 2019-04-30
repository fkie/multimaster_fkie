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
import time

import fkie_node_manager_daemon.screen as screen

PKG = 'fkie_node_manager_daemon'


class TestScreen(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_create_session_name(self):
        name = screen.create_session_name(None)
        self.assertEqual(name, '', "wrong screen session name from `None`, got: %s, expected: %s" % (name, ''))
        name = screen.create_session_name('/test/node')
        self.assertEqual(name, '_test_node', "wrong screen session name from `/test/node`, got: %s, expected: %s" % (name, '_test_node'))

    def test_session_name2node_name(self):
        sname = screen.create_session_name('/test/node')
        nname = screen.session_name2node_name(sname)
        self.assertEqual(nname, '/test/node', "wrong node name from session name, got: %s, expected: %s" % (nname, '/test/node'))

    def test_split_session_name(self):
        _pid, name = screen.split_session_name(None)
        self.assertEqual(name, '', "wrong screen session name after split from `None`, got: %s, expected: %s" % (name, ''))
        _pid, name = screen.split_session_name('123._test_node')
        self.assertEqual(name, '_test_node', "wrong screen session name after split from `123._test_node`, got: %s, expected: %s" % (name, '_test_node'))
        pid, _name = screen.split_session_name('was._test_node')
        self.assertEqual(pid, -1, "wrong pid after screen split session `was._test_node`, got: %d, expected: %d" % (pid, -1))
        _pid, name = screen.split_session_name('666. ')
        self.assertEqual(name, '', "wrong name after screen split session `666.`, got: %s, expected: %s" % (name, ''))

    def test_rosclean(self):
        screen.rosclean()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestScreen)


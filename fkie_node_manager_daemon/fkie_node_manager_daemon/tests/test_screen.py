# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        self.assertEqual(
            name, '', "wrong screen session name from `None`, got: %s, expected: %s" % (name, ''))
        name = screen.create_session_name('/test/node')
        self.assertEqual(
            name, '_test_node', "wrong screen session name from `/test/node`, got: %s, expected: %s" % (name, '_test_node'))

    def test_session_name2node_name(self):
        sname = screen.create_session_name('/test/node')
        nname = screen.session_name2node_name(sname)
        self.assertEqual(
            nname, '/test/node', "wrong node name from session name, got: %s, expected: %s" % (nname, '/test/node'))

    def test_split_session_name(self):
        _pid, name = screen.split_session_name(None)
        self.assertEqual(
            name, '', "wrong screen session name after split from `None`, got: %s, expected: %s" % (name, ''))
        _pid, name = screen.split_session_name('123._test_node')
        self.assertEqual(name, '_test_node', "wrong screen session name after split from `123._test_node`, got: %s, expected: %s" % (
            name, '_test_node'))
        pid, _name = screen.split_session_name('was._test_node')
        self.assertEqual(
            pid, -1, "wrong pid after screen split session `was._test_node`, got: %d, expected: %d" % (pid, -1))
        _pid, name = screen.split_session_name('666. ')
        self.assertEqual(
            name, '', "wrong name after screen split session `666.`, got: %s, expected: %s" % (name, ''))

    def test_rosclean(self):
        screen.rosclean()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestScreen)

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

import fkie_node_manager_daemon.host as host

PKG = 'fkie_node_manager_daemon'


class TestHost(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_get_hostname(self):
        hostname = host.get_hostname(None)
        self.assertEqual(hostname, None, "Hostname from `None` should be `None`, got: %s, expected: %s" % (hostname, None))
        hostname = host.get_hostname('')
        self.assertEqual(hostname, '', "Hostname from `` should be ``, got: %s, expected: %s" % (hostname, ''))
        name = 'myhost'
        hostname = host.get_hostname('http://%s' % name)
        self.assertEqual(hostname, name, "Wrong hostname from `http://%s`, got: %s, expected: %s" % (name, hostname, name))
        hostname = host.get_hostname('http://%s:11111' % name)
        self.assertEqual(hostname, name, "Wrong hostname from `http://%s:11111`, got: %s, expected: %s" % (name, hostname, name))
        hostname = host.get_hostname('%s:11111' % name)
        self.assertEqual(hostname, name, "Wrong hostname from `%s:11111`, got: %s, expected: %s" % (name, hostname, name))
        wrong = '%s:11:2' % name
        hostname = host.get_hostname(wrong)
        self.assertEqual(hostname, wrong, "Wrong hostname from `%s`, got: %s, expected: %s" % (wrong, hostname, wrong))

    def test_get_port(self):
        port = host.get_port(None)
        self.assertEqual(port, None, "Port from `None` should be `None`, got: %s, expected: %s" % (port, None))
        port = host.get_port('')
        self.assertEqual(port, '', "Port from `` should be ``, got: %s, expected: %s" % (port, ''))
        port = host.get_port('host:21')
        self.assertEqual(port, 21, "wrong port from `:21`, got: %s, expected: %d" % (port, 21))
        port = host.get_port('https://host:21')
        self.assertEqual(port, 21, "wrong port from `https://host:21`, got: %s, expected: %d" % (port, 21))
        port = host.get_port('https://host:s21')
        self.assertEqual(port, None, "wrong port from `https://host:s21`, got: %s, expected: %s" % (port, None))

    def test_get_ros_hostname(self):
        roshn = host.get_ros_hostname(None)
        self.assertEqual(roshn, '', "ros hostname from `None` should be ``, got: %s, expected: %s" % (roshn, ''))
        roshn = host.get_ros_hostname('http://myhost:11311')
        self.assertEqual(roshn, 'myhost', "wrong ros hostname from `'http://myhost:11311'`, got: %s, expected: %s" % (roshn, 'myhost'))
        roshn = host.get_ros_hostname('http://192.168.11.5:11311')
        self.assertEqual(roshn, '', "wrong ros hostname from `'http://192.168.11.5:11311'`, got: %s, expected: %s" % (roshn, ''))

    def test_is_local(self):
        local = host.is_local(None)
        self.assertEqual(local, True, "wrong is_local from `None`, got: %s, expected: %s" % (local, True))
        local = host.is_local('localhost')
        self.assertEqual(local, True, "wrong is_local from `localhost`, got: %s, expected: %s" % (local, True))
        local = host.is_local('localhost')
        self.assertEqual(local, True, "wrong cached is_local from `localhost`, got: %s, expected: %s" % (local, True))
        local = host.is_local('127.0.0.1')
        self.assertEqual(local, True, "wrong cached is_local from `127.0.0.1`, got: %s, expected: %s" % (local, True))
        local = host.is_local('heise.de', False)
        self.assertEqual(local, False, "wrong is_local from `heise.de`, got: %s, expected: %s" % (local, False))
        local = host.is_local('heise.de', False)
        self.assertEqual(local, False, "wrong cached is_local from `heise.de`, got: %s, expected: %s" % (local, False))
        local = host.is_local('unknown', False)
        self.assertEqual(local, False, "wrong is_local from `unknown`, got: %s, expected: %s" % (local, False))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestHost)


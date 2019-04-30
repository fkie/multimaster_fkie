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


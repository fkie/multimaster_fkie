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
import rospkg

from fkie_node_manager_daemon import host
from fkie_node_manager_daemon import url

PKG = 'fkie_node_manager_daemon'


class TestUrlLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.test_include_file = "%s/resources/include_dummy.launch" % os.getcwd()

    def tearDown(self):
        pass

    def test_nmduri(self):
        grpc_exp = 'grpc://localhost:12321'
        grpc_url = url.nmduri(grpc_exp)
        self.assertEqual(grpc_exp, grpc_url, "wrong grpc url from default grpc uri, expected: %s, got: %s" % (
            grpc_exp, grpc_url))
        grpc_exp = 'grpc://%s:12321' % host.get_host_name()
        grpc_url = url.nmduri('')
        self.assertEqual(grpc_exp, grpc_url, "wrong grpc url from empty string, expected: %s, got: %s" % (
            grpc_exp, grpc_url))

    def test_nmdport(self):
        port_exp = url.NMD_DEFAULT_PORT
        port_res = url.nmdport()
        self.assertEqual(port_exp, port_res, "wrong port from empty grpc url, expected: %s, got: %s" % (
            port_exp, port_res))

    def test_nmduri_from_path(self):
        grpc_path = 'grpc://localhost:12321/test/path.launch'
        grpc_exp = 'grpc://localhost:12321'
        grpc_res = url.nmduri_from_path(grpc_path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from default grpc path, expected: %s, got: %s" % (
            grpc_exp, grpc_res))
        grpc_path = 'localhost:12321/test/path.launch'
        grpc_exp = ''
        try:
            grpc_res = url.nmduri_from_path(grpc_path)
            self.assertEqual(grpc_exp, grpc_res, "missed exeption on grpc url without scheme, expected: %s, got: %s" % (
                'ValueError', grpc_res))
        except ValueError:
            pass

    def test_join(self):
        grpc_url = 'grpc://localhost:12321'
        path = 'grpc://localhost:12321/test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from path with grpc scheme, expected: %s, got: %s" % (
            grpc_exp, grpc_res))
        grpc_url = 'grpc://localhost:12321'
        path = '/test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from grpc scheme and path with leading slash, expected: %s, got: %s" % (
            grpc_exp, grpc_res))
        grpc_url = 'grpc://localhost:12321'
        path = 'test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from grpc scheme and path without leading slash, expected: %s, got: %s" % (
            grpc_exp, grpc_res))
        grpc_url = 'localhost:12321'
        path = 'test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        try:
            grpc_res = url.join(grpc_url, path)
            self.assertEqual(grpc_exp, grpc_res, "missed exeption on grpc without scheme and path without leading slash, expected: %s, got: %s" % (
                'ValueError', grpc_res))
        except ValueError:
            pass

    def test_split(self):
        grpc_url = 'grpc://localhost:12321/test/path.launch'
        uri_exp, path_exp = 'localhost:12321', '/test/path.launch'
        uri_res, path_res = url.split(grpc_url)
        self.assertEqual(
            uri_exp, uri_res, "wrong url after split default grpc path, expected: %s, got: %s" % (uri_exp, uri_res))
        self.assertEqual(path_exp, path_res, "wrong path after split default grpc path, expected: %s, got: %s" % (
            path_exp, path_res))
        grpc_url = 'grpc://localhost:12321/test/path.launch'
        uri_exp, path_exp = 'grpc://localhost:12321', '/test/path.launch'
        uri_res, path_res = url.split(grpc_url, True)
        self.assertEqual(
            uri_exp, uri_res, "wrong url after split default grpc path with request also a scheme, expected: %s, got: %s" % (uri_exp, uri_res))
        self.assertEqual(path_exp, path_res, "wrong path after split default grpc path, expected: %s, got: %s" % (
            path_exp, path_res))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestUrlLib)

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
import rospkg

from fkie_master_discovery.common import masteruri_from_master
from fkie_node_manager_daemon import host
from fkie_node_manager_daemon import url

PKG = 'fkie_node_manager_daemon'


class TestUrlLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.test_include_file = "%s/resources/include_dummy.launch" % os.getcwd()
        self.master_port = host.get_port(masteruri_from_master(True))

    def tearDown(self):
        pass

    def test_nmduri(self):
        masteruri = 'http://localhost:11311/'
        grpc_exp = 'grpc://localhost:12321'
        grpc_url = url.nmduri(masteruri)
        self.assertEqual(grpc_exp, grpc_url, "wrong grpc url from default masteruri, expected: %s, got: %s" % (grpc_exp, grpc_url))
        grpc_exp = 'grpc://%s:%d' % (host.get_hostname(masteruri_from_master(True)), self.master_port + url.NMD_SERVER_PORT_OFFSET)
        grpc_url = url.nmduri('')
        self.assertEqual(grpc_exp, grpc_url, "wrong grpc url from requested masteruri, expected: %s, got: %s" % (grpc_exp, grpc_url))

    def test_masteruri(self):
        grpc_url = 'grpc://localhost:12321'
        muri_exp = 'http://localhost:11311/'
        muri_res = url.masteruri(grpc_url)
        self.assertEqual(muri_exp, muri_res, "wrong masteruri from default grpc url, expected: %s, got: %s" % (muri_exp, muri_res))
        grpc_url = ''
        muri_exp = masteruri_from_master(True)
        muri_res = url.masteruri(grpc_url)
        self.assertEqual(muri_exp, muri_res, "wrong masteruri from empty grpc url, expected: %s, got: %s" % (muri_exp, muri_res))
        grpc_url = 'localhost:1232'
        muri_exp = ''
        try:
            muri_res = url.masteruri(grpc_url)
            self.assertEqual(muri_exp, muri_res, "missed exeption on grpc url without scheme, expected: %s, got: %s" % ('ValueError', muri_res))
        except ValueError:
            pass

    def test_nmdport(self):
        grpc_url = 'grpc://localhost:12321'
        port_exp = 12321
        port_res = url.nmdport(grpc_url)
        self.assertEqual(port_exp, port_res, "wrong port from default grpc url, expected: %s, got: %s" % (port_exp, port_res))
        grpc_url = ''
        port_exp = self.master_port + url.NMD_SERVER_PORT_OFFSET
        port_res = url.nmdport(grpc_url)
        self.assertEqual(port_exp, port_res, "wrong port from empty grpc url, expected: %s, got: %s" % (port_exp, port_res))
        grpc_url = 'http://localhost:11311'
        port_exp = 12321
        port_res = url.nmdport(grpc_url)
        self.assertEqual(port_exp, port_res, "wrong nmd port from masteruri, expected: %s, got: %s" % (port_exp, port_res))

#     def test_grpc_create_url(self):
#         masteruri = 'http://localhost:11311'
#         path = 'test/path.launch'
#         grpc_exp = 'grpc://localhost:12321/test/path.launch'
#         grpc_res = url.grpc_create_url(masteruri, path)
#         self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from default masteruri and without leading slash, expected: %s, got: %s" % (grpc_exp, grpc_res))
#         path = '/test/path.launch'
#         grpc_exp = 'grpc://localhost:12321/test/path.launch'
#         grpc_res = url.grpc_create_url(masteruri, path)
#         self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from default masteruri and with leading slash, expected: %s, got: %s" % (grpc_exp, grpc_res))

    def test_nmduri_from_path(self):
        grpc_path = 'grpc://localhost:12321/test/path.launch'
        grpc_exp = 'grpc://localhost:12321'
        grpc_res = url.nmduri_from_path(grpc_path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from default grpc path, expected: %s, got: %s" % (grpc_exp, grpc_res))
        grpc_path = 'localhost:12321/test/path.launch'
        grpc_exp = ''
        try:
            grpc_res = url.nmduri_from_path(grpc_path)
            self.assertEqual(grpc_exp, grpc_res, "missed exeption on grpc url without scheme, expected: %s, got: %s" % ('ValueError', grpc_res))
        except ValueError:
            pass

    def test_join(self):
        grpc_url = 'grpc://localhost:12321'
        path = 'grpc://localhost:12321/test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from path with grpc scheme, expected: %s, got: %s" % (grpc_exp, grpc_res))
        grpc_url = 'grpc://localhost:12321'
        path = '/test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from grpc scheme and path with leading slash, expected: %s, got: %s" % (grpc_exp, grpc_res))
        grpc_url = 'grpc://localhost:12321'
        path = 'test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        grpc_res = url.join(grpc_url, path)
        self.assertEqual(grpc_exp, grpc_res, "wrong grpc url from grpc scheme and path without leading slash, expected: %s, got: %s" % (grpc_exp, grpc_res))
        grpc_url = 'localhost:12321'
        path = 'test/path.launch'
        grpc_exp = 'grpc://localhost:12321/test/path.launch'
        try:
            grpc_res = url.join(grpc_url, path)
            self.assertEqual(grpc_exp, grpc_res, "missed exeption on grpc without scheme and path without leading slash, expected: %s, got: %s" % ('ValueError', grpc_res))
        except ValueError:
            pass

    def test_split(self):
        grpc_url = 'grpc://localhost:12321/test/path.launch'
        uri_exp, path_exp = 'localhost:12321', '/test/path.launch' 
        uri_res, path_res = url.split(grpc_url)
        self.assertEqual(uri_exp, uri_res, "wrong url after split default grpc path, expected: %s, got: %s" % (uri_exp, uri_res))
        self.assertEqual(path_exp, path_res, "wrong path after split default grpc path, expected: %s, got: %s" % (path_exp, path_res))
        grpc_url = ''
        uri_exp, path_exp = '%s:%d' % (host.get_hostname(masteruri_from_master(True)), self.master_port + url.NMD_SERVER_PORT_OFFSET), '' 
        uri_res, path_res = url.split(grpc_url)
        self.assertEqual(uri_exp, uri_res, "wrong url after split empty grpc path, expected: %s, got: %s" % (uri_exp, uri_res))
        self.assertEqual(path_exp, path_res, "wrong path after split empty grpc path, expected: %s, got: %s" % (path_exp, path_res))
        grpc_url = 'grpc://localhost:12321/test/path.launch'
        uri_exp, path_exp = 'grpc://localhost:12321', '/test/path.launch' 
        uri_res, path_res = url.split(grpc_url, True)
        self.assertEqual(uri_exp, uri_res, "wrong url after split default grpc path with request also a scheme, expected: %s, got: %s" % (uri_exp, uri_res))
        self.assertEqual(path_exp, path_res, "wrong path after split default grpc path, expected: %s, got: %s" % (path_exp, path_res))

if __name__ == '__main__':
     import rosunit
     rosunit.unitrun(PKG, os.path.basename(__file__), TestUrlLib)


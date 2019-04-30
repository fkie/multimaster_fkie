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

from fkie_node_manager_daemon.common import get_cwd, find_included_files, interpret_path, package_name

PKG = 'fkie_node_manager_daemon'


class TestCommonLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.nm_path = interpret_path("$(find fkie_node_manager_daemon)/")
        self.res_dir = 'resources'
        if os.path.exists(os.path.join(self.nm_path, 'tests')):
            self.res_dir = 'tests/resources'
        self.test_include_file = "%s%s/include_dummy.launch" % (self.nm_path, self.res_dir)

    def tearDown(self):
        pass

    def test_get_cwd(self):
        test_path = '/this/is/path/to'
        result_path = get_cwd('node', '%s/bin' % test_path)
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'node' type, expected: %s, got: %s" % (test_path, result_path))
        test_path = os.getcwd()
        result_path = get_cwd('cwd', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'cwd' type, expected: %s, got: %s" % (test_path, result_path))
        test_path = rospkg.get_ros_root()
        result_path = get_cwd('ros-root', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'ros-root' type, expected: %s, got: %s" % (test_path, result_path))
        test_path = rospkg.get_ros_home()
        result_path = get_cwd('', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from empty type, expected: %s, got: %s" % (test_path, result_path))

    def test_package_name(self):
        pkg = package_name(self.nm_path)
        self.assertEqual('fkie_node_manager_daemon', pkg[0], "wrong package name, expected: %s, got: %s" % ('fkie_node_manager_daemon', pkg[0]))
        pkg_dir = self.nm_path
        self.assertEqual(pkg_dir, pkg[1], "wrong package path, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test cache
        pkg = package_name(self.nm_path)
        self.assertEqual('fkie_node_manager_daemon', pkg[0], "wrong package name from cache, expected: %s, got: %s" % ('fkie_node_manager_daemon', pkg[0]))
        self.assertEqual(pkg_dir, pkg[1], "wrong package path from cache, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test invalid path
        pkg = package_name('INVALID')
        self.assertEqual(None, pkg[0], "wrong package name, expected: %s, got: %s" % (None, pkg[0]))
        self.assertEqual(None, pkg[1], "wrong package name, expected: %s, got: %s" % (None, pkg[1]))

    def test_interpret_path(self):
        text_path = "$(find fkie_node_manager_daemon)/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "resources/include_dummy.launch"
        path = interpret_path(text_path, self.nm_path)
        exp_path = os.path.join(self.nm_path, text_path)
        self.assertEqual(exp_path, path, "wrong interpreted relative path, expected: %s, got: %s" % (exp_path, path))
        text_path = "pkg://fkie_node_manager_daemon/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted pkg:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "package://fkie_node_manager_daemon/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted package:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "file://tests/resources/include_dummy.launch"
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted file:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "pkg://fkie_node_manager_daemon///include_dummy.launch"
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path with replace the subdirectory by `///`, expected: %s, got: %s" % (self.test_include_file, path))
        
        text_path = "$(find invalid_name)/%s/include_dummy.launch" % self.res_dir
        self.assertRaises(rospkg.ResourceNotFound, interpret_path, text_path, "No rospkg.ResourceNotFound raises on invalid pacakge name")

        text_path = "some other --args here '$(find fkie_node_manager_daemon)/%s/include_dummy.launch'" % self.res_dir
        path = interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (self.test_include_file, path))

    def test_include_files(self):
        file_list = [file_tuple for file_tuple in find_included_files(self.test_include_file, unique=True)]
        self.assertEqual(5, len(file_list), "Count of unique included files is wrong, expected: %d, got: %d" % (5, len(file_list)))
        file_list = [file_tuple for file_tuple in find_included_files(self.test_include_file, recursive=False, unique=True)]
        self.assertEqual(3, len(file_list), "Count of unique included files while not recursive search is wrong, expected: %d, got: %d" % (3, len(file_list)))
        file_list = [file_tuple for file_tuple in find_included_files(self.test_include_file, unique=False)]
        self.assertEqual(10, len(file_list), "Count of included files is wrong, expected: %d, got: %d" % (10, len(file_list)))
        self.assertEqual(6, file_list[0].line_number, "Wrong line number of first included file, expected: %d, got: %d" % (6, file_list[0].line_number))
        self.assertEqual(10, file_list[2].line_number, "Wrong line number of second included file, expected: %d, got: %d" % (10, file_list[2].line_number))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestCommonLib)


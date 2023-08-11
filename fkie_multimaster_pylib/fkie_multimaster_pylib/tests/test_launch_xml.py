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
import rospkg

from fkie_multimaster_pylib import ros_pkg
from fkie_multimaster_pylib.launch import xml

PKG = 'fkie_multimaster_msgs'


class TestLaunchXmlLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.pkg_path = xml.interpret_path("$(find %s)/" % PKG)
        self.nmgr_path = xml.interpret_path("$(find fkie_node_manager_daemon)/")
        self.res_dir = 'tests/test_launch'
        if os.path.exists(os.path.join(self.pkg_path, '%s/tests' % PKG)):
            self.res_dir = '%s/tests/test_launch' % PKG
        self.test_include_file = "%s/include_dummy.launch" % os.path.join(
            self.pkg_path, self.res_dir)
        self.test_wrong_include_file = "%s/test_wrong_include.launch" % os.path.join(
            self.pkg_path, self.res_dir)

    def tearDown(self):
        pass

    def test_includefile_repr(self):
        inc_file = xml.IncludedFile(
            "path_or_str", 1, "inc_path", False, "raw_inc_path", 0, {'one': 'empty'}, -1)
        inc_file_str = "<IncludedFile  from=path_or_str line_number=1 inc_path=inc_path raw_inc_path=raw_inc_path exists=False size=-1 rec_depth=0 args={'one': 'empty'} />"
        self.assertEqual(inc_file_str, '%s' % inc_file, "wrong _repr__ from IncludedFile, expected: %s, got: %s" % (
            inc_file_str, '%s' % inc_file))

    def test_interpret_path(self):
        text_path = "$(find fkie_multimaster_msgs)/%s/include_dummy.launch" % self.res_dir
        path = xml.interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (
            self.test_include_file, path))
        text_path = "resources/include_dummy.launch"
        path = xml.interpret_path(text_path, self.pkg_path)
        exp_path = os.path.join(self.pkg_path, text_path)
        self.assertEqual(
            exp_path, path, "wrong interpreted relative path, expected: %s, got: %s" % (exp_path, path))
        text_path = "pkg://fkie_multimaster_msgs/%s/include_dummy.launch" % self.res_dir
        path = xml.interpret_path(text_path, self.pkg_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted pkg:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "package://fkie_multimaster_msgs/%s/include_dummy.launch" % self.res_dir
        path = xml.interpret_path(text_path, self.pkg_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted package:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "file://fkie_multimaster_msgs/tests/test_launch/include_dummy.launch"
        path = xml.interpret_path(text_path, self.pkg_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted file:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "pkg://fkie_multimaster_msgs///include_dummy.launch"
        path = xml.interpret_path(text_path, self.pkg_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted path with replace the subdirectory by `///`, expected: %s, got: %s" % (self.test_include_file, path))

        text_path = "$(find invalid_name)/%s/include_dummy.launch" % self.res_dir
        self.assertRaises(rospkg.ResourceNotFound, xml.interpret_path, text_path,
                          "No rospkg.ResourceNotFound raises on invalid pacakge name")

        text_path = "some other --args here '$(find fkie_multimaster_msgs)/%s/include_dummy.launch'" % self.res_dir
        path = xml.interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (
            self.test_include_file, path))

    def test_replace_paths(self):
        text_path = "$(find fkie_multimaster_msgs)/fkie_multimaster_msgs/tests/test_launch/include_dummy.launch, $(find fkie_node_manager_daemon)/launch/demo_bar.launch"
        path = xml.replace_paths(text_path)
        nm_path = os.path.dirname(self.pkg_path.rstrip(os.path.sep))
        path_exp = "%s/fkie_multimaster_msgs/fkie_multimaster_msgs/tests/test_launch/include_dummy.launch, %s/launch/demo_bar.launch" % (
            nm_path, self.nmgr_path.rstrip(os.path.sep))
        self.assertEqual(
            path_exp, path, "wrong replace_paths, expected: %s, got: %s" % (path_exp, path))

    def test_get_arg_names(self):
        args = xml.get_arg_names('')
        self.assertEqual(
            0, len(args), "wrong get_arg_names, expected: %d, got: %d" % (0, len(args)))
        args = xml.get_arg_names('some text $(arg test_name_arg)')
        self.assertEqual(
            1, len(args), "wrong get_arg_names, expected: %d, got: %d" % (1, len(args)))
        self.assertEqual('test_name_arg', args[0], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_arg', args[0]))
        args = xml.get_arg_names(
            'some text $(arg test_name_arg)/\n$(arg test_name_second) ')
        self.assertEqual(
            2, len(args), "wrong get_arg_names, expected: %d, got: %d" % (2, len(args)))
        self.assertEqual('test_name_arg', args[0], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_arg', args[0]))
        self.assertEqual('test_name_second', args[1], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_second', args[1]))

    def test_include_files(self):
        file_list = [file_tuple for file_tuple in xml.find_included_files(
            self.test_include_file, unique=True)]
        self.assertEqual(5, len(
            file_list), "Count of unique included files is wrong, expected: %d, got: %d" % (5, len(file_list)))
        file_list = [file_tuple for file_tuple in xml.find_included_files(
            self.test_include_file, recursive=False, unique=True)]
        self.assertEqual(3, len(
            file_list), "Count of unique included files while not recursive search is wrong, expected: %d, got: %d" % (3, len(file_list)))
        file_list = [file_tuple for file_tuple in xml.find_included_files(
            self.test_include_file, unique=False)]
        self.assertEqual(10, len(
            file_list), "Count of included files is wrong, expected: %d, got: %d" % (10, len(file_list)))
        self.assertEqual(6, file_list[0].line_number, "Wrong line number of first included file, expected: %d, got: %d" % (
            6, file_list[0].line_number))
        self.assertEqual(10, file_list[2].line_number, "Wrong line number of second included file, expected: %d, got: %d" % (
            10, file_list[2].line_number))
        no_file_list = [file_tuple for file_tuple in xml.find_included_files(
            self.test_wrong_include_file, unique=True)]
        self.assertEqual(0, len(
            no_file_list), "Count of 'wrong' included files is wrong, expected: %d, got: %d" % (0, len(no_file_list)))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestLaunchXmlLib)

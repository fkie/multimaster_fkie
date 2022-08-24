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
from datetime import tzinfo, timedelta, datetime

from fkie_node_manager_daemon.common import IncludedFile
from fkie_node_manager_daemon.common import sizeof_fmt
from fkie_node_manager_daemon.common import formated_ts
from fkie_node_manager_daemon.common import get_packages
from fkie_node_manager_daemon.common import get_cwd, find_included_files, get_pkg_path, interpret_path, package_name
from fkie_node_manager_daemon.common import replace_paths
from fkie_node_manager_daemon.common import get_arg_names

PKG = 'fkie_node_manager_daemon'


class TestCommonLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.nm_path = get_pkg_path("fkie_node_manager_daemon")
        #self.nmgr_path = interpret_path("$(find-pkg-share fkie_node_manager)/")
        self.res_dir = 'resources'
        # if os.path.exists(os.path.join(self.nm_path, 'share/fkie_node_manager_daemon/resources')):
        #    self.res_dir = 'share/fkie_node_manager_daemon/resources'
        self.test_include_file = "%s/%s/include_dummy.launch" % (
            self.nm_path, 'share/fkie_node_manager_daemon/resources')

    def tearDown(self):
        pass

    def test_includefile_repr(self):
        inc_file = IncludedFile(
            "path_or_str", 1, "inc_path", False, "raw_inc_path", 0, {'one': 'empty'}, -1)
        inc_file_str = "<IncludedFile  from=path_or_str line_number=1 inc_path=inc_path raw_inc_path=raw_inc_path exists=False size=-1 rec_depth=0 args={'one': 'empty'} />"
        self.assertEqual(inc_file_str, '%s' % inc_file, "wrong _repr__ from IncludedFile, expected: %s, got: %s" % (
            inc_file_str, '%s' % inc_file))

    def test_sizeof_fmt(self):
        sizeof_str = sizeof_fmt(12345678)
        sizeof_str_res = "12MiB"
        self.assertEqual(sizeof_str_res, sizeof_str, "wrong sizeof_fmt, expected: %s, got: '%s'" % (
            sizeof_str_res, sizeof_str))

    def test_formated_ts(self):

        class UTC(tzinfo):
            def utcoffset(self, dt):
                return timedelta(0)

            def tzname(self, dt):
                return "UTC"

            def dst(self, dt):
                return timedelta(0)

        tsstr_ff = formated_ts(
            1557480759.608808, with_date=False, with_nanosecs=False, tz=UTC())
        tsstr_ff_res = "09:32:39"
        self.assertEqual(tsstr_ff_res, tsstr_ff, "wrong formated_ts(value, False, False), expected: %s, got: %s" % (
            tsstr_ff_res, tsstr_ff))
        tsstr_tf = formated_ts(
            1557480759.608808, with_date=True, with_nanosecs=False, tz=UTC())
        tsstr_tf_res = "09:32:39 (10.05.2019)"
        self.assertEqual(tsstr_tf_res, tsstr_tf, "wrong formated_ts(value, True, False), expected: %s, got: %s" % (
            tsstr_tf_res, tsstr_tf))
        tsstr_tt = formated_ts(
            1557480759.608808, with_date=True, with_nanosecs=True, tz=UTC())
        tsstr_tt_res = "09:32:39.608808 (10.05.2019)"
        self.assertEqual(tsstr_tt_res, tsstr_tt, "wrong formated_ts(value, True, True), expected: %s, got: %s" % (
            tsstr_tt_res, tsstr_tt))
        tsstr_ft = formated_ts(
            1557480759.608808, with_date=False, with_nanosecs=True, tz=UTC())
        tsstr_ft_res = "09:32:39.608808"
        self.assertEqual(tsstr_ft_res, tsstr_ft, "wrong formated_ts(value, False, True), expected: %s, got: %s" % (
            tsstr_ft_res, tsstr_ft))

    def test_get_packages(self):
        path = os.path.dirname(self.nm_path.rstrip(os.path.sep))
        pkg_res = get_packages(path)
        count_exp = 2  # TODO: while port to ros2 we have only 2 package
        if 'industrial_ci' in pkg_res:
            count_exp += 1
        self.assertEqual(count_exp, len(
            pkg_res), "wrong count of get_packages(%s), expected: %d, got: %d -> packages: %s" % (path, count_exp, len(pkg_res), pkg_res))

    def test_get_cwd(self):
        test_path = '/this/is/path/to'
        result_path = get_cwd('node', '%s/bin' % test_path)
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'node' type, expected: %s, got: %s" % (
            test_path, result_path))
        test_path = os.getcwd()
        result_path = get_cwd('cwd', '')
        self.assertEqual(test_path, result_path, "wrong get_cwd from 'cwd' type, expected: %s, got: %s" % (
            test_path, result_path))
        test_path = rospkg.get_ros_root()
        #result_path = get_cwd('ros-root', '')
        #self.assertEqual(test_path, result_path, "wrong get_cwd from 'ros-root' type, expected: %s, got: %s" % (test_path, result_path))
        #test_path = rospkg.get_ros_home()
        #result_path = get_cwd('', '')
        #self.assertEqual(test_path, result_path, "wrong get_cwd from empty type, expected: %s, got: %s" % (test_path, result_path))

    def test_package_name(self):
        pkg = package_name(self.nm_path)
        self.assertEqual('fkie_node_manager_daemon', pkg[0], "wrong package name, expected: %s, got: %s" % (
            'fkie_node_manager_daemon', pkg[0]))
        pkg_dir = "%s/share/fkie_node_manager_daemon" % self.nm_path
        self.assertEqual(
            pkg_dir, pkg[1], "wrong package path, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test cache
        pkg = package_name(self.nm_path)
        self.assertEqual('fkie_node_manager_daemon', pkg[0], "wrong package name from cache, expected: %s, got: %s" % (
            'fkie_node_manager_daemon', pkg[0]))
        self.assertEqual(
            pkg_dir, pkg[1], "wrong package path from cache, expected: %s, got: %s" % (pkg_dir, pkg[1]))
        # test invalid path
        pkg = package_name('INVALID')
        self.assertEqual(
            '', pkg[0], "wrong package name, expected: %s, got: %s" % ('', pkg[0]))
        self.assertEqual(
            '', pkg[1], "wrong package name, expected: %s, got: %s" % ('', pkg[1]))

    def test_interpret_path(self):
        text_path = "$(find-pkg-share fkie_node_manager_daemon)/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (
            self.test_include_file, path))
        text_path = "resources/include_dummy.launch"
        path = interpret_path(text_path, self.nm_path)
        exp_path = os.path.join(self.nm_path, text_path)
        self.assertEqual(
            exp_path, path, "wrong interpreted relative path, expected: %s, got: %s" % (exp_path, path))
        text_path = "pkg://fkie_node_manager_daemon/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted pkg:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "package://fkie_node_manager_daemon/%s/include_dummy.launch" % self.res_dir
        path = interpret_path(text_path, self.nm_path)
        self.assertEqual(self.test_include_file, path,
                         "wrong interpreted package:// path, expected: %s, got: %s" % (self.test_include_file, path))
#        text_path = "file://resources/include_dummy.launch"
#        path = interpret_path(text_path, self.nm_path)
#        self.assertEqual(self.test_include_file, path, "wrong interpreted file:// path, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "pkg://fkie_node_manager_daemon///include_dummy.launch"
        path = interpret_path(text_path, self.nm_path)
#        self.assertEqual(self.test_include_file, path, "wrong interpreted path with replace the subdirectory by `///`, expected: %s, got: %s" % (self.test_include_file, path))
        text_path = "$(find-pkg-share invalid_name)/%s/include_dummy.launch" % self.res_dir
        self.assertRaises(LookupError, interpret_path, text_path,
                          "No rospkg.ResourceNotFound raises on invalid pacakge name")

        text_path = "some other --args here '$(find-pkg-share fkie_node_manager_daemon)/%s/include_dummy.launch'" % self.res_dir
        path = interpret_path(text_path)
        self.assertEqual(self.test_include_file, path, "wrong interpreted path, expected: %s, got: %s" % (
            self.test_include_file, path))

    def test_replace_paths(self):
        text_path = "$(find-pkg-share fkie_node_manager_daemon)/resources/include_dummy.launch, $(find-pkg-share fkie_node_manager_daemon)/launch/demo_bar.launch"
        path = replace_paths(text_path)
        nm_path = os.path.dirname(self.nm_path.rstrip(os.path.sep))
        path_exp = "%s/fkie_node_manager_daemon/share/fkie_node_manager_daemon/resources/include_dummy.launch, %s/fkie_node_manager_daemon/share/fkie_node_manager_daemon/launch/demo_bar.launch" % (
            nm_path, nm_path)
        self.assertEqual(
            path_exp, path, "wrong replace_paths, expected: %s, got: %s" % (path_exp, path))

    def test_get_arg_names(self):
        args = get_arg_names('')
        self.assertEqual(
            0, len(args), "wrong get_arg_names, expected: %d, got: %d" % (0, len(args)))
        args = get_arg_names('some text $(arg test_name_arg)')
        self.assertEqual(
            1, len(args), "wrong get_arg_names, expected: %d, got: %d" % (1, len(args)))
        self.assertEqual('test_name_arg', args[0], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_arg', args[0]))
        args = get_arg_names(
            'some text $(arg test_name_arg)/\n$(arg test_name_second) ')
        self.assertEqual(
            2, len(args), "wrong get_arg_names, expected: %d, got: %d" % (2, len(args)))
        self.assertEqual('test_name_arg', args[0], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_arg', args[0]))
        self.assertEqual('test_name_second', args[1], "wrong get_arg_names, expected: %s, got: %s" % (
            'test_name_second', args[1]))

    def test_include_files(self):
        # TODO: included files does not work
        return
        file_list = [file_tuple for file_tuple in find_included_files(
            self.test_include_file, unique=True)]
        self.assertEqual(5, len(
            file_list), "Count of unique included files is wrong, expected: %d, got: %d" % (5, len(file_list)))
        file_list = [file_tuple for file_tuple in find_included_files(
            self.test_include_file, recursive=False, unique=True)]
        self.assertEqual(3, len(
            file_list), "Count of unique included files while not recursive search is wrong, expected: %d, got: %d" % (3, len(file_list)))
        file_list = [file_tuple for file_tuple in find_included_files(
            self.test_include_file, unique=False)]
        self.assertEqual(10, len(
            file_list), "Count of included files is wrong, expected: %d, got: %d" % (10, len(file_list)))
        self.assertEqual(6, file_list[0].line_number, "Wrong line number of first included file, expected: %d, got: %d" % (
            6, file_list[0].line_number))
        self.assertEqual(10, file_list[2].line_number, "Wrong line number of second included file, expected: %d, got: %d" % (
            10, file_list[2].line_number))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestCommonLib)

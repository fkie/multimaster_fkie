#!/usr/bin/env python
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

import math
import os
import unittest
import rospy
import time

from fkie_node_manager_daemon.common import interpret_path
from fkie_node_manager_daemon.file_item import FileItem
from fkie_node_manager_daemon.server import GrpcServer
import fkie_node_manager_daemon.remote as remote
import fkie_node_manager_daemon.file_stub as fstub
import fkie_node_manager_daemon.launch_stub as lstub
import fkie_node_manager_daemon.exceptions as exceptions

PKG = 'fkie_node_manager_daemon'


class TestGrpcServer(unittest.TestCase):
    '''
    '''

    server = GrpcServer()

    @classmethod
    def setUpClass(cls):
        cls.server.start('localhost:12311')

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def setUp(self):
        url = 'localhost:12311'
        channel = remote.get_insecure_channel(url)
        self.fs = fstub.FileStub(channel)
        self.ls = lstub.LaunchStub(channel)
        self.test_include_file = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/include_dummy.launch")
        path = interpret_path("$(find fkie_node_manager_daemon)/../../../build")
        self.test_save_file = "%s/tmp_save_dummy.launch" % path
        self.test_rename_from_file = "%s/tmp_rename_from_dummy.launch" % path
        self.test_rename_to_file = "%s/tmp_rename_to_dummy.launch" % path

    def tearDown(self):
        try:
            os.remove(self.test_save_file)
        except Exception:
            pass
        try:
            os.remove(self.test_rename_from_file)
        except Exception:
            pass
        try:
            os.remove(self.test_rename_to_file)
        except Exception:
            pass

    def test_list_path(self):
        rpp = os.environ['ROS_PACKAGE_PATH'].split(':')
        result = self.fs.list_path('')
        self.assertEqual(len(result), len(rpp), "wrong count of items in the root path, got: %s, expected: %s" % (len(result), len(rpp)))
        count_dirs = [p.path for p in result if p.type in [FileItem.DIR, FileItem.PACKAGE]]
        self.assertEqual(len(count_dirs), len(rpp), "wrong count of directories in the root path, got: %s, expected: %s" % (len(count_dirs), len(rpp)))
        try:
            result = self.fs.list_path('/xyz')
            self.fail("`list_path` did not raises `OSError` on not existing path `/xyz`")
        except OSError:
            pass
        except Exception as err:
            self.fail("`list_path` raises wrong Exception on not existing path `/xyz`, got: %s, expected: `OSError`" % type(err))

    def test_get_file_content(self):
        file_size, file_mtime, file_content = self.fs.get_file_content(self.test_include_file)
        self.assertEqual(file_size, 892, "wrong returned file size, got: %s, expected: %s" % (file_size, 892))
        self.assertEqual(len(file_content), 892, "wrong size of the content, got: %s, expected: %s" % (len(file_content), 892))
        try:
            result = self.fs.get_file_content('xyz')
            self.fail("`get_file_content` did not raises `IOError` on not existing path `/xyz`")
        except IOError:
            pass
        except Exception as err:
            self.fail("`get_file_content` raises wrong Exception on not existing path `/xyz`, got: %s, expected: `IOError`" % type(err))

    def test_get_included_files(self):
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/include_dummy.launch")
        file_list = [file_tuple for file_tuple in self.ls.get_included_files_set(path, recursive=True, include_pattern=[])]
#        for linenr, path, exists, file_list in inc_files:
        self.assertEqual(len(file_list), 5, "Count of unique included, recursive files is wrong, got: %d, expected: %d" % (len(file_list), 5))
        file_list = [file_tuple for file_tuple in self.ls.get_included_files_set(path, recursive=False, include_pattern=[])]
        self.assertEqual(len(file_list), 3, "Count of unique included files while not recursive search is wrong, got: %d, expected: %d" % (len(file_list), 3))
        file_list = [file_tuple for file_tuple in self.ls.get_included_files(path, recursive=False, include_pattern=[])]
        self.assertEqual(len(file_list), 6, "Count of recursive, not unique included files is wrong, got: %d, expected: %d" % (len(file_list), 6))
        file_list = [file_tuple for file_tuple in self.ls.get_included_files(path, recursive=True, include_pattern=[])]
        self.assertEqual(len(file_list), 10, "Count of recursive included files is wrong, got: %d, expected: %d" % (len(file_list), 10))
        self.assertEqual(file_list[0].line_number, 6, "Wrong line number of first included file, got: %d, expected: %d" % (file_list[0].line_number, 6))
        # self.assertEqual(file_list[0][1], file_list[0][3][0][1], "Wrong root path of second included file, expected: %s, got: %s" % (file_list[0][1], file_list[0][3][0][1]))
        self.assertEqual(file_list[1].line_number, 4, "Wrong line number of second included file, got: %d, expected: %d" % (file_list[1].line_number, 4))
        self.assertEqual(file_list[2].line_number, 10, "Wrong line number of third included file, got: %d, expected: %d" % (file_list[2].line_number, 10))

    def _test_load_launch(self, unload=True):
        args = {}
        path = ''
        request_args = True
        nexttry = True
        ok = False
        launch_file = ''
        package = 'fkie_node_manager_daemon'
        launch = 'description_example.launch'
        try:
            launch_file, _argv = self.ls.load_launch(package, launch, path=path, args=args, request_args=request_args)
            self.fail("`load_launch` did not raises `exceptions.LaunchSelectionRequest` on multiple launch files")
        except exceptions.LaunchSelectionRequest as _lsr:
            path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/description_example.launch")
        except Exception as err:
            self.fail("`load_launch` raises wrong Exception on multiple launch files, got: %s, expected: `exceptions.LaunchSelectionRequest`: %s" % (type(err), err))
        try:
            launch_file, _argv = self.ls.load_launch(package, launch, path=path, args=args, request_args=request_args)
            self.fail("`load_launch` did not raises `exceptions.ParamSelectionRequest` on args requests")
        except exceptions.AlreadyOpenException as aoe:
            # it is already open from other tests
            return
        except exceptions.ParamSelectionRequest as psr:
            request_args = False
            args = psr.choices
        except Exception as err:
            import traceback
            self.fail("`load_launch` raises wrong Exception on args requests, got: %s, expected: `exceptions.ParamSelectionReques`: %s" % (type(err), traceback.format_exc()))
        request_args = False
        try:
            launch_file, _argv = self.ls.load_launch(package, launch, path=path, args=args, request_args=request_args)
        except Exception as err:
            self.fail("`load_launch` raises unexpected exception, got: %s, expected: no error" % type(err))
        try:
            launch_file, _argv = self.ls.load_launch(package, launch, path=path, args=args, request_args=request_args)
            self.fail("`load_launch` did not raises `exceptions.AlreadyOpenException` on load an already loaded file")
        except exceptions.AlreadyOpenException as aoe:
            ao_path = aoe.path
            ao_error = aoe.error
        except Exception as err:
            self.fail("`load_launch` raises wrong Exception on second reload, got: %s, expected: `exceptions.AlreadyOpenException`: %s" % (type(err), err))
        if unload:
            self.ls.unload_launch(launch_file)

    def test_get_nodes(self):
        self._test_load_launch(unload=False)
        result = self.ls.get_nodes()
        self.assertEqual(len(result), 1, "wrong count of descriptions in get_nodes, got: %d, expected: %d" % (len(result), 1))
        self.assertEqual(len(result[0].nodes), 15, "wrong count of nodes in get_nodes, got: %d, expected: %d" % (len(result[0].nodes), 15))
        self.assertEqual(len(result[0].robot_descriptions), 0, "wrong count of robot descriptions in get_nodes, got: %d, expected: %d" % (len(result[0].robot_descriptions), 0))
        # get robot descriptions
        result = self.ls.get_nodes(request_description=True)
        self.assertEqual(len(result[0].robot_descriptions), 2, "wrong count of robot descriptions in get_nodes, got: %d, expected: %d" % (len(result[0].robot_descriptions), 2))

    def test_reload_launch(self):
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/include_dummy.launch")
        try:
            launch_file, _argv = self.ls.reload_launch(path)
            self.fail("`load_launch` did not raises `exceptions.ResourceNotFoundt` on reload launch file")
        except exceptions.ResourceNotFound as rnf:
            pass
        except Exception as err:
            self.fail("`load_launch` raises wrong Exception on reload launch file, got: %s, expected: `exceptions.ResourceNotFoundt`: %s" % (type(err), err))

    def test_save_file_content(self):
        content = b'This is a temporary content to test the save functionality'
        self.fs.FILE_CHUNK_SIZE = 3
        acks_expected_count = math.ceil(float(len(content)) / float(self.fs.FILE_CHUNK_SIZE))
        acks = self.fs.save_file_content(self.test_save_file, content, 0)
        self.assertEqual(len(acks), acks_expected_count, "wrong returned count of acks, got: %d, expected: %d" % (len(acks), acks_expected_count))
        file_size, file_mtime, file_content = self.fs.get_file_content(self.test_save_file)
        self.assertEqual(file_size, len(content), "wrong size of saved file, got: %d, expected: %d" % (file_size, len(content)))
        self.assertEqual(file_content, content, "wrong content of saved file, got: '%s', expected: '%s'" % (file_content, content))

    def test_rename_file(self):
        with open(self.test_rename_from_file, 'w') as testfile:
            testfile.write('test content for file which will be renamed')
        self.fs.rename(self.test_rename_from_file, self.test_rename_to_file)
        if not os.path.exists(self.test_rename_to_file):
            self.fail("After `rename` the target file does not exists")

    def _test_run_node(self):
#        self.test_load_launch(unload=False)
        self.ls.start_node('/example/gps')

if __name__ == '__main__':
    import rostest
    rospy.init_node("test_grpc_server")
    rostest.rosrun(PKG, os.path.basename(__file__), TestGrpcServer)
#     import rosunit
#     rosunit.unitrun(PKG, os.path.basename(__file__), TestGrpcServer)

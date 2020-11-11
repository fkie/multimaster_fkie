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
import grpc
import unittest
import time
from grpc.beta._metadata import beta

from fkie_node_manager_daemon.common import interpret_path
import fkie_multimaster_msgs.grpc.launch_pb2 as lmsg
from fkie_node_manager_daemon.launch_servicer import LaunchServicer
from fkie_node_manager_daemon.launch_description import RobotDescription, Capability

PKG = 'fkie_node_manager_daemon'


class DummyContext(grpc.RpcContext):
    """Dummy context used to avoid a creation GRPC deamon."""

    def peer(self):
        return "DUMMY"

    def set_code(self, code):
        pass

    def set_details(self, text):
        pass

    def is_active(self):
        return True

    def time_remaining(self):
        return None

    def cancel(self):
        pass

    def add_callback(self, callback):
        return False

    def invocation_metadata(self):
        return beta([('user-agent', 'DUMMY for test')])


class TestLaunchServicer(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.test_get_content_path = "%s/content_test.txt" % os.getcwd()
        self.current_pose = 0

    def tearDown(self):
        try:
            os.remove(self.test_get_content_path)
        except Exception:
            pass

    def test_get_loaded_files(self):
        ls = LaunchServicer(monitor_servicer=None)
        response_stream = ls.GetLoadedFiles(lmsg.Empty(), DummyContext())
        items = [response for response in response_stream]
        self.assertEqual(len(items), 0, "The list of loaded launch files on startup is not empty!")
#            self.assertEqual(resp.ack.size, self.current_pose, "incorrect transferred file size: %d, expected: %d" % (resp.ack.size, self.current_pose))

    def test_get_included_files(self):
        ls = LaunchServicer(monitor_servicer=None)
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/include_dummy.launch")
        response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(path=path, recursive=True, unique=True, pattern=[]), DummyContext())
        file_list = [response for response in response_stream]
        self.assertEqual(len(file_list), 5, "Count of unique included, recursive  files is wrong, got: %d, expected: %d" % (len(file_list), 5))
        response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(path=path, recursive=False, unique=True, pattern=[]), DummyContext())
        file_list = [response for response in response_stream]
        self.assertEqual(len(file_list), 3, "Count of unique included files while not recursive search is wrong, got: %d, expected: %d" % (len(file_list), 3))
        response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(path=path, recursive=False, unique=False, pattern=[]), DummyContext())
        file_list = [response for response in response_stream]
        self.assertEqual(len(file_list), 6, "Count of not recursive, not unique included files is wrong, expected: %d, got: %d" % (6, len(file_list)))
        response_stream = ls.GetIncludedFiles(lmsg.IncludedFilesRequest(path=path, recursive=True, unique=False, pattern=[]), DummyContext())
        file_list = [response for response in response_stream]
        self.assertEqual(len(file_list), 10, "Count of included files is wrong, got: %d, expected: %d" % (len(file_list), 10))
        self.assertEqual(file_list[0].linenr, 6, "Wrong line number of first included file, got: %d, expected: %d" % (file_list[0].linenr, 6))
        self.assertEqual(file_list[1].linenr, 4, "Wrong line number of second included file, got: %d, expected: %d" % (file_list[1].linenr, 4))
        self.assertEqual(file_list[2].linenr, 10, "Wrong line number of third included file, got: %d, expected: %d" % (file_list[2].linenr, 10))

    def test_load_launch(self):
        ls = LaunchServicer(monitor_servicer=None)
        path = ''
        args = {}
        request_args = True
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='no_file.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), 
                         "wrong status code if launch file not exists, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='unknonwn_package', launch='no_file.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), 
                         "wrong status code if package not exists, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES'), 
                         "wrong status code for multiple launch files, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('MULTIPLE_LAUNCHES'), response.status.error_msg))
        self.assertEqual(len(response.path), 2, "wrong count of multiple launch files, result: %d, expected: %d" % (len(response.path), 2))
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/description_example.launch")
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED'),
                         "wrong status code for request arguments, result: %d, expected: %d, reported error: %s"
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('PARAMS_REQUIRED'), response.status.error_msg))
        args = {arg.name: arg.value for arg in response.args}
        request_args = False
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
                         "wrong status code on successful load, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))

        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path, request_args=request_args), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN'),
                         "wrong status code if file is already open, result: %d, expected: %d, reported error: %s"
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('ALREADY_OPEN'), response.status.error_msg))
        self.assertEqual(response.path[0], path, "Wrong returned path in ALREADY_OPEN reply, expected: %s, got: %s" % (response.path[0], path))

        # test loaded file
        response_stream = ls.GetLoadedFiles(lmsg.Empty(), DummyContext())
        items = [response for response in response_stream]
        self.assertEqual(len(items), 1, "The count of loaded files after successful load is wrong")
        self.assertEqual(items[0].path, path, "Wrong reported loaded file path, got: %s, expected: %s" % (items[0].path, path))
        self.assertEqual(items[0].package, 'fkie_node_manager_daemon', "Wrong reported loaded package, got: %s, expected: %s" % (items[0].package, 'fkie_node_manager_daemon'))
        self.assertEqual(items[0].launch, 'description_example.launch', "Wrong reported loaded launch, got: %s, expected: %s" % (items[0].launch, 'description_example.launch'))

    def test_reload_file(self):
        ls = LaunchServicer(monitor_servicer=None)
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/description_example.launch")
        response = ls.ReloadLaunch(lmsg.LaunchFile(path=path), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'),
                         "wrong status code on reload of not loaded file, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('FILE_NOT_FOUND'), response.status.error_msg))
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
                         "wrong status code on successful load, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))
        response = ls.ReloadLaunch(lmsg.LaunchFile(path=path), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
                         "wrong status code on reload of already loaded file, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))

    def test_get_nodes(self):
        ls = LaunchServicer(monitor_servicer=None)
        path = interpret_path("$(find fkie_node_manager_daemon)/tests/resources/description_example.launch")
        response = ls.LoadLaunch(lmsg.LoadLaunchRequest(package='fkie_node_manager_daemon', launch='description_example.launch', path=path), DummyContext())
        self.assertEqual(response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'),
                         "wrong status code on successful load, result: %d, expected: %d, reported error: %s" 
                         % (response.status.code, lmsg.ReturnStatus.StatusType.Value('OK'), response.status.error_msg))
        response_stream = ls.GetNodes(lmsg.ListNodesRequest(request_description=True), DummyContext())
        nodes = {}
        descriptions = []
        for response in response_stream:
            nodes[response.launch_file] = list(response.node)
            for descr in response.description:
                rd = RobotDescription(machine=descr.machine, robot_name=descr.robot_name, robot_type=descr.robot_type, robot_images=list(descr.robot_images), robot_descr=descr.robot_descr)
                for cap in descr.capabilities:
                    cp = Capability(name=cap.name, namespace=cap.namespace, cap_type=cap.type, images=[img for img in cap.images], description=cap.description, nodes=[n for n in cap.nodes])
                    rd.capabilities.append(cp)
                descriptions.append(rd)
        self.assertEqual(len(nodes), 1, "wrong count of returned launch files for nodes, result: %d, expected: %d"  % (len(nodes), 1))
        self.assertEqual(len(nodes[path]), 15, "wrong count of returned nodes, result: %d, expected: %d"  % (len(nodes[path]), 15))
        self.assertEqual(len(descriptions), 2, "wrong count of descriptions, result: %d, expected: %d"  % (len(descriptions), 2))
        self.assertEqual(descriptions[0].robot_name, 'pc', "wrong robot_name in first description, result: %s, expected: %s, description: %s"  % (descriptions[0].robot_name, 'pc', descriptions[0]))
        self.assertEqual(len(descriptions[0].capabilities), 0, "wrong count of capabilities in first description, result: %d, expected: %d, description: %s"  % (len(descriptions[0].capabilities), 0, descriptions[0]))
        self.assertEqual(len(descriptions[1].capabilities), 9, "wrong count of capabilities in second description, result: %d, expected: %d, description: %s"  % (len(descriptions[1].capabilities), 9, descriptions[1]))

#         launch_manager.test_start_node('/example/test_node')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestLaunchServicer)


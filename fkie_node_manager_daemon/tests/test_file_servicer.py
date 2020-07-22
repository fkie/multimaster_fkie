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

from io import FileIO
import os
import grpc
import unittest
import time
from grpc.beta._metadata import beta

import fkie_multimaster_msgs.grpc.file_pb2 as fmsg
from fkie_node_manager_daemon.common import interpret_path
from fkie_node_manager_daemon.file_servicer import FileServicer

PKG = 'fkie_node_manager_daemon'


class DummyContext(grpc.RpcContext):
    '''Dummy context used to avoid a creation GRPC deamon.'''

    def peer(self):
        return 'DUMMY'

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


class TestFileServiceServicer(unittest.TestCase):
    '''
    '''

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.current_pose = 0
        path = interpret_path('$(find fkie_node_manager_daemon)/../../../build')
        if not os.path.exists(path):
            os.mkdir(path)
        self.test_get_content_path = '%s/tmp_get_content_test.txt' % path
        self.test_save_content_path = '%s/tmp_save_content_test.txt' % path
        self.test_rename_from_file = '%s/tmp_rename_from_dummy.launch' % path
        self.test_rename_to_file = '%s/tmp_rename_to_dummy.launch' % path

    def tearDown(self):
        try:
            os.remove(self.test_get_content_path)
        except Exception:
            pass
        try:
            os.remove(self.test_save_content_path)
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
        fs = FileServicer()
        root_paths = set(os.getenv('ROS_PACKAGE_PATH').split(':'))
#        launch_response = fs.ListPath(fmsg.ListPathRequest(path=''), DummyContext())
#        self.assertEqual(len(root_paths), len(launch_response.items), 'ROS root paths are not equal, expected: %s, got: %s' % (root_paths, launch_response.items))
        launch_response = fs.ListPath(fmsg.ListPathRequest(path=os.getcwd()), DummyContext())
        self.assertEqual(os.getcwd(), launch_response.path, 'reported list for different path: %s, expected: %s' % (launch_response.path, os.getcwd()))
        self.assertEqual(len(os.listdir(os.getcwd())), len(launch_response.items), 'reported different count of items in working directory: %s' % os.getcwd())
        # test cache
        launch_response = fs.ListPath(fmsg.ListPathRequest(path='%s/../..' % os.getcwd()), DummyContext())
        count_dirs = len([d for d in launch_response.items if d.type in [1, 3]])
        launch_response = fs.ListPath(fmsg.ListPathRequest(path='%s/../..' % os.getcwd()), DummyContext())
        self.assertEqual(len([d for d in launch_response.items if d.type in [1, 3]]), count_dirs, 'count of directories from cache is different')
        # test invalid path
        launch_response = fs.ListPath(fmsg.ListPathRequest(path='/unknown'), DummyContext())
        self.assertEqual(0, len(launch_response.items), 'list of invalid path returns more then 0 items')
        self.assertEqual(fmsg.ReturnStatus.StatusType.Value('OS_ERROR'), launch_response.status.code, 'wrong status code if path not exists')

    def test_list_packages(self):
        fs = FileServicer()
        pacakges_response = fs.ListPackages(fmsg.ListPackagesRequest(clear_ros_cache=True), DummyContext())

    def test_get_content(self):
        fs = FileServicer()
        fs.FILE_CHUNK_SIZE = 10
        content_response = fs.GetFileContent(fmsg.GetFileContentRequest(path=''), DummyContext())
        self.assertEqual(next(content_response).status.code, fmsg.ReturnStatus.StatusType.Value('IO_ERROR'), 'wrong status code if path is empty')
        content_response = fs.GetFileContent(fmsg.GetFileContentRequest(path=self.test_get_content_path), DummyContext())
        self.assertEqual(next(content_response).status.code, fmsg.ReturnStatus.StatusType.Value('IO_ERROR'), 'wrong status code if path not exists')
        # create a test file
        test_data = b'This is a test file for get content test.'
        with FileIO(self.test_get_content_path, 'w') as testfile:
            testfile.write(test_data)
        received_data = b''
        content_response = fs.GetFileContent(fmsg.GetFileContentRequest(path=self.test_get_content_path), DummyContext())
        for resp in content_response:
            self.assertEqual(resp.status.code, fmsg.ReturnStatus.StatusType.Value('OK'), 'wrong status code if path exists')
            if resp.file.offset == 0:
                self.assertEqual(resp.file.path, self.test_get_content_path, 'wrong returned path in file content')
                self.assertGreater(resp.file.mtime, 0, 'wrong returned file mtime in file GetFileContentReply: %.1f, expected: >0' % resp.file.mtime)
                self.assertEqual(resp.file.size, len(test_data), 'wrong returned file size in file GetFileContentReply: %d, expected: %d' % (resp.file.size, len(test_data)))
            received_data += resp.file.data
        self.assertEqual(len(received_data), len(test_data), 'wrong returned length of data in file GetFileContentReply: %d, expected: %d' % (len(resp.file.data), len(test_data)))
        self.assertEqual(received_data, test_data, 'wrong returned data in file GetFileContentReply: %s, expected: %s' % (resp.file.data, test_data))
        os.remove(self.test_get_content_path)

    def _read_from_list(self, test_data, path):
        data_len = 0
        for l in test_data:
            data_len += len(l)
        self.current_pose = 0
        for line in test_data:
            content = fmsg.SaveFileContentRequest()
            content.overwrite = True
            content.file.path = path
            content.file.mtime = 0.0  # something not zero to update a not existing file
            content.file.size = data_len
            content.file.offset = self.current_pose
            content.file.data = line
            self.current_pose += len(line)
            yield content

    def test_save_content(self):
        fs = FileServicer()
        test_data = b'This is a test file for save content test.'
        content = fmsg.SaveFileContentRequest()
        content.file.path = self.test_save_content_path
        content.file.mtime = 1.0  # something not zero to update a not existing file
        content.file.size = len(test_data)
        content.file.data = test_data
        save_response = next(fs.SaveFileContent([content], DummyContext()))
        self.assertEqual(save_response.status.code, fmsg.ReturnStatus.StatusType.Value('REMOVED_FILE'), "wrong status code '%d' if file was removed in meantime." % save_response.status.code)
        # save new file
        content.file.mtime = 0
        save_response = next(fs.SaveFileContent([content], DummyContext()))
        self.assertEqual(save_response.status.code, fmsg.ReturnStatus.StatusType.Value('OK'), 'new file was not saved')
        self.assertTrue(os.path.exists(self.test_save_content_path), 'new file was not saved to %s' % self.test_save_content_path)
        self.assertEqual(save_response.ack.mtime, os.path.getmtime(self.test_save_content_path), 'wrong mtime returned after create a new file')
        # sleep to change the mtime in file
        time.sleep(0.3)
        # change file in meantime
        new_utime = time.time()
        os.utime(self.test_save_content_path, (new_utime, new_utime))
        new_test_data = b'This is a changed text for save content test.'
        content.file.mtime = save_response.ack.mtime
        content.file.size = len(new_test_data)
        content.file.data = new_test_data
        save_response = next(fs.SaveFileContent([content], DummyContext()))
        self.assertEqual(save_response.status.code, fmsg.ReturnStatus.StatusType.Value('CHANGED_FILE'), 'wrong status code if file was changed in meantime.')
        # overwrite the changed file
        content.overwrite = True
        save_response = next(fs.SaveFileContent([content], DummyContext()))
        self.assertEqual(save_response.status.code, fmsg.ReturnStatus.StatusType.Value('OK'), 'file was not overwritten')
        self.assertEqual(save_response.ack.mtime, os.path.getmtime(self.test_save_content_path), 'wrong mtime returned after overwrite file')
        with FileIO(self.test_save_content_path, 'r') as outfile:
            self.assertEqual(new_test_data, outfile.read(), 'wrong content in file')
        # try to save in root folder
        content.file.path = '/content_test.txt'
        content.file.mtime = 0
        save_response = next(fs.SaveFileContent([content], DummyContext()))
        if save_response.status.code == fmsg.ReturnStatus.StatusType.Value('OK'):
            # test in industrial ci, use source folder
            content.file.path = interpret_path('$(find fkie_node_manager_daemon)/') + content.file.path
            save_response = next(fs.SaveFileContent([content], DummyContext()))
        self.assertEqual(save_response.status.code, fmsg.ReturnStatus.StatusType.Value('IO_ERROR'), 'save in root folder returns a wrong result: %d, expected: %d' % (save_response.status.code, fmsg.ReturnStatus.StatusType.Value('IO_ERROR')))
        # save file in more chunks
        test_data = [b'First line.\n', b'Second line.\n', b'Third line.\n']
        for resp in fs.SaveFileContent(self._read_from_list(test_data, self.test_save_content_path), DummyContext()):
            self.assertEqual(resp.status.code, fmsg.ReturnStatus.StatusType.Value('OK'), 'file was not overwritten, result code: %d' % resp.status.code)
            self.assertEqual(resp.ack.size, self.current_pose, 'incorrect transferred file size: %d, expected: %d' % (resp.ack.size, self.current_pose))
        with FileIO(self.test_save_content_path, 'r') as outfile:
            self.assertEqual(str(test_data), str(outfile.readlines()), 'wrong content in file')

    def test_rename_file(self):
        with FileIO(self.test_rename_from_file, 'w') as testfile:
            testfile.write(b'test content for file which will be renamed')
        fs = FileServicer()
        # test rename of not existing file
        rename_request = fmsg.RenameRequest()
        rename_request.old = self.test_rename_to_file
        rename_request.new = self.test_rename_from_file
        rename_response = fs.Rename(rename_request, DummyContext())
        self.assertEqual(rename_response.code, fmsg.ReturnStatus.StatusType.Value('OS_ERROR'), 'rename of not existing file returns a wrong result error: %d, expected: %d' % (rename_response.code, fmsg.ReturnStatus.StatusType.Value('OS_ERROR')))
        # test rename
        rename_request = fmsg.RenameRequest()
        rename_request.old = self.test_rename_from_file
        rename_request.new = self.test_rename_to_file
        rename_response = fs.Rename(rename_request, DummyContext())
        if not os.path.exists(self.test_rename_to_file):
            self.fail('After `rename` the target file does not exists')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFileServiceServicer)


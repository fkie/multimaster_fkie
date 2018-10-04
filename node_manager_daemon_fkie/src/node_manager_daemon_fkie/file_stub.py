# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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

from __future__ import print_function
import os
import settings
import node_manager_daemon_fkie.generated.file_pb2_grpc as fgrpc
import node_manager_daemon_fkie.generated.file_pb2 as fmsg
from .common import package_name
from .file_item import FileItem
import remote

OK = fmsg.ReturnStatus.StatusType.Value('OK')
ERROR = fmsg.ReturnStatus.StatusType.Value('ERROR')
IO_ERROR = fmsg.ReturnStatus.StatusType.Value('IO_ERROR')
OS_ERROR = fmsg.ReturnStatus.StatusType.Value('OS_ERROR')
CHANGED_FILE = fmsg.ReturnStatus.StatusType.Value('CHANGED_FILE')
REMOVED_FILE = fmsg.ReturnStatus.StatusType.Value('REMOVED_FILE')


class FileStub(object):

    FILE_CHUNK_SIZE = 1024

    def __init__(self, channel):
        self.fm_stub = fgrpc.FileServiceStub(channel)
        self._running = True

    def stop(self):
        self._running = False

    def list_path(self, path):
        result = []
#        response = self.fm_stub.ListPath(fmsg.ListPathRequest(path=path), timeout=settings.GRPC_TIMEOUT, metadata=[('authorization', 'user:robot')])
        response = self.fm_stub.ListPath(fmsg.ListPathRequest(path=path))
        if response.status.code == OK:
            for p in response.items:
                item = FileItem(p.path, p.type, p.size, p.mtime)
                result.append(item)
        elif response.status.code == OS_ERROR:
            raise OSError(response.status.error_code, response.status.error_msg, response.status.error_file)
        elif response.status.code in [IO_ERROR, CHANGED_FILE, REMOVED_FILE]:
            raise IOError(response.status.error_code, response.status.error_msg, response.status.error_file)
        elif response.status.code == ERROR:
            raise Exception("%s %s" % (response.status.error_msg, response.status.error_file))
        return result

    def list_packages(self, clear_ros_cache=False):
        result = {}
        response = self.fm_stub.ListPackages(fmsg.ListPackagesRequest(clear_ros_cache=clear_ros_cache))
        if response.status.code == OK:
            for p in response.items:
                result[p.path] = p.name
        elif response.status.code == ERROR:
            raise Exception(response.status.error_msg)
        return result

    def get_file_content(self, path):
        response_stream = self.fm_stub.GetFileContent(fmsg.ListPathRequest(path=path))
        file_size = None
        file_mtime = None
        file_content = ''
        for response in response_stream:
            if self._running:
                if response.status.code == OK:
                    if response.file.offset == 0:
                        file_size = response.file.size
                        file_mtime = response.file.mtime
                    file_content += response.file.data
                elif response.status.code == OS_ERROR:
                    raise OSError(response.status.error_code, response.status.error_msg, response.status.error_file)
                elif response.status.code in [IO_ERROR, CHANGED_FILE, REMOVED_FILE]:
                    raise IOError(response.status.error_code, response.status.error_msg, response.status.error_file)
                elif response.status.code == ERROR:
                    raise Exception("%s %s" % (response.status.error_msg, response.status.error_file))
            else:
                raise Exception("receiving for '%s' aborted! %d of %d transmitted." % (response.file.path, response.file.offset, response.file.size))
        return (file_size, file_mtime, file_content)

    def _gen_save_content_list(self, path, content, mtime, package=''):
        send_content = content
        while send_content:
            chunk = send_content
            # split into small parts on big files
            if len(chunk) > self.FILE_CHUNK_SIZE:
                chunk = send_content[0:self.FILE_CHUNK_SIZE]
                send_content = send_content[self.FILE_CHUNK_SIZE:]
            else:
                send_content = ''
            msg = fmsg.SaveFileContentRequest()
            msg.overwrite = mtime == 0
            msg.file.path = path
            msg.file.mtime = mtime  # something not zero to update a not existing file
            msg.file.size = len(content)
            msg.file.data = chunk
            msg.file.package = package
            yield msg

    def save_file_content(self, path, content, mtime, package=''):
        result = []
        response_stream = self.fm_stub.SaveFileContent(self._gen_save_content_list(path, content, mtime, package), timeout=settings.GRPC_TIMEOUT)
        for response in response_stream:
            if response.status.code == OK:
                result.append(response.ack)
            elif response.status.code == OS_ERROR:
                raise OSError(response.status.error_code, response.status.error_msg, response.status.error_file)
            elif response.status.code in [IO_ERROR, CHANGED_FILE, REMOVED_FILE]:
                raise IOError(response.status.error_code, response.status.error_msg, response.status.error_file)
            elif response.status.code == ERROR:
                raise Exception("%s %s" % (response.status.error_msg, response.status.error_file))
        return result

    def copy(self, path, dest_url, override=False):
        # get package from path
        pname, ppath = package_name(path)
        if pname is not None:
            prest = path.replace(ppath, '')
            with open(path, 'r') as outfile:
                mtime = os.path.getmtime(path)
                content = outfile.read()
                # get channel to the remote grpc server
                # TODO: get secure channel, if available
                channel = remote.get_insecure_channel(dest_url)
                if channel is not None:
                    fs = FileStub(channel)
                    # save file on remote server
                    return fs.save_file_content(prest, content, mtime, pname)

    def rename(self, old, new):
        response = self.fm_stub.Rename(fmsg.RenameRequest(old=old, new=new))
        if response.code == OK:
            pass
        elif response.code == OS_ERROR:
            raise OSError(response.error_code, response.error_msg, response.error_file)
        elif response.code in [IO_ERROR]:
            raise IOError(response.error_code, response.error_msg, response.error_file)
        elif response.code == ERROR:
            raise Exception("%s %s" % (response.error_msg, response.error_file))

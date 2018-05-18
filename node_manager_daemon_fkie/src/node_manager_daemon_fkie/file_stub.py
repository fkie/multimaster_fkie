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

import settings
import node_manager_daemon_fkie.generated.file_pb2_grpc as fgrpc
import node_manager_daemon_fkie.generated.file_pb2 as fmsg
from .file_item import FileItem

OK = fmsg.ReturnStatus.StatusType.Value('OK')
ERROR = fmsg.ReturnStatus.StatusType.Value('ERROR')
IO_ERROR = fmsg.ReturnStatus.StatusType.Value('IO_ERROR')
OS_ERROR = fmsg.ReturnStatus.StatusType.Value('OS_ERROR')
CHANGED_FILE = fmsg.ReturnStatus.StatusType.Value('CHANGED_FILE')
REMOVED_FILE = fmsg.ReturnStatus.StatusType.Value('REMOVED_FILE')


class FileStub(object):

    def __init__(self, channel):
        self.fm_stub = fgrpc.FileServiceStub(channel)

    def list_path(self, path):
        result = []
        response = self.fm_stub.ListPath(fmsg.ListPathRequest(path=path), timeout=settings.GRPC_TIMEOUT, metadata=[('authorization', 'user:robot')])
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

    def get_file_content(self, path):
        response_stream = self.fm_stub.GetFileContent(fmsg.ListPathRequest(path=path))
        file_size = None
        file_mtime = None
        file_content = ''
        for response in response_stream:
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
        return (file_size, file_mtime, file_content)

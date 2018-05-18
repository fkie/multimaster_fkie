# Copyright 2016 gRPC authors.
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

"""Generates protocol messages and gRPC stubs."""
import os

from grpc_tools import protoc

PROTOS = '../../protos/'
FILES = [
    'file.proto',
    'launch.proto'
]

#     'launch_manager_client.proto',
#     'msg_cancel.proto',
#     'msg_get_file_content.proto',
#     'msg_get_included_files.proto',
#     'msg_list_path.proto',
#     'msg_load_launch.proto',
#     'msg_loaded_files.proto',
#     'msg_register_client.proto',
#     'msg_status.proto'

gendir = 'generated'
if not os.path.exists(gendir):
    os.makedirs(gendir)
    init_file = '%s/__init__.py' % gendir
    open(init_file, 'a').close()
    for pf in FILES:
        if pf.startswith('msg_'):
            protoc.main(
                (
                    '',
                    '-I../../protos',
                    '--python_out=./%s/.' % gendir,
                    "%s%s" % (PROTOS, pf),
                )
            )
        else:
            protoc.main(
                (
                    '',
                    '-I../../protos',
                    '--python_out=./%s/.' % gendir,
                    '--grpc_python_out=./%s/.' % gendir,
                    "%s%s" % (PROTOS, pf),
                )
            )

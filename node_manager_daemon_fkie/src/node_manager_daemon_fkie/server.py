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

from concurrent import futures
import grpc
import rospy

from .file_servicer import FileServicer
from .launch_servicer import LaunchServicer

import node_manager_daemon_fkie.generated.file_pb2_grpc as fgrpc
import node_manager_daemon_fkie.generated.launch_pb2_grpc as lgrpc


class GrpcServer:

    def __init__(self):
        self.server = None

    def __del__(self):
        self.server.stop(3)

    def start(self, url='[::]:12311'):
        rospy.loginfo("Start grpc server on %s" % url)
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        # create credentials
        # read in key and certificate
#         with open('/home/tiderko/grpc_cert/server.key', 'rb') as f:
#             private_key = f.read()
#         with open('/home/tiderko/grpc_cert/server.crt', 'rb') as f:
#             certificate_chain = f.read()
#         # create server credentials
#         server_credentials = grpc.ssl_server_credentials(((private_key, certificate_chain,),))
#         print("port: ", self.server.add_secure_port(url, server_credentials))
        insecure_port = self.server.add_insecure_port(url)
        if insecure_port == 0:
            raise Exception("Can not add insecure channel to '%s'!" % url)
        fgrpc.add_FileServiceServicer_to_server(FileServicer(), self.server)
        lgrpc.add_LaunchServiceServicer_to_server(LaunchServicer(), self.server)
        self.server.start()

    def shutdown(self):
        self.server.stop(3)

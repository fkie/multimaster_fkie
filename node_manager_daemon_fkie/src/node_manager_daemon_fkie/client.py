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
from concurrent import futures
import grpc
import rospy

import exceptions
import node_manager_daemon_fkie.generated.file_pb2_grpc as fgrpc
import remote
from .file_client_servicer import FileClientServicer
import file_stub as fstub
import launch_stub as lstub


class GrpcClient:

    def __init__(self):
        self.url = None
        self.grpc_server = None
        self._channels = {}

    def start(self, url='[::]:12312'):
        self.url = url
        rospy.loginfo("Start grcp server on %s" % url)
        self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        fgrpc.add_FileClientServiceServicer_to_server(FileClientServicer(), self.grpc_server)
        self.grpc_server.add_insecure_port(url)
        self.grpc_server.start()

    def get_file_manager(self, url='localhost:12311'):
        channel = remote.get_insecure_channel(url)
        if channel is not None:
            return fstub.FileStub(channel)
        return None

    def get_launch_manager(self, url='localhost:12311'):
        channel = remote.get_insecure_channel(url)
        if channel is not None:
            return lstub.LaunchStub(channel)
        return None

    def test_list_path(self, path='', url='localhost:12311'):
        fm = self.get_file_manager(url)
        print("FM", fm)
        if fm is None:
            return
        try:
            rospy.loginfo("%s:" % path)
            result = fm.list_path(path)
            for p in result:
                if p.type in [1, 3]:
                    rospy.loginfo("  [%d] %s" % (p.type, p.path))
                else:
                    rospy.loginfo("  [%d] %s %d" % (p.type, p.path, p.size))
        except Exception as e:
            rospy.logwarn("LIST PATH: %s" % e)
            import traceback
            print(traceback.format_exc())

    def test_get_file_content(self, path='', url='localhost:12311'):
        fm = self.get_file_manager(url)
        if fm is None:
            return
        try:
            rospy.loginfo("get file content of %s:" % path)
            file_size, file_mtime, file_content = fm.get_file_content(path)
            rospy.loginfo("  [%d] [%d] %s" % (file_size, file_mtime, file_content))
        except Exception as e:
            rospy.logwarn("GET FILE CONTENT: %s" % e)
            import traceback
            print(traceback.format_exc())

    def _print_inc_file(self, indent, linenr, path, exists, inc_files):
        rospy.loginfo("%s %.4d\t%s %s" % (" " * indent, linenr, '+' if exists else '-', path))
        for ln, ph, ex, ifi in inc_files:
            self._print_inc_file(indent + 2, ln, ph, ex, ifi)

    def test_get_included_files(self, path='', url='localhost:12311'):
        lm = self.get_launch_manager(url)
        if lm is None:
            return
        try:
            rospy.loginfo("get_included_files for %s:" % path)
            inc_files = lm.get_included_path(path)
            for linenr, path, exists, file_list in inc_files:
                self._print_inc_file(2, linenr, path, exists, file_list)
        except Exception as e:
            rospy.logwarn("ERROR WHILE GET INCLUDE FILE: %s" % e)
            import traceback
            print(traceback.format_exc())

    def test_load_launch(self, package, launch, url='localhost:12311'):
        lm = self.get_launch_manager(url)
        if lm is None:
            return ''
        args = {}
        path = ''
        request_args = True
        nexttry = True
        ok = False
        launch_file = ''
        while nexttry:
            try:
                rospy.loginfo("load launch file %s [%s]:" % (launch, package))
                launch_file, _argv = lm.load_launch(package, launch, path=path, args=args, request_args=request_args)
                ok = True
            except exceptions.LaunchSelectionRequest as lsr:
                rospy.logwarn("%s\n  ...load the last one!" % lsr)
                path = lsr.choices[-1]
            except exceptions.ParamSelectionRequest as psr:
                rospy.logwarn("Params requered: %s\n  ...use defaults!" % ["%s:=%s" % (name, value) for name, value in psr.choices.items()])
                request_args = False
                args = psr.choices
            except exceptions.AlreadyOpenException as aoe:
                rospy.logwarn(aoe)
                nexttry = False
                ok = True
                launch_file = aoe.path
            except Exception as err:
                rospy.logwarn("ERROR WHILE LOAD LAUNCH: %s" % err)
                nexttry = False
        rospy.loginfo("  %s: %s" % ('OK' if ok else "ERR", launch_file))
        return launch_file

    def test_reload_launch(self, path, url='localhost:12311'):
        lm = self.get_launch_manager(url)
        if lm is None:
            return
        ok = False
        launch_file = path
        try:
            rospy.loginfo("reload launch file %s:" % path)
            launch_file = lm.reload_launch(path)
            ok = True
        except exceptions.ResourceNotFound as rnf:
            rospy.logwarn(rnf)
        except exceptions.RemoteException as re:
            rospy.logwarn("Unexpected error code %d; %s" % (re.code, re))
        except Exception as e:
            rospy.logwarn("ERROR WHILE RELOAD LAUNCH: %s" % e)
            import traceback
            print(traceback.format_exc())
        rospy.loginfo("  %s: %s" % ('OK' if ok else "ERR", launch_file))

    def test_get_nodes(self, url='localhost:12311'):
        lm = self.get_launch_manager(url)
        if lm is None:
            return
        try:
            rospy.loginfo("get nodes:")
            nodes_dict, descriptions = lm.get_nodes(True)
            for path, nodes in nodes_dict.items():
                rospy.loginfo("  %s [%d]:" % (path, len(nodes)))
                for node in nodes:
                    rospy.loginfo("    %s" % node)
            for d in descriptions:
                rospy.loginfo("Description: %s", str(d))
        except Exception as e:
            rospy.logwarn("ERROR WHILE GET NODES: %s" % e)
            import traceback
            print(traceback.format_exc())

    def test_start_node(self, name, url='localhost:12311'):
        lm = self.get_launch_manager(url)
        if lm is None:
            return
        try:
            rospy.loginfo("start node: %s" % name)
            lm.start_node(name)
            rospy.loginfo("  ok")
        except Exception as e:
            rospy.logwarn("ERROR WHILE GET NODES: %s" % e)
            import traceback
            print(traceback.format_exc())

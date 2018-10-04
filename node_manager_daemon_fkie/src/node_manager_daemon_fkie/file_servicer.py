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

import os
import rospy


import node_manager_daemon_fkie.generated.file_pb2_grpc as fms_grpc
import node_manager_daemon_fkie.generated.file_pb2 as fms
from .common import is_package, utf8

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

OK = fms.ReturnStatus.StatusType.Value('OK')
ERROR = fms.ReturnStatus.StatusType.Value('ERROR')
IO_ERROR = fms.ReturnStatus.StatusType.Value('IO_ERROR')
OS_ERROR = fms.ReturnStatus.StatusType.Value('OS_ERROR')
CHANGED_FILE = fms.ReturnStatus.StatusType.Value('CHANGED_FILE')
REMOVED_FILE = fms.ReturnStatus.StatusType.Value('REMOVED_FILE')
PATH_PACKAGE = fms.PathObj.PathType.Value('PACKAGE')
PATH_DIR = fms.PathObj.PathType.Value('DIR')
PATH_FILE = fms.PathObj.PathType.Value('FILE')
MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'


class FileServicer(fms_grpc.FileServiceServicer):

    FILE_CHUNK_SIZE = 1024

    def __init__(self):
        rospy.loginfo("Create file manger servicer")
        fms_grpc.FileServiceServicer.__init__(self)
        self.DIR_CACHE = {}
        self._peers = {}

#     def _terminated(self):
#         rospy.loginfo("terminated context")
#
#     def _register_callback(self, context):
#         if (context.peer() not in self._peers):
#             rospy.loginfo("Add callback to peer context @%s" % context.peer())
#             if context.add_callback(self._terminated):
#                 pass
#                 # self._peers[context.peer()] = context

    def GetFileContent(self, request, context):
        result = fms.GetFileContentReply()
        try:
            with open(request.path, 'r') as outfile:
                result.file.path = request.path
                a = os.path.getmtime(request.path)
                result.file.mtime = a
                result.file.size = os.path.getsize(request.path)
                result.file.offset = 0
                result.file.data = outfile.read(self.FILE_CHUNK_SIZE)
                yield result
                datalen = len(result.file.data)
                while len(result.file.data) == self.FILE_CHUNK_SIZE:
                    result = fms.GetFileContentReply()
                    result.file.offset = datalen
                    result.file.data = outfile.read(self.FILE_CHUNK_SIZE)
                    datalen += len(result.file.data)
                    yield result
        except IOError as ioe:
            result.status.code = IO_ERROR
            if ioe.errno:
                result.status.error_code = ioe.errno
            result.status.error_msg = utf8(ioe.strerror)
            result.status.error_file = utf8(ioe.filename)
            yield result

    def SaveFileContent(self, request_iterator, context):
        try:
            path = ''
            dest_size = 0
            curr_size = 0
            first = True
            file_org = None
            file_tmp = None
            count = 0
            for chunk in request_iterator:
                result = fms.SaveFileContentReply()
                if first:
                    if os.path.exists(chunk.file.path):
                        # checks for mtime
                        if chunk.overwrite or chunk.file.mtime == os.path.getmtime(chunk.file.path):
                            file_org = open(chunk.file.path, 'w')
                            file_tmp = open("%s.tmp" % chunk.file.path, 'w')
                            path = chunk.file.path
                            dest_size = chunk.file.size
                        else:
                            result.status.code = CHANGED_FILE
                            result.status.error_code = CHANGED_FILE
                            result.status.error_msg = utf8("file was changed in meantime")
                            result.status.error_file = utf8(chunk.file.path)
                    elif chunk.overwrite or chunk.file.mtime == 0:
                        # mtime == 0 stands for create a new file
                        try:
                            os.makedirs(os.path.dirname(chunk.file.path))
                        except OSError:
                            pass
                        file_org = open(chunk.file.path, 'w')
                        file_tmp = open("%s.tmp" % chunk.file.path, 'w')
                        path = chunk.file.path
                        dest_size = chunk.file.size
                    else:
                        result.status.code = REMOVED_FILE
                        result.status.error_code = REMOVED_FILE
                        result.status.error_msg = utf8("file was removed in meantime")
                        result.status.error_file = utf8(chunk.file.path)
                    first = False
                if result.status.code == 0:
                    if chunk.file.data:
                        file_tmp.write(chunk.file.data)
                    curr_size += len(chunk.file.data)
                    result.ack.path = path
                    result.ack.size = curr_size
                if dest_size == curr_size:
                    if file_tmp is not None:
                        file_tmp.close()
                    if file_org is not None:
                        file_org.close()
                    if file_tmp is not None and file_org is not None:
                        os.remove(path)
                        os.rename("%s.tmp" % path, path)
                        result.ack.mtime = os.path.getmtime(path)
                count += 1
                yield result
            if count == 0:
                result.status.code = ERROR
                result.status.error_msg = utf8("No iterating objects found")
                result.status.error_file = utf8(path)
                yield result
        except IOError as ioe:
            result.status.code = IO_ERROR
            if ioe.errno:
                result.status.error_code = ioe.errno
            result.status.error_msg = utf8(ioe.strerror)
            result.status.error_file = utf8(ioe.filename)
            yield result

    def Rename(self, request, context):
        result = fms.ReturnStatus()
        try:
            os.rename(request.old, request.new)
            result.code = OK
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = utf8(ose.strerror)
            result.error_file = utf8(request.old)
        except Exception as err:
            result.code = ERROR
            result.error_msg = utf8(err)
            result.error_file = utf8(request.old)
        return result

    def ListPath(self, request, context):
        result = fms.ListPathReply()
        result.path = request.path
        path_list = []
        if not request.path:
            # list ROS root items
            for p in os.getenv('ROS_PACKAGE_PATH').split(':'):
                path = os.path.normpath(p)
                path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(path), size=os.path.getsize(path), type=PATH_DIR, ))
        else:
            try:
                # list the path
                dirlist = os.listdir(request.path)
                for cfile in dirlist:
                    path = os.path.normpath('%s%s%s' % (request.path, os.path.sep, cfile))
                    if os.path.isfile(path):
                        path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(path), size=os.path.getsize(path), type=PATH_FILE))
                    elif path in self.DIR_CACHE:
                        path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(path), size=os.path.getsize(path), type=self.DIR_CACHE[path]))
                    elif os.path.isdir(path):
                        try:
                            fileList = os.listdir(path)
                            file_type = None
                            if is_package(fileList):
                                file_type = PATH_PACKAGE
                            else:
                                file_type = PATH_DIR
                            self.DIR_CACHE[path] = file_type
                            path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(path), size=os.path.getsize(path), type=file_type))
                        except Exception as _:
                            pass
            except OSError as ose:
                # import traceback
                # print(traceback.format_exc())
                result.status.code = OS_ERROR
                if ose.errno:
                    result.status.error_code = ose.errno
                result.status.error_msg = utf8(ose.strerror)
                result.status.error_file = utf8(ose.filename)
        result.items.extend(path_list)
        return result

    def _get_packages(self, path):
        result = {}
        if os.path.isdir(path):
            fileList = os.listdir(path)
            if MANIFEST_FILE in fileList:
                return {os.path.basename(path): path}
            if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
                try:
                    pkg = parse_package(path)
                    return {pkg.name: path}
                except Exception:
                    pass
                return {}
            for f in fileList:
                ret = self._get_packages(os.path.join(path, f))
                result = dict(ret.items() + result.items())
        return result

    def ListPackages(self, request, context):
        if request.clear_ros_cache:
            try:
                from roslaunch import substitution_args
                import rospkg
                substitution_args._rospack = rospkg.RosPack()
            except Exception as err:
                rospy.logwarn("Cannot reset package cache: %s" % utf8(err))
        result = fms.ListPackagesReply()
        try:
            # fill the input fields
            root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
            for p in root_paths:
                ret = self._get_packages(p)
                for name, path in ret.items():
                    package = fms.PackageObj(name=name, path=path)
                    result.items.extend([package])
            result.status.code = OK
        except Exception as err:
            result.status.code = ERROR
            result.status.error_msg = utf8(err)
        return result

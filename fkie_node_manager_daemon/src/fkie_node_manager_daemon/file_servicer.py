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


from io import FileIO
import os
import rospy
import shutil

import fkie_multimaster_msgs.grpc.file_pb2_grpc as fms_grpc
import fkie_multimaster_msgs.grpc.file_pb2 as fms
from . import file_item
from . import remote
from . import settings
from . import url as nmdurl
from .common import interpret_path, is_package, get_pkg_path, package_name, utf8

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
PATH_SYMLINK = fms.PathObj.PathType.Value('SYMLINK')
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
            with FileIO(request.path, 'r') as outfile:
                result.file.path = interpret_path(request.path)
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
        result = fms.SaveFileContentReply()
        try:
            path = ''
            dest_size = 0
            curr_size = 0
            first = True
            file_tmp = None
            count = 0
            for chunk in request_iterator:
                if chunk.file.package:
                    pkg_path = get_pkg_path(chunk.file.package)
                    if pkg_path:
                        path = os.path.join(pkg_path, chunk.file.path.lstrip(os.path.sep))
                else:
                    path = chunk.file.path
                result = fms.SaveFileContentReply()
                if first:
                    if os.path.exists(path):
                        # checks for mtime
                        if chunk.overwrite or chunk.file.mtime == os.path.getmtime(path):
                            file_tmp = FileIO("%s.tmp" % path, 'w')
                            dest_size = chunk.file.size
                        else:
                            result.status.code = CHANGED_FILE
                            result.status.error_code = file_item.EFILE_CHANGED
                            result.status.error_msg = utf8("file was changed in meantime")
                            result.status.error_file = utf8(path)
                    elif chunk.overwrite or chunk.file.mtime == 0:
                        # mtime == 0 stands for create a new file
                        try:
                            os.makedirs(os.path.dirname(path))
                        except OSError:
                            pass
                        file_tmp = FileIO("%s.tmp" % path, 'w')
                        dest_size = chunk.file.size
                    else:
                        result.status.code = REMOVED_FILE
                        result.status.error_code = file_item.EFILE_REMOVED
                        result.status.error_msg = utf8("file was removed in meantime")
                        result.status.error_file = utf8(path)
                    first = False
                if result.status.code == 0:
                    written = 0
                    if chunk.file.data:
                        written = file_tmp.write(chunk.file.data)
                        if written is None:
                            written = len(chunk.file.data)
                    if written != len(chunk.file.data):
                        result.status.code = ERROR
                        result.status.error_msg = utf8("error while write to tmp file")
                        result.status.error_file = utf8(path)
                        yield result
                    curr_size += written
                    result.ack.path = path
                    result.ack.size = curr_size
                if dest_size == curr_size:
                    if file_tmp is not None:
                        file_tmp.close()
                    if file_tmp is not None:
                        os.rename("%s.tmp" % path, path)
                        result.ack.mtime = os.path.getmtime(path)
                        result.status.code = OK
                count += 1
                yield result
                if result.status.code != OK:
                    break
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
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = utf8(ioe.strerror)
            result.error_file = utf8(ioe.filename)
        except Exception as err:
            result.code = ERROR
            result.error_msg = utf8(err)
            result.error_file = utf8(request.old)
        return result

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
            msg = fms.SaveFileContentRequest()
            msg.overwrite = mtime == 0
            msg.file.path = path
            msg.file.mtime = mtime  # something not zero to update a not existing file
            msg.file.size = len(content)
            msg.file.data = chunk
            msg.file.package = package
            yield msg

    def CopyFileTo(self, request, context):
        result = fms.ReturnStatus()
        try:
            path = request.path
            dest_uri, dest_path = nmdurl.split(request.uri)
            # get package from path
            pname, ppath = package_name(dest_path)
            if pname is not None:
                # we need relative package path without leading slash
                prest = dest_path.replace(ppath, '').lstrip(os.path.sep)
                with FileIO(path, 'r') as outfile:
                    mtime = 0.0 if request.overwrite else os.path.getmtime(path)
                    content = outfile.read()
                    # get channel to the remote grpc server
                    # TODO: get secure channel, if available
                    channel = remote.get_insecure_channel(dest_uri)
                    if channel is not None:
                        # save file on remote server
                        fs = fms_grpc.FileServiceStub(channel)
                        response_stream = fs.SaveFileContent(self._gen_save_content_list(prest, content, mtime, pname), timeout=settings.GRPC_TIMEOUT)
                        for response in response_stream:
                            if response.status.code == OK:
                                result.code = OK
                            else:
                                result.code = response.status.code
                                result.error_code = response.status.error_code
                                result.error_msg = response.status.error_msg
                                result.error_file = response.status.error_file
                                return result
                    else:
                        result.code = ERROR
                        result.error_msg = utf8("can not establish insecure channel to '%s'" % dest_uri)
                        result.error_file = utf8(request.path)
            else:
                result.code = ERROR
                result.error_msg = utf8("no package found! Only launch files from packages can be copied!")
                result.error_file = utf8(request.path)
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = utf8(ose.strerror)
            result.error_file = utf8(request.path)
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = utf8(ioe.strerror)
            result.error_file = utf8(ioe.filename)
        except Exception as err:
            result.code = ERROR
            result.error_msg = utf8(err)
            result.error_file = utf8(request.path)
        return result

    def ListPath(self, request, context):
        result = fms.ListPathReply()
        result.path = request.path
        path_list = []
        if not request.path:
            # list ROS root items
            for p in os.getenv('ROS_PACKAGE_PATH').split(':'):
                try:
                    path = os.path.normpath(p)
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
                result.update(ret)
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

    def ChangedFiles(self, request, context):
        result = fms.PathList()
        chnged_files = []
        for item in request.items:
            mtime = 0
            if os.path.exists(item.path):
                mtime = os.path.getmtime(item.path)
            if mtime != item.mtime:
                chnged_files.append(fms.PathObj(path=item.path, mtime=mtime))
        result.items.extend(chnged_files)
        return result

    def _get_binaries(self, path, binaries):
        if os.path.isdir(path):
            fileList = os.listdir(path)
            for f in fileList:
                if f and f[0] != '.' and f not in ['build'] and not f.endswith('.cfg') and not f.endswith('.so'):
                    self._get_binaries(os.path.join(path, f), binaries)
        elif os.path.isfile(path) and os.access(path, os.X_OK):
            binaries.append(fms.PathObj(path=path, mtime=os.path.getmtime(path)))

    def GetPackageBinaries(self, request, context):
        result = fms.PathList()
        binaries = []
        try:
            path = get_pkg_path(request.name)
            self._get_binaries(path, binaries)
            result.items.extend(binaries)
            # find binaries in catkin workspace
            from catkin.find_in_workspaces import find_in_workspaces as catkin_find
            search_paths = catkin_find(search_dirs=['libexec', 'share'], project=request.name, first_matching_workspace_only=True)
            for p in search_paths:
                self._get_binaries(p, binaries)
        except Exception:
            import traceback
            print(traceback.format_exc())
            pass
        return result

    def Delete(self, request, context):
        result = fms.ReturnStatus()
        try:
            if self._is_in_ros_root(request.path):
                if os.path.isfile(request.path):
                    os.remove(request.path)
                    result.code = OK
                elif os.path.isdir(request.path):
                    if not self._contains_packages(request.path):
                        shutil.rmtree(request.path)
                        result.code = OK
                    else:
                        result.code = ERROR
                        result.error_msg = utf8("path contains packages")
                        result.error_file = utf8(request.path)
            else:
                result.code = ERROR
                result.error_msg = utf8("path not in ROS_PACKAGE_PATH")
                result.error_file = utf8(request.path)
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = utf8(ose.strerror)
            result.error_file = utf8(request.path)
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = utf8(ioe.strerror)
            result.error_file = utf8(ioe.filename)
        except Exception as err:
            result.code = ERROR
            result.error_msg = utf8(err)
            result.error_file = utf8(request.path)
        return result

    def _is_in_ros_root(self, path):
        # list ROS root items
        for p in os.getenv('ROS_PACKAGE_PATH').split(':'):
            root = os.path.abspath(p)
            if path.startswith(root):
                rest = path.replace(root, '').strip(os.path.sep)
                if rest:
                    return True
        return False

    def _contains_packages(self, path):
        # list ROS root items
        path = os.path.abspath(path)
        for d, dirs, files in os.walk(path, topdown=True):
            if PACKAGE_FILE in files or MANIFEST_FILE in files:
                del dirs[:]
                return True
            elif 'rospack_nosubdirs' in files:
                del dirs[:]
                continue #leaf
            # small optimization
            elif '.svn' in dirs:
                dirs.remove('.svn')
            elif '.git' in dirs:
                dirs.remove('.git')
        return False

    def New(self, request, context):
        result = fms.ReturnStatus()
        try:
            if request.type == PATH_FILE:
                new_file = FileIO(request.path, "w+")
                new_file.close()
            elif request.type == PATH_DIR:
                os.mkdir(request.path)
            elif request.type == PATH_SYMLINK:
                raise Exception("creation of symlinks not supported")
            elif request.type == PATH_PACKAGE:
                raise Exception("creation of packages not supported")
            result.code = OK
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = utf8(ose.strerror)
            result.error_file = utf8(request.path)
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = utf8(ioe.strerror)
            result.error_file = utf8(ioe.filename)
        except Exception as err:
            result.code = ERROR
            result.error_msg = utf8(err)
            result.error_file = utf8(request.path)
        return result

# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
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


from io import FileIO
import os
import shutil

import fkie_node_manager_daemon as nmd
from fkie_node_manager_daemon.common import get_packages
import fkie_node_manager_daemon.grpc_proto.file_pb2_grpc as fms_grpc
import fkie_node_manager_daemon.grpc_proto.file_pb2 as fms
from . import file_item
from . import remote
from . import settings
from . import url as nmdurl
from .common import interpret_path, is_package, get_pkg_path, package_name

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
        nmd.rosnode.get_logger().info("Create file manger servicer")
        fms_grpc.FileServiceServicer.__init__(self)
        self.DIR_CACHE = {}
        self._peers = {}

#     def _terminated(self):
#         nm.rosnode.get_logger().info("terminated context")
#
#     def _register_callback(self, context):
#         if (context.peer() not in self._peers):
#             nm.rosnode.get_logger().info("Add callback to peer context @%s" % context.peer())
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
            result.status.error_msg = ioe.strerror
            result.status.error_file = ioe.filename
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
                        path = os.path.join(
                            pkg_path, chunk.file.path.lstrip(os.path.sep))
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
                            result.status.error_msg = "file was changed in meantime"
                            result.status.error_file = path
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
                        result.status.error_msg = "file was removed in meantime"
                        result.status.error_file = path
                    first = False
                if result.status.code == 0:
                    written = 0
                    if chunk.file.data:
                        written = file_tmp.write(chunk.file.data)
                        if written is None:
                            written = len(chunk.file.data)
                    if written != len(chunk.file.data):
                        result.status.code = ERROR
                        result.status.error_msg = "error while write to tmp file"
                        result.status.error_file = path
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
                result.status.error_msg = "No iterating objects found"
                result.status.error_file = path
                yield result
        except IOError as ioe:
            result.status.code = IO_ERROR
            if ioe.errno:
                result.status.error_code = ioe.errno
            result.status.error_msg = ioe.strerror
            result.status.error_file = ioe.filename
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
            result.error_msg = ose.strerror
            result.error_file = request.old
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = ioe.strerror
            result.error_file = ioe.filename
        except Exception as err:
            result.code = ERROR
            result.error_msg = err
            result.error_file = request.old
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
            path_src = request.path
            dest_uri, path_dst = nmdurl.split(request.uri)
            pkg_name = ''
            if not path_dst:
                # we have no destination path, determine package name from source path
                pkg_name, ppath = package_name(path_src)
                if pkg_name:
                    # we need relative package path without leading slash
                    path_dst = request.path.replace(
                        ppath, '').lstrip(os.path.sep)
                else:
                    result.code = ERROR
                    result.error_msg = 'no package found! If no destination path is given, only launch files from packages can be copied!'
                    result.error_file = request.path
            if path_dst:
                nmd.rosnode.get_logger().debug("Copy '%s' to '%s' [package: '%s'] @ %s (overwrite: %s)" % (
                    path_src, path_dst, pkg_name, dest_uri, request.overwrite))
                with FileIO(path_src, 'r') as outfile:
                    mtime = 0.0 if request.overwrite else os.path.getmtime(
                        path_src)
                    content = outfile.read()
                    # get channel to the remote grpc server
                    channel = remote.open_channel(
                        dest_uri, rosnode=nmd.rosnode)
                    if channel is not None:
                        # save file on remote server
                        fs = fms_grpc.FileServiceStub(channel)
                        response_stream = fs.SaveFileContent(self._gen_save_content_list(
                            path_dst, content, mtime, pkg_name), timeout=settings.GRPC_TIMEOUT)
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
                        result.error_msg = "can not establish insecure channel to '%s'" % dest_uri
                        result.error_file = request.path
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = ose.strerror
            result.error_file = request.path
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = ioe.strerror
            result.error_file = ioe.filename
        except Exception as err:
            result.code = ERROR
            result.error_msg = err
            result.error_file = request.path
        return result

    def ListPath(self, request, context):
        result = fms.ListPathReply()
        result.path = request.path
        path_list = []
        if not request.path:
            # list ROS root items
            count_share_paths = {}
            added_paths = []
            packages = get_packages(None)
            for pkgname, p in packages.items():
                p_share = os.path.normpath(os.path.join(p, 'share'))
                if p_share in count_share_paths:
                    count_share_paths[p_share] += 1
                else:
                    count_share_paths[p_share] = 1
            for pkgname, p in packages.items():
                p_share = os.path.normpath(os.path.join(p, 'share'))
                try:
                    if count_share_paths[p_share] == 1:
                        # on single occurrence we take the share path
                        path = os.path.join(p, 'share', pkgname)
                        if not os.path.exists(path):
                            path = p
                        fileList = os.listdir(path)
                        file_type = None
                        if is_package(fileList):
                            file_type = PATH_PACKAGE
                    else:
                        # if we have more than one occurrence, we take the root.
                        path = p
                        file_type = PATH_DIR
                    # add path if it is not already in
                    if path not in added_paths:
                        self.DIR_CACHE[path] = file_type
                        added_paths.append(path)
                        path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(
                            path), size=os.path.getsize(path), type=file_type))
                except Exception as _:
                    pass
        else:
            try:
                # list the path
                dirlist = os.listdir(request.path)
                for cfile in dirlist:
                    path = os.path.normpath('%s%s%s' % (
                        request.path, os.path.sep, cfile))
                    if os.path.isfile(path):
                        path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(
                            path), size=os.path.getsize(path), type=PATH_FILE))
                    elif path in self.DIR_CACHE:
                        path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(
                            path), size=os.path.getsize(path), type=self.DIR_CACHE[path]))
                    elif os.path.isdir(path):
                        try:
                            fileList = os.listdir(path)
                            file_type = None
                            if is_package(fileList):
                                file_type = PATH_PACKAGE
                            else:
                                file_type = PATH_DIR
                            self.DIR_CACHE[path] = file_type
                            path_list.append(fms.PathObj(path=path, mtime=os.path.getmtime(
                                path), size=os.path.getsize(path), type=file_type))
                        except Exception as _:
                            pass
            except OSError as ose:
                # import traceback
                # print(traceback.format_exc())
                result.status.code = OS_ERROR
                if ose.errno:
                    result.status.error_code = ose.errno
                result.status.error_msg = ose.strerror
                result.status.error_file = ose.filename
        result.items.extend(path_list)
        return result

    def ListPackages(self, request, context):
        # TODO: does clear option exists in ROS2?
        # if request.clear_ros_cache:
        #     try:
        #         from roslaunch import substitution_args
        #         import rospkg
        #         substitution_args._rospack = rospkg.RosPack()
        #     except Exception as err:
        #         nmd.rosnode.get_logger().warn("Cannot reset package cache: %s" % err)
        result = fms.ListPackagesReply()
        try:
            ret = get_packages(None)
            for name, path in ret.items():
                package = fms.PackageObj(
                    name=name, path=os.path.join(path, 'share', name))
                result.items.extend([package])
            result.status.code = OK
        except Exception as err:
            result.status.code = ERROR
            result.status.error_msg = err
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
            binaries.append(fms.PathObj(
                path=path, mtime=os.path.getmtime(path)))

    def GetPackageBinaries(self, request, context):
        result = fms.PathList()
        binaries = []
        try:
            path = get_pkg_path(request.name)
            self._get_binaries(path, binaries)
            result.items.extend(binaries)
            # find binaries in catkin workspace
            from catkin.find_in_workspaces import find_in_workspaces as catkin_find
            search_paths = catkin_find(search_dirs=[
                                       'libexec', 'share'], project=request.name, first_matching_workspace_only=True)
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
                nmd.rosnode.get_logger().debug("Delete '%s'" % (request.path))
                if os.path.isfile(request.path):
                    os.remove(request.path)
                    result.code = OK
                elif os.path.isdir(request.path):
                    if not self._contains_packages(request.path):
                        shutil.rmtree(request.path)
                        result.code = OK
                    else:
                        result.code = ERROR
                        result.error_msg = "path contains packages"
                        result.error_file = request.path
            else:
                result.code = ERROR
                result.error_msg = "path not in ROS_PACKAGE_PATH"
                result.error_file = request.path
        except OSError as ose:
            result.code = OS_ERROR
            if ose.errno:
                result.error_code = ose.errno
            result.error_msg = ose.strerror
            result.error_file = request.path
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = ioe.strerror
            result.error_file = ioe.filename
        except Exception as err:
            result.code = ERROR
            result.error_msg = err
            result.error_file = request.path
        return result

    def _is_in_ros_root(self, path):
        # list ROS root items
        for _pkgname, root in get_packages(None).items():
            if path.startswith(root):
                # do not delete the roor path itself
                if len(path) - 2 > len(root):
                    return True
        return False

    def _contains_packages(self, path):
        # list ROS root items
        path = os.path.abspath(path)
        for _d, dirs, files in os.walk(path, topdown=True):
            if PACKAGE_FILE in files or MANIFEST_FILE in files:
                del dirs[:]
                return True
            elif 'rospack_nosubdirs' in files:
                del dirs[:]
                continue  # leaf
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
            result.error_msg = ose.strerror
            result.error_file = request.path
        except IOError as ioe:
            result.code = IO_ERROR
            if ioe.errno:
                result.error_code = ioe.errno
            result.error_msg = ioe.strerror
            result.error_file = ioe.filename
        except Exception as err:
            result.code = ERROR
            result.error_msg = err
            result.error_file = request.path
        return result

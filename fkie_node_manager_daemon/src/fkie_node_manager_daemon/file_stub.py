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



from . import file_item
from . import settings
import fkie_multimaster_msgs.grpc.file_pb2_grpc as fgrpc
import fkie_multimaster_msgs.grpc.file_pb2 as fmsg
from .common import utf8

OK = fmsg.ReturnStatus.StatusType.Value('OK')
ERROR = fmsg.ReturnStatus.StatusType.Value('ERROR')
IO_ERROR = fmsg.ReturnStatus.StatusType.Value('IO_ERROR')
OS_ERROR = fmsg.ReturnStatus.StatusType.Value('OS_ERROR')
CHANGED_FILE = fmsg.ReturnStatus.StatusType.Value('CHANGED_FILE')
REMOVED_FILE = fmsg.ReturnStatus.StatusType.Value('REMOVED_FILE')


class FileStub(object):

    FILE_CHUNK_SIZE = 1024
    ''':ivar FileStub.FILE_CHUNK_SIZE: while save on remote server the file will be split into chunks of this size.'''

    def __init__(self, channel):
        self.fm_stub = fgrpc.FileServiceStub(channel)
        self._running = True

    def stop(self):
        '''
        Set not running flag and all communication loops should cancel.
        '''
        self._running = False

    def list_path(self, path):
        '''
        Request content of path from remote gRPC-server.

        :param str path: a directory
        :retrun: a list of directories and files represented by `FileItem`.
        :rtype: list(:class:`file_item.FileItem`)
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        result = []
#        response = self.fm_stub.ListPath(fmsg.ListPathRequest(path=path), timeout=settings.GRPC_TIMEOUT, metadata=[('authorization', 'user:robot')])
        response = self.fm_stub.ListPath(fmsg.ListPathRequest(path=path))
        if response.status.code == OK:
            for p in response.items:
                item = file_item.FileItem(p.path, p.type, p.size, p.mtime)
                result.append(item)
        elif response.status.code == OS_ERROR:
            raise OSError(response.status.error_code, response.status.error_msg, response.status.error_file)
        elif response.status.code in [IO_ERROR, CHANGED_FILE, REMOVED_FILE]:
            raise IOError(response.status.error_code, response.status.error_msg, response.status.error_file)
        elif response.status.code == ERROR:
            raise Exception("%s %s" % (response.status.error_msg, response.status.error_file))
        return result

    def list_packages(self, clear_ros_cache=False):
        '''
        Requests all known packages from gRPC-server.

        :param bool clear_ros_cache: clear ROS cache before list the packages.
        :return: a dictionary of path to package and package name.
        :rype: dict(str: str)
        :raise Exception:
        '''
        result = {}
        response = self.fm_stub.ListPackages(fmsg.ListPackagesRequest(clear_ros_cache=clear_ros_cache))
        if response.status.code == OK:
            for p in response.items:
                result[p.path] = p.name
        elif response.status.code == ERROR:
            raise Exception(response.status.error_msg)
        return result

    def get_file_content(self, path):
        '''
        Requests the content of the file.

        :path str path: the path to the file.
        :retrun: the size, last modification time and the content of the file.
        :rtype: tuple(int, float, str)
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        response_stream = self.fm_stub.GetFileContent(fmsg.ListPathRequest(path=path))
        file_size = None
        file_mtime = None
        file_content = b''
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
        minone = True
        try:
            send_content = content  # .encode('utf-8')
            while send_content or minone:
                minone = False
                chunk = send_content
                # split into small parts on big files
                if len(chunk) > self.FILE_CHUNK_SIZE:
                    chunk = send_content[0:self.FILE_CHUNK_SIZE]
                    send_content = send_content[self.FILE_CHUNK_SIZE:]
                else:
                    send_content = b''
                msg = fmsg.SaveFileContentRequest()
                msg.overwrite = mtime == 0
                msg.file.path = path
                msg.file.mtime = mtime  # something not zero to update a not existing file
                msg.file.size = len(content)
                msg.file.data = chunk
                msg.file.package = package
                yield msg
        except Exception:
            import traceback
            print(traceback.format_exc())

    def save_file_content(self, path, content, mtime, package=''):
        '''
        Save the file to gRPC-server.

        :param str path: path where to save the content.
        :param str content: the content of the file.
        :param float mtime: last modification time. Set to zero to replace or create a new file without exception.
        :raise OSError:
        :raise IOError: if file was changed or removed in the meantime and
            `mtime` parameter is not zero. Sets the `errno` of the exception
            to :const:`fkie_node_manager_daemon.file_item.EFILE_CHANGED`, :const:`fkie_node_manager_daemon.file_item.EFILE_REMOVED`.
        :raise Exception:
        '''
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

    def copy(self, path, dest_uri, overwrite=True):
        '''
        Copy given file to an other remote gRPC-server.

        :param str path: path of the file to copy.
        :param str dest_uri: URI of destination gRPC-server. The URI should not contain any scheme. (example: hostname:port)
        :param bool overwrite: True to replace or create a file without exception.
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        result = self.fm_stub.CopyFileTo(fmsg.CopyToRequest(path=path, uri=dest_uri, overwrite=overwrite), timeout=settings.GRPC_TIMEOUT)
        if result.code == OK:
            pass
        elif result.code == OS_ERROR:
            raise OSError(result.error_code, result.error_msg, result.error_file)
        elif result.code in [IO_ERROR, CHANGED_FILE, REMOVED_FILE]:
            raise IOError(result.error_code, result.error_msg, result.error_file)
        elif result.code == ERROR:
            raise Exception("%s file: %s" % (result.error_msg, result.error_file))

    def rename(self, old, new):
        '''
        Rename path.

        :param str old: existing path
        :param str new: new path
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        response = self.fm_stub.Rename(fmsg.RenameRequest(old=old, new=new), timeout=settings.GRPC_TIMEOUT)
        if response.code == OK:
            pass
        elif response.code == OS_ERROR:
            raise OSError(response.error_code, response.error_msg, response.error_file)
        elif response.code in [IO_ERROR]:
            raise IOError(response.error_code, response.error_msg, response.error_file)
        elif response.code == ERROR:
            raise Exception("%s, path: %s" % (response.error_msg, response.error_file))

    def changed_files(self, files):
        '''
        Request to test files for last modification time.

        :param files: dictionary with files and their last known modification time.
            See results of :meth:`fkie_node_manager_daemon.launch_stub.LaunchStub.get_mtimes`,
            :meth:`fkie_node_manager_daemon.launch_stub.LaunchStub.load_launch`.
        :type files: dict(str: float)
        '''
        request = fmsg.PathList()
        pathlist = []
        for path, mtime in files.items():
            pathlist.append(fmsg.PathObj(path=path, mtime=mtime))
        request.items.extend(pathlist)
        response = self.fm_stub.ChangedFiles(request, timeout=settings.GRPC_TIMEOUT)
        return response.items

    def get_package_binaries(self, pkgname):
        '''
        Request for given package a list with all known binaries.

        :param str pkgname: the name of a package
        :return: the list with file items
        :rtype: :class:`file_pb2.PathObj`
        '''
        request = fmsg.PackageObj(name=pkgname)
        response = self.fm_stub.GetPackageBinaries(request, timeout=settings.GRPC_TIMEOUT)
        return response.items

    def delete(self, path):
        '''
        Delete path.

        :param str path: existing path
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        response = self.fm_stub.Delete(fmsg.PathObj(path=path), timeout=settings.GRPC_TIMEOUT)
        if response.code == OK:
            pass
        elif response.code == OS_ERROR:
            raise OSError(response.error_code, response.error_msg, response.error_file)
        elif response.code in [IO_ERROR]:
            raise IOError(response.error_code, response.error_msg, response.error_file)
        elif response.code == ERROR:
            raise Exception("%s, path: %s" % (response.error_msg, response.error_file))

    def new(self, path, path_type):
        '''
        Create a new path.

        :param str path: existing path
        :param int path_type: path type, 0=file, 1=directory
        :raise OSError:
        :raise IOError:
        :raise Exception:
        '''
        response = self.fm_stub.New(fmsg.PathObj(path=path, type=path_type), timeout=settings.GRPC_TIMEOUT)
        if response.code == OK:
            pass
        elif response.code == OS_ERROR:
            raise OSError(response.error_code, response.error_msg, response.error_file)
        elif response.code in [IO_ERROR]:
            raise IOError(response.error_code, response.error_msg, response.error_file)
        elif response.code == ERROR:
            raise Exception("%s, path: %s" % (response.error_msg, response.error_file))

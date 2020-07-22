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
from python_qt_binding.QtCore import Signal

import fkie_node_manager_daemon.file_stub as fstub
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon.common import utf8

from .channel_interface import ChannelInterface


class FileChannel(ChannelInterface):

    listed_path = Signal(str, str, list)
    '''
    :ivar str,str,list listed_path: listed_path is a signal, which is emitted, if path is listed successful {url, path, list with paths}.
    '''
    changed_file = Signal(str, float)
    '''
    :ivar str,float changed_file: this signal is emitted after test request for file changes {grpc_path, mtime}.
    '''
    packages = Signal(str, dict)
    '''
    :ivar str,dict packages: this signal is emitted on new list with packages  {grpc_url, dict(grpc_path, name)}.
    '''
    packages_available = Signal(str)
    '''
    :ivar str packages_available: this signal is emitted on new list with packages  {grpc_url}.
    '''
    file_content = Signal(str, int, float, str)
    '''
    :ivar str,int,int,str file_content: gets the content of the file  {grpc_url, size, mtime, content}.
    '''

    def __init__(self):
        ChannelInterface.__init__(self)
        self._cache_file_content = {}
        self._cache_packages = {}
        self._cache_path = {}

    def clear_cache(self, grpc_path=''):
        if grpc_path:
            try:
                del self._cache_file_content[grpc_path]
            except Exception:
                pass
            try:
                del self._cache_path[grpc_path]
            except Exception:
                pass
            try:
                del self._cache_packages[grpc_path]
            except Exception:
                pass
        else:
            self._cache_file_content.clear()
            self._cache_packages.clear()
            self._cache_path.clear()

    def get_file_manager(self, uri='localhost:12321'):
        channel = self.get_insecure_channel(uri)
        return fstub.FileStub(channel), channel

    def package_name(self, grpc_path):
        uri, _path = nmdurl.split(grpc_path, with_scheme=True)
        path = grpc_path
        try:
            pl = self._cache_packages[uri]
            while path and path != os.path.sep:
                if path in pl:
                    return pl[path], path
                path = os.path.dirname(path).rstrip(os.path.sep)
        except Exception:
            pass
        return None, None

    def get_packages(self, url=''):
        if url:
            grpc_url = nmdurl.nmduri(url)
            if grpc_url in self._cache_packages:
                return self._cache_packages[grpc_url]
            return {}
        return self._cache_packages

    def clear_package_cache(self, url):
        if url:
            grpc_url = nmdurl.nmduri(url)
            try:
                del self._cache_packages[grpc_url]
                rospy.logdebug("cache for packages from '%s' removed", grpc_url)
            except KeyError:
                pass

    def list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        self._threads.start_thread("lpt_%s" % grpc_path, target=self._list_path_threaded, args=(grpc_path, clear_cache))

    def _list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] list path: %s, '%s'" % (uri, path))
        fm, channel = self.get_file_manager(uri)
        result = None
        try:
            if not clear_cache:
                result = self._cache_path[grpc_path]
            else:
                self._cache_path['']  # only to cause an exception
        except KeyError:
            try:
                result = fm.list_path(path)
                if uri not in self._cache_packages:
                    self.list_packages_threaded(grpc_path, clear_cache)
#                    self.get_loaded_files_threaded(grpc_path)
            except Exception as e:
                self.error.emit("list_path", "grpc://%s" % uri, path, e)
                # rospy.logwarn("LIST PATH: %s" % e)
        if result is not None:
            self._cache_path[grpc_path] = result
            self.listed_path.emit("grpc://%s" % uri, path, result)
        self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("lpt_%s" % grpc_path)

    def check_for_changed_files_threaded(self, grpc_path_dict):
        dests = {}
        for grpc_path, mtime in grpc_path_dict.items():
            uri, path = nmdurl.split(grpc_path, with_scheme=True)
            if uri not in dests:
                dests[uri] = {}
            dests[uri][path] = mtime
        for uri, paths in dests.items():
            self._threads.start_thread("cft_%s" % uri, target=self._check_for_changed_files_threaded, args=(uri, paths))

    def _check_for_changed_files_threaded(self, grpc_url, path_dict):
        rospy.logdebug("[thread] check_for_changed_files_threaded: with %d files on %s" % (len(path_dict), grpc_url))
        uri, _path = nmdurl.split(grpc_url, with_scheme=False)
        fm, channel = self.get_file_manager(uri)
        try:
            response = fm.changed_files(path_dict)
            for item in response:
                self.changed_file.emit(nmdurl.join(grpc_url, item.path), item.mtime)
        except Exception as e:
            self.error.emit("changed_files", "grpc://%s" % uri, "", e)
            # rospy.logwarn("check_for_changed_files_threaded: %s" % e)
        url, _path = nmdurl.split(grpc_url, with_scheme=True)
        self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("cft_%s" % url)

    def list_packages_threaded(self, grpc_url_or_path='grpc://localhost:12321', clear_ros_cache=False):
        self._threads.start_thread("gmt_%s_%d" % (grpc_url_or_path, clear_ros_cache), target=self._list_packages, args=(grpc_url_or_path, clear_ros_cache))

    def _list_packages(self, grpc_url_or_path='grpc://localhost:12321', clear_ros_cache=False):
        uri, path = nmdurl.split(grpc_url_or_path)
        grpc_url = "grpc://%s" % uri
        result = {}
        try:
            if not clear_ros_cache:
                result = self._cache_packages[grpc_url]
            else:
                self._cache_packages['']  # only to cause an exception
        except KeyError:
            rospy.logdebug("[thread] get packages %s" % grpc_url)
            fm, channel = self.get_file_manager(uri)
            try:
                result = fm.list_packages(clear_ros_cache)
                fixed_result = {nmdurl.join(grpc_url, path): name for path, name in result.items()}
                self._cache_packages[grpc_url] = fixed_result
                self.packages.emit(grpc_url, fixed_result)
                self.packages_available.emit(grpc_url)
            except Exception as err:
                self.error.emit("_list_packages", "grpc://%s" % uri, path, err)
            finally:
                self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("gmt_%s_%d" % (grpc_url_or_path, clear_ros_cache))

    def get_file_content_threaded(self, grpc_path='grpc://localhost:12321', force=False):
        self._threads.start_thread("gfc_%s_%d" % (grpc_path, force), target=self.get_file_content, args=(grpc_path, force))

    def get_file_content(self, grpc_path='grpc://localhost:12321', force=False):
        file_size, file_mtime, file_content = (0, 0, '')
        try:
            if force:
                del self._cache_file_content[grpc_path]
            file_size, file_mtime, file_content = self._cache_file_content[grpc_path]
        except KeyError:
            rospy.logdebug("get file content for %s:" % grpc_path)
            uri, path = nmdurl.split(grpc_path)
            fm, channel = self.get_file_manager(uri)
            try:
                file_size, file_mtime, file_content = fm.get_file_content(path)
                file_content = utf8(file_content)
                self._cache_file_content[grpc_path] = (file_size, file_mtime, file_content)
            except Exception as e:
                self.error.emit("get_file_content", "grpc://%s" % uri, grpc_path, e)
                raise e
            finally:
                self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("gfc_%s_%d" % (grpc_path, force))
        self.file_content.emit(grpc_path, file_size, file_mtime, file_content)
        return file_size, file_mtime, file_content

    def save_file(self, grpc_path, content, mtime):
        rospy.logdebug("save_file_content: %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        fm, channel = self.get_file_manager(uri)
        result = fm.save_file_content(path, content, mtime)
        for ack in result:
            if ack.path == path and ack.mtime != 0:
                self.clear_cache(grpc_path)
                self.close_channel(channel, uri)
                return ack.mtime
        self.close_channel(channel, uri)
        return 0

    def rename(self, grpc_path_old='grpc://localhost:12321', grpc_path_new='grpc://localhost:12321'):
        uri, old = nmdurl.split(grpc_path_old)
        _, new = nmdurl.split(grpc_path_new)
        rospy.logdebug("rename path on %s" % uri)
        fm, channel = self.get_file_manager(uri)
        result = fm.rename(old, new)
        self.close_channel(channel, uri)
        return result

    def copy(self, grpc_path='grpc://localhost:12321', grpc_dest='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("copy '%s' to '%s'" % (grpc_path, grpc_dest))
        fm, channel = self.get_file_manager(uri)
        fm.copy(path, grpc_dest)
        self.close_channel(channel, uri)

    def get_package_binaries(self, pkgname, grpc_url='grpc://localhost:12321'):
        uri, _path = nmdurl.split(grpc_url)
        rospy.logdebug("get_package_binaries for '%s' from '%s'" % (pkgname, uri))
        fm, channel = self.get_file_manager(uri)
        response = fm.get_package_binaries(pkgname)
        url, _ = nmdurl.split(grpc_url, with_scheme=True)
        result = {}
        for item in response:
            result[nmdurl.join(url, item.path)] = item.mtime
        self.close_channel(channel, uri)
        return result

    def delete(self, grpc_path='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("delete '%s' @ %s" % (path, uri))
        fm, channel = self.get_file_manager(uri)
        result = fm.delete(path)
        self.close_channel(channel, uri)
        return result

    def new(self, grpc_path='grpc://localhost:12321', path_type=0):
        '''
        :param int path_type: 0=file, 1=dir
        '''
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("create new '%s' @ %s" % (path, uri))
        fm, channel = self.get_file_manager(uri)
        result = fm.new(path, path_type)
        self.close_channel(channel, uri)
        return result

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
import os
import grpc
from python_qt_binding.QtCore import QObject, Signal
import rospy
import threading

# import node_manager_fkie as nm

import node_manager_daemon_fkie.exceptions as exceptions
#import node_manager_daemon_fkie.generated.file_pb2_grpc as fgrpc
import node_manager_daemon_fkie.remote as remote
#from node_manager_daemon_fkie.file_client_servicer import FileClientServicer
import node_manager_daemon_fkie.file_stub as fstub
import node_manager_daemon_fkie.launch_stub as lstub
from node_manager_daemon_fkie.url import get_nmd_url, grpc_join, grpc_split_url, grpc_create_url
from .common import utf8


class LaunchArgsSelectionRequest(Exception):
    ''' Request needed to set the args of a launchfile from another thread.
    :param dict args: a dictionary with args and values
    :param str error: an error description
    '''

    def __init__(self, launchfile, args, error):
        Exception.__init__(self)
        self.launchfile = launchfile
        self.args_dict = args
        self.error = error

    def __str__(self):
        return "LaunchArgsSelectionRequest for " + utf8(self.args_dict) + "::" + repr(self.error)


class NmdClient(QObject):
    '''
    A thread to get data from node manager daemon and
    publish there be sending a QT signal.
    '''
    listed_path = Signal(str, str, list)
    '''
    :ivar: listed_path is a signal, which is emitted, if path is listed successful {url, path, list with paths}.
    '''
    error = Signal(str, str, str, Exception)
    '''
    :ivar: error is a signal, which is emitted on errors {method, url, path, Exception}.
    '''
    changed_file = Signal(str, float)
    '''
    :ivar: this signal is emitted after test request for file changes {grpc_path, mtime}.
    '''
    packages = Signal(str, dict)
    '''
    :ivar: this signal is emitted on new list with packages  {grpc_url, dict(grpc_path, name)}.
    '''
    packages_available = Signal(str)
    '''
    :ivar: this signal is emitted on new list with packages  {grpc_url}.
    '''
    mtimes = Signal(str, float, dict)
    '''
    :ivar: this signal is emitted on new mtimes for requested files {grpc_url, mtime, {grpc_path: mtime}}.
    '''

    def __init__(self):
        QObject.__init__(self)
#        self.url = None
#        self.grpc_server = None
        self._channels = {}
        self._cache_file_content = {}
        self._cache_file_includes = {}
        self._cache_file_unique_includes = {}
        self._cache_path = {}
        self._cache_packages = {}
        self._thread_list_path = None

    def stop(self):
        print("clear grpc channels...")
        remote.clear_channels()
        print("clear grpc channels...ok")
        self._cache_file_content.clear()
        self._cache_file_includes.clear()
        self._cache_file_unique_includes.clear()
        self._cache_path.clear()
        self._cache_packages.clear()

    def _delete_cache_for(self, grpc_path):
        try:
            del self._cache_file_content[grpc_path]
        except Exception:
            pass
        try:
            del self._cache_file_includes[grpc_path]
        except Exception:
            pass
        try:
            del self._cache_file_unique_includes[grpc_path]
        except Exception:
            pass
        try:
            del self._cache_path[grpc_path]
        except Exception:
            pass

    def package_name(self, grpc_path):
        result = None
        url, _path = grpc_split_url(grpc_path, with_scheme=True)
        path = grpc_path
        try:
            pl = self._cache_packages[url]
            while path and path != os.path.sep:
                if path in pl:
                    return pl[path]
                path = os.path.dirname(path).rstrip(os.path.sep)
        except Exception:
            import traceback
            print(traceback.format_exc())
        return result

    def get_packages(self, url=''):
        if url:
            grpc_url = get_nmd_url(url)
            if grpc_url in self._cache_packages:
                return self._cache_packages[grpc_url]
            return {}
        return self._cache_packages

#     def start(self, url='[::]:12322'):
#         self.url = url
#         rospy.loginfo("Start grpc server on %s" % url)
#         self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
#         # fgrpc.add_FileClientServiceServicer_to_server(FileClientServicer(), self.grpc_server)
#         self.grpc_server.add_insecure_port(url)
#         self.grpc_server.start()

    def get_file_manager(self, url='localhost:12321'):
        channel = remote.get_insecure_channel(url)
        if channel is not None:
            return fstub.FileStub(channel)
        print("return None in get_file_manager()")
        return None

    def get_launch_manager(self, url='localhost:12321'):
        channel = remote.get_insecure_channel(url)
        if channel is not None:
            return lstub.LaunchStub(channel)
        print("return None in get_launch_manager()")
        return None

    def list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        self._thread_list_path = threading.Thread(target=self._list_path_threaded, args=(grpc_path, clear_cache))
        self._thread_list_path.setDaemon(True)
        self._thread_list_path.start()

    def _list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        rospy.logdebug("list path: %s" % grpc_path)
        url, path = grpc_split_url(grpc_path)
        rospy.logdebug("list path: %s, %s" % (url, path))
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        result = None
        try:
            if not clear_cache:
                result = self._cache_path[grpc_path]
            else:
                self._cache_path['']  # only to cause an exception
        except KeyError:
            try:
                result = fm.list_path(path)
                if url not in self._cache_packages:
                    self.list_packages_threaded(grpc_path, clear_cache)
                print("OK LISTPATH", grpc_path)
            except Exception as e:
                remote.remove_insecure_channel(url)
                self.error.emit("list_path", "grpc://%s" % url, path, e)
                rospy.logwarn("LIST PATH: %s" % e)
        if result is not None:
            self._cache_path[grpc_path] = result
            self.listed_path.emit("grpc://%s" % url, path, result)

    def check_for_changed_files_threaded(self, grpc_path_dict):
        dests = {}
        for grpc_path, mtime in grpc_path_dict.items():
            url, path = grpc_split_url(grpc_path, with_scheme=True)
            if url not in dests:
                dests[url] = {}
            dests[url][path] = mtime
        for url, paths in dests.items():
            somethingThread = threading.Thread(target=self._check_for_changed_files_threaded, args=(url, paths))
            somethingThread.start()

    def _check_for_changed_files_threaded(self, grpc_url, path_dict):
        rospy.logdebug("check_for_changed_files_threaded: with %d files on %s" % (len(path_dict), grpc_url))
        url, _path = grpc_split_url(grpc_url, with_scheme=False)
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        try:
            response = fm.changed_files(path_dict)
            for item in response:
                print("EMIT CHANGED:", item.path, item.mtime)
                self.changed_file.emit(grpc_join(grpc_url, item.path), item.mtime)
        except Exception as e:
            self.error.emit("changed_files", "grpc://%s" % url, "", e)
            rospy.logwarn("check_for_changed_files_threaded: %s" % e)
            import traceback
            print(traceback.format_exc())

    def get_file_content(self, grpc_path='grpc://localhost:12321', force=False):
        result = ''
        try:
            if force:
                del self._cache_file_content[grpc_path]
            result = self._cache_file_content[grpc_path]
        except KeyError:
            rospy.logdebug("get file content for %s:" % grpc_path)
            url, path = grpc_split_url(grpc_path)
            fm = self.get_file_manager(url)
            if fm is None:
                raise Exception("Node manager daemon '%s' not reachable" % url)
            result = fm.get_file_content(path)
            self._cache_file_content[grpc_path] = result
        return result

    def save_file(self, grpc_path, content, mtime):
        rospy.logdebug("save_file_content: %s" % grpc_path)
        url, path = grpc_split_url(grpc_path)
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        result = fm.save_file_content(path, content, mtime)
        for ack in result:
            if ack.path == path and ack.mtime != 0:
                self._delete_cache_for(grpc_path)
                return ack.mtime
        return 0

    def rename(self, grpc_path_old='grpc://localhost:12321', grpc_path_new='grpc://localhost:12321'):
        url, old = grpc_split_url(grpc_path_old)
        _, new = grpc_split_url(grpc_path_new)
        rospy.logdebug("rename path on %s" % url)
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        print("nmd_rename: old", old, ", new: ", new)
        return fm.rename(old, new)

    def copy(self, grpc_path='grpc://localhost:12321', grpc_dest='grpc://localhost:12321'):
        url, path = grpc_split_url(grpc_path)
        url_dest, _ = grpc_split_url(grpc_dest)
        rospy.logdebug("copy '%s' to '%s'" % (grpc_path, url_dest))
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        print("nmd_rename: path", path, ", url_dest: ", url_dest)
        fm.copy(path, url_dest)

    def get_package_binaries(self, pkgname, grpc_url='grpc://localhost:12321'):
        url, _path = grpc_split_url(grpc_url)
        rospy.logdebug("get_package_binaries for '%s' from '%s'" % (pkgname, url))
        fm = self.get_file_manager(url)
        if fm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        response = fm.get_package_binaries(pkgname)
        url, _ = grpc_split_url(grpc_url, with_scheme=True)
        result = {}
        for item in response:
            result[grpc_join(url, item.path)] = item.mtime
        return result

    def _print_inc_file(self, indent, linenr, path, exists, inc_files):
        rospy.loginfo("%s %.4d\t%s %s" % (" " * indent, linenr, '+' if exists else '-', path))
        for ln, ph, ex, ifi in inc_files:
            self._print_inc_file(indent + 2, ln, ph, ex, ifi)

    def get_included_files_set(self, grpc_path='grpc://localhost:12321', recursive=True, include_pattern=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of included files.
        :rtype: [str]
        '''
        result = []
        try:
            result = self._cache_file_unique_includes[grpc_path]
        except KeyError:
            rospy.logdebug("get_included_files_set for %s:" % grpc_path)
            url, path = grpc_split_url(grpc_path, with_scheme=False)
            lm = self.get_launch_manager(url)
            if lm is None:
                raise Exception("Node manager daemon '%s' not reachable" % url)
            url, path = grpc_split_url(grpc_path, with_scheme=True)
            reply = lm.get_included_files_set(path, recursive, include_pattern)
            for fname in reply:
                result.append(grpc_join(url, fname))
            self._cache_file_unique_includes[grpc_path] = result
        return result

    def _append_grpc_prefix_to_include_files(self, url, included_files=[]):
        result = []
        for linenr, path, exists, file_list in included_files:
            recursive_list = self._append_grpc_prefix_to_include_files(file_list)
            result.append((linenr, grpc_join(url, path), exists, recursive_list))
        return result

    def get_included_files(self, grpc_path='grpc://localhost:12321', recursive=True, include_pattern=[]):
        '''
        :param str grpc_path: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of tuples with line number, path of included file, file exists or not and a recursive list of tuples with included files.
        :rtype: [(int, str, bool, [])]
        '''
        result = []
        try:
            result = self._cache_file_includes[grpc_path]
        except KeyError:
            url, path = grpc_split_url(grpc_path)
            lm = self.get_launch_manager(url)
            if lm is None:
                raise Exception("Node manager daemon '%s' not reachable" % url)
            rospy.logdebug("get_included_files for %s:" % grpc_path)
            reply = lm.get_included_files(path, recursive, include_pattern)
            url, _ = grpc_split_url(grpc_path, with_scheme=True)
            result = self._append_grpc_prefix_to_include_files(url, reply)
            self._cache_file_includes[grpc_path] = result
        return result

    def get_included_path(self, grpc_url_or_path='grpc://localhost:12321', text='', include_pattern=[]):
        '''
        :param str grpc_url_or_path: the url for node manager daemon
        :param str text: text where to search for included files
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns a list of tuples with line number, path of included file, file exists or not and a recursive list of tuples with included files.
        :rtype: [(int, str, bool, [])]
        '''
        url, _ = grpc_split_url(grpc_url_or_path)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        rospy.logdebug("get_included_path in text %s:" % text)
        reply = lm.get_included_path(text, include_pattern)
        url, _ = grpc_split_url(grpc_url_or_path, with_scheme=True)
        result = self._append_grpc_prefix_to_include_files(url, reply)
        return result

    def load_launch(self, grpc_path, masteruri='', host='', package='', launch='', args={}):
        url, path = grpc_split_url(grpc_path)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        myargs = args
        request_args = True
        nexttry = True
        ok = False
        launch_file = ''
        while nexttry:
            try:
#                 import inspect
#                 print("CALLER:", inspect.stack()[1][3])
                rospy.loginfo("load launch file %s:" % (grpc_path))
                launch_files, _argv = lm.load_launch(package, launch, path=path, args=myargs, request_args=request_args, masteruri=masteruri, host=host)
                if launch_files:
                    launch_file = launch_files[0]
                nexttry = False
                ok = True
            except exceptions.LaunchSelectionRequest as lsr:
                # TODO: selection dialog
                rospy.logwarn("%s\n  ...load the last one!" % lsr)
                path = lsr.choices[-1]
            except exceptions.ParamSelectionRequest as psr:
                rospy.loginfo("Params requered for: %s" % ["%s:=%s" % (name, value) for name, value in psr.choices.items()])
                request_args = False
                myargs = psr.choices
                # request the args: the dialog must run in the main thread of Qt
                params = {}
                for name, value in psr.choices.items():
                    params[name] = ('string', value)
                raise LaunchArgsSelectionRequest(grpc_path, params, 'Needs input for args')
            except exceptions.AlreadyOpenException as aoe:
                rospy.logwarn(aoe)
                nexttry = False
                ok = True
                launch_file = aoe.path
        rospy.loginfo("  %s: %s" % ('OK' if ok else "ERR", launch_file))
        return launch_file
#        includes = {path: mtime}
#        return launch_file, mtime, 

    def reload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("reload launch %s" % grpc_path)
        url, path = grpc_split_url(grpc_path)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        launch_file = ''
        launch_files, changed_nodes = lm.reload_launch(path, masteruri=masteruri)
        if launch_files:
            launch_file = launch_files[0]
        return launch_file, [node for node in changed_nodes]

    def unload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("unload launch %s" % grpc_path)
        url, path = grpc_split_url(grpc_path)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        launch_file = lm.unload_launch(path, masteruri)
        return launch_file
#         except exceptions.ResourceNotFound as rnf:
#             rospy.logwarn(rnf)
#         except exceptions.RemoteException as re:
#             rospy.logwarn("Unexpected error code %d; %s" % (re.code, re))
#         except Exception as e:
#             rospy.logwarn("ERROR WHILE RELOAD LAUNCH: %s" % e)
#             import traceback
#             print(traceback.format_exc())
#         rospy.loginfo("  %s: %s" % ('OK' if ok else "ERR", launch_file))

    def get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        thread = threading.Thread(target=self._get_mtimes_threaded, args=(grpc_path,))
        thread.setDaemon(True)
        thread.start()

    def _get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        url, path = grpc_split_url(grpc_path)
        rospy.logdebug("get nodes from %s" % url)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        rpath, mtime, included_files = lm.get_mtimes(path)
        url, _ = grpc_split_url(grpc_path, with_scheme=True)
        self.mtimes.emit(grpc_join(url, rpath), mtime, {grpc_join(url, pobj.path): pobj.mtime for pobj in included_files})

    def get_nodes(self, grpc_path='grpc://localhost:12321', masteruri=''):
        url, _ = grpc_split_url(grpc_path)
        rospy.logdebug("get nodes from %s" % url)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        launch_descriptions = lm.get_nodes(True, masteruri=masteruri)
        return launch_descriptions

    def list_packages_threaded(self, grpc_url_or_path='grpc://localhost:12321', clear_ros_cache=False):
        somethingThread = threading.Thread(target=self._list_packages, args=(grpc_url_or_path, clear_ros_cache))
        somethingThread.setDaemon(True)
        somethingThread.start()

    def _list_packages(self, grpc_url_or_path='grpc://localhost:12321', clear_ros_cache=False):
        print("grpc_url_or_path", grpc_url_or_path)
        url, path = grpc_split_url(grpc_url_or_path)
        print("LLLIIIRIISS", url, path)
        grpc_url = "grpc://%s" % url
        result = {}
        try:
            if not clear_ros_cache:
                result = self._cache_packages[grpc_url]
            else:
                self._cache_packages['']  # only to cause an exception
        except KeyError:
            rospy.logdebug("get packages %s" % grpc_url)
            fm = self.get_file_manager(url)
            if fm is None:
                raise Exception("Node manager daemon '%s' not reachable" % url)
            try:
                result = fm.list_packages(clear_ros_cache)
                fixed_result = {grpc_join(grpc_url, path): name for path, name in result.items()}
                self._cache_packages[grpc_url] = fixed_result
                self.packages.emit(grpc_url, fixed_result)
                self.packages_available.emit(grpc_url)
            except Exception as err:
                remote.remove_insecure_channel(url)
                self.error.emit("_list_packages", "grpc://%s" % url, path, err)

    def start_node(self, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False):
        rospy.loginfo("start node: %s on %s" % (name, masteruri))
        url, _ = grpc_split_url(grpc_path)
        lm = self.get_launch_manager(url)
        if lm is None:
            raise Exception("Node manager daemon '%s' not reachable" % url)
        try:
            return lm.start_node(name, masteruri=masteruri, reload_global_param=reload_global_param)
        except Exception as err:
            remote.remove_insecure_channel(url)
            raise err

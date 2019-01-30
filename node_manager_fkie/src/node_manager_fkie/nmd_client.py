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
import grpc
import os
from python_qt_binding.QtCore import QObject, Signal
import rospy
import threading

# import node_manager_fkie as nm

import node_manager_daemon_fkie.exceptions as exceptions
import node_manager_daemon_fkie.remote as remote
import node_manager_daemon_fkie.file_stub as fstub
import node_manager_daemon_fkie.launch_stub as lstub
import node_manager_daemon_fkie.screen_stub as sstub
import node_manager_daemon_fkie.version_stub as vstub
from node_manager_daemon_fkie.startcfg import StartConfig
from node_manager_daemon_fkie import url as nmdurl
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


class ThreadManager(QObject):
    '''
    Class to manage threads with request to remote grpc-server. This can avoid multiple same requests.
    '''

    def __init__(self):
        QObject.__init__(self)
        self._threads = {}

    def __del__(self):
        self.finished('')

    def has_thread(self, thread_id):
        try:
            return self._threads[thread_id].is_alive()
        except Exception:
            pass
        return False

    def start_thread(self, thread_id, target, args=()):
        '''
        Starts a new thread to execute a callable object given by target.
        Avoids the start of new thread if one with same thread_id currently active.
        :param str thread_id: thread identification string determine by caller.
        :param object target: callable object to be invoked by start of new thread
        :param tule args: the argument tuple for the target invocation. Defaults to ().
        '''
        if not rospy.is_shutdown() and not self.has_thread(thread_id):
            thread = threading.Thread(target=target, args=args)
            thread.setDaemon(True)
            self._threads[thread_id] = thread
            thread.start()
            return True
        return False

    def finished(self, thread_id):
        '''
        Removes a thread with given thread_id from the managed list.
        :param str thread_id: thread identification string used while start_thread().
        :raise KeyError: if thread_id does not exists.
        '''
        if thread_id:
            del self._threads[thread_id]
        else:
            self._threads.clear()


class NmdClient(QObject):
    '''
    A thread to get data from node manager daemon and
    publish there be sending a QT signal.
    '''
    listed_path = Signal(str, str, list)
    '''
    :ivar str,str,list listed_path: listed_path is a signal, which is emitted, if path is listed successful {url, path, list with paths}.
    '''
    error = Signal(str, str, str, Exception)
    '''
    :ivar  str,str,str,Exception error: error is a signal, which is emitted on errors {method, url, path, Exception}.
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
    mtimes = Signal(str, float, dict)
    '''
    :ivar str,float,dict mtimes: this signal is emitted on new mtimes for requested files {grpc_url, mtime, {grpc_path: mtime}}.
    '''
    changed_binaries = Signal(str, dict)
    '''
    :ivar str,dict changed_binaries: this signal is emitted on result of change binaries request {grpc_url, {node names: mtime}}.
    '''
    multiple_screens = Signal(str, dict)
    '''
    :ivar str,dict multiple_screens: this signal is emitted if new multiple screen are detected {grpc_url, {node_name: [screen_session]}}.
    '''
    launch_nodes = Signal(str, list)
    '''
      :ivar str,list update_signal: is a signal, which is emitted, if a new list with launch files is retrieved. The signal has the URI of the node manager daemon and a list with node_manager_daemon_fkie.launch_description.LaunchDescription.
    '''
    version_signal = Signal(str, str, str)
    '''
      :ivar str,str,str version_signal: signal emitted on new version {grpc_url, version, date}.
    '''

    def __init__(self):
        QObject.__init__(self)
        self._threads = ThreadManager()
        self._args_lock = threading.RLock()
        self._channels = {}
        self._cache_file_content = {}
        self._cache_file_includes = {}
        self._cache_file_unique_includes = {}
        self._cache_path = {}
        self._cache_packages = {}
        self._launch_args = {}

    def stop(self):
        print("clear grpc channels...")
        self._threads.finished('')
        remote.clear_channels()
        print("clear grpc channels...ok")
        self._cache_file_content.clear()
        self._cache_file_includes.clear()
        self._cache_file_unique_includes.clear()
        self._cache_path.clear()
        self._cache_packages.clear()
        with self._args_lock:
            self._launch_args.clear()

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

    def launch_args(self, grpc_path):
        with self._args_lock:
            if grpc_path in self._launch_args:
                return self._launch_args[grpc_path]
        return {}

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

    def get_file_manager(self, uri='localhost:12321'):
        channel = remote.get_insecure_channel(uri)
        if channel is not None:
            return fstub.FileStub(channel)
        raise Exception("Node manager daemon '%s' not reachable" % uri)

    def get_launch_manager(self, uri='localhost:12321'):
        channel = remote.get_insecure_channel(uri)
        if channel is not None:
            return lstub.LaunchStub(channel)
        raise Exception("Node manager daemon '%s' not reachable" % uri)

    def get_screen_manager(self, uri='localhost:12321'):
        channel = remote.get_insecure_channel(uri)
        if channel is not None:
            return sstub.ScreenStub(channel)
        raise Exception("Node manager daemon '%s' not reachable" % uri)

    def get_version_manager(self, uri='localhost:12321'):
        channel = remote.get_insecure_channel(uri)
        if channel is not None:
            return vstub.VersionStub(channel)
        raise Exception("Node manager daemon '%s' not reachable" % uri)

    def list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        self._threads.start_thread("lpt_%s" % grpc_path, target=self._list_path_threaded, args=(grpc_path, clear_cache))

    def _list_path_threaded(self, grpc_path='grpc://localhost:12321', clear_cache=False):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] list path: %s, '%s'" % (uri, path))
        fm = self.get_file_manager(uri)
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
                    self.get_loaded_files_threaded(grpc_path)
            except Exception as e:
                remote.remove_insecure_channel(uri)
                self.error.emit("list_path", "grpc://%s" % uri, path, e)
                # rospy.logwarn("LIST PATH: %s" % e)
        if result is not None:
            self._cache_path[grpc_path] = result
            self.listed_path.emit("grpc://%s" % uri, path, result)
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
        fm = self.get_file_manager(uri)
        try:
            response = fm.changed_files(path_dict)
            for item in response:
                self.changed_file.emit(nmdurl.join(grpc_url, item.path), item.mtime)
        except Exception as e:
            self.error.emit("changed_files", "grpc://%s" % uri, "", e)
            # rospy.logwarn("check_for_changed_files_threaded: %s" % e)
        url, _path = nmdurl.split(grpc_url, with_scheme=True)
        if hasattr(self, '_threads'):
            self._threads.finished("cft_%s" % url)

    def get_file_content(self, grpc_path='grpc://localhost:12321', force=False):
        result = ''
        try:
            if force:
                del self._cache_file_content[grpc_path]
            result = self._cache_file_content[grpc_path]
        except KeyError:
            rospy.logdebug("get file content for %s:" % grpc_path)
            uri, path = nmdurl.split(grpc_path)
            fm = self.get_file_manager(uri)
            result = fm.get_file_content(path)
            self._cache_file_content[grpc_path] = result
        return result

    def save_file(self, grpc_path, content, mtime):
        rospy.logdebug("save_file_content: %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        fm = self.get_file_manager(uri)
        result = fm.save_file_content(path, content, mtime)
        for ack in result:
            if ack.path == path and ack.mtime != 0:
                self._delete_cache_for(grpc_path)
                return ack.mtime
        return 0

    def rename(self, grpc_path_old='grpc://localhost:12321', grpc_path_new='grpc://localhost:12321'):
        uri, old = nmdurl.split(grpc_path_old)
        _, new = nmdurl.split(grpc_path_new)
        rospy.logdebug("rename path on %s" % uri)
        fm = self.get_file_manager(uri)
        return fm.rename(old, new)

    def copy(self, grpc_path='grpc://localhost:12321', grpc_dest='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("copy '%s' to '%s'" % (grpc_path, grpc_dest))
        fm = self.get_file_manager(uri)
        fm.copy(path, grpc_dest)

    def get_package_binaries(self, pkgname, grpc_url='grpc://localhost:12321'):
        uri, _path = nmdurl.split(grpc_url)
        rospy.logdebug("get_package_binaries for '%s' from '%s'" % (pkgname, uri))
        fm = self.get_file_manager(uri)
        response = fm.get_package_binaries(pkgname)
        url, _ = nmdurl.split(grpc_url, with_scheme=True)
        result = {}
        for item in response:
            result[nmdurl.join(url, item.path)] = item.mtime
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
            rospy.logdebug("get_included_files_set for %s, recursive: %s" % (grpc_path, recursive))
            uri, path = nmdurl.split(grpc_path, with_scheme=False)
            lm = self.get_launch_manager(uri)
            url, path = nmdurl.split(grpc_path, with_scheme=True)
            reply = lm.get_included_files_set(path, recursive, include_pattern)
            for fname in reply:
                result.append(nmdurl.join(url, fname))
            self._cache_file_unique_includes[grpc_path] = result
        return result

    def get_included_files(self, grpc_path='grpc://localhost:12321', recursive=True, include_pattern=[]):
        '''
        :param str grpc_path: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :return: Returns an iterator for tuple with root path, line number, path of included file, file exists or not, file size and dictionary with defined arguments.
        :rtype: iterator (str, int, str, bool, int, {str: str})
        '''
        dorequest = False
        try:
            for entry in self._cache_file_includes[grpc_path]:
                yield entry
        except KeyError:
            dorequest = True
        if dorequest:
            current_path = grpc_path
            try:
                uri, path = nmdurl.split(current_path)
                lm = self.get_launch_manager(uri)
                rospy.logdebug("get_included_files for %s, recursive: %s, pattern: %s" % (grpc_path, recursive, include_pattern))
                reply = lm.get_included_files(path, recursive, include_pattern)
                url, _ = nmdurl.split(grpc_path, with_scheme=True)
                # initialize requested path in cache
                if grpc_path not in self._cache_file_includes:
                    self._cache_file_includes[grpc_path] = []
                for root_path, linenr, path, exists, size, include_args in reply:
                    current_path = nmdurl.join(url, root_path)
                    entry = (linenr, nmdurl.join(url, path), exists, size, include_args)
                    # initialize and add returned root path to cache
                    if current_path not in self._cache_file_includes:
                        self._cache_file_includes[current_path] = []
                    self._cache_file_includes[current_path].append(entry)
                    yield entry
            except grpc._channel._Rendezvous as grpc_error:
                self._delete_cache_for(grpc_path)
                self._delete_cache_for(current_path)
                if grpc_error.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    raise exceptions.GrpcTimeout(grpc_path, grpc_error)
                raise

    def get_interpreted_path(self, grpc_url_or_path='grpc://localhost:12321', text=[]):
        '''
        :param str grpc_url_or_path: the url for node manager daemon
        :param [str] text: list of string with text to interpret to a path
        :return: Returns an iterator for tuples with interpreted path and file exists or not
        :rtype: tuple(str, bool)
        '''
        uri, _ = nmdurl.split(grpc_url_or_path)
        lm = self.get_launch_manager(uri)
        rospy.logdebug("get_interpreted_path in text %s" % text)
        reply = lm.get_interpreted_path(text)
        url, _ = nmdurl.split(grpc_url_or_path, with_scheme=True)
        for path, exists in reply:
            yield (nmdurl.join(url, path), exists)

    def load_launch(self, grpc_path, masteruri='', host='', package='', launch='', args={}):
        '''
        Loads given file on remote grpc-server.

        :return: Path of loaded file
        :rtype: str
        '''
        uri, path = nmdurl.split(grpc_path)
        lm = self.get_launch_manager(uri)
        myargs = args
        request_args = True
        nexttry = True
        ok = False
        launch_file = ''
        args_res = {}
        while nexttry:
            try:
                rospy.logdebug("load launch file on gRPC server: %s" % (grpc_path))
                launch_file, args_res = lm.load_launch(package, launch, path=path, args=myargs, request_args=request_args, masteruri=masteruri, host=host)
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
            except grpc._channel._Rendezvous as grpc_error:
                if grpc_error.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    raise exceptions.GrpcTimeout(grpc_path, grpc_error)
                raise
        launch_file = nmdurl.join("grpc://%s" % uri, launch_file)
        rospy.logdebug("  load launch file result - %s: %s" % ('OK' if ok else "ERR", launch_file))
        with self._args_lock:
            rospy.logdebug("add args after load %s: %s" % (launch_file, args_res))
            self._launch_args[launch_file] = args_res
        return launch_file, args_res

    def reload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("reload launch %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        lm = self.get_launch_manager(uri)
        launch_file = ''
        try:
            launch_files, changed_nodes = lm.reload_launch(path, masteruri=masteruri)
            if launch_files:
                launch_file = launch_files[0]
            return launch_file, [node for node in changed_nodes]
        except grpc._channel._Rendezvous as grpc_error:
            if grpc_error.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                raise exceptions.GrpcTimeout(grpc_path, grpc_error)
            raise

    def unload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("unload launch %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        lm = self.get_launch_manager(uri)
        launch_file = lm.unload_launch(path, masteruri)
        with self._args_lock:
            if launch_file in self._launch_args:
                rospy.logdebug("delete args after unload %s" % launch_file)
                del self._launch_args[launch_file]
        return launch_file

    def get_loaded_files_threaded(self, grpc_path='grpc://localhost:12321'):
        self._threads.start_thread("glft_%s" % grpc_path, target=self._get_loaded_files_threaded, args=(grpc_path,))

    def _get_loaded_files_threaded(self, grpc_path='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get loaded_files from %s" % uri)
        try:
            lm = self.get_launch_manager(uri)
            result = lm.get_loaded_files()
            url, _ = nmdurl.split(grpc_path, with_scheme=True)
            for _package, path, args, _masteruri, _host in result:
                with self._args_lock:
                    gpath = nmdurl.join(url, path)
                    rospy.logdebug("[thread] add args for %s: %s" % (gpath, args))
                    self._launch_args[gpath] = args
        except Exception:
            import traceback
            print(traceback.format_exc())
        if hasattr(self, '_threads'):
            self._threads.finished("glft_%s" % grpc_path)

    def get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        self._threads.start_thread("gmt_%s" % grpc_path, target=self._get_mtimes_threaded, args=(grpc_path,))

    def _get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get mtimes from %s" % uri)
        try:
            lm = self.get_launch_manager(uri)
            rpath, mtime, included_files = lm.get_mtimes(path)
            url, _ = nmdurl.split(grpc_path, with_scheme=True)
            self.mtimes.emit(nmdurl.join(url, rpath), mtime, {nmdurl.join(url, pobj.path): pobj.mtime for pobj in included_files})
        except Exception:
            pass
        if hasattr(self, '_threads'):
            self._threads.finished("gmt_%s" % grpc_path)

    def get_changed_binaries_threaded(self, grpc_url='grpc://localhost:12321', nodes=[]):
        self._threads.start_thread("gcbt_%s" % grpc_url, target=self._get_changed_binaries_threaded, args=(grpc_url, nodes))

    def _get_changed_binaries_threaded(self, grpc_url='grpc://localhost:12321', nodes=[]):
        uri, _path = nmdurl.split(grpc_url)
        rospy.logdebug("[thread] get changed binaries from %s" % uri)
        try:
            lm = self.get_launch_manager(uri)
            nodes = lm.get_changed_binaries(nodes)
            self.changed_binaries.emit(grpc_url, nodes)
        except Exception:
            import traceback
            print(traceback.format_exc())
        if hasattr(self, '_threads'):
            self._threads.finished("gcbt_%s" % grpc_url)

    def get_nodes(self, grpc_path='grpc://localhost:12321', masteruri=''):
        uri, _ = nmdurl.split(grpc_path)
        rospy.logdebug("get nodes from %s" % uri)
        lm = self.get_launch_manager(uri)
        try:
            launch_descriptions = lm.get_nodes(True, masteruri=masteruri)
            return launch_descriptions
        except grpc.RpcError as gerr:
            rospy.logdebug("remove connection", uri)
            remote.remove_insecure_channel(uri)
            raise gerr

    def get_nodes_threaded(self, grpc_path='grpc://localhost:12321', masteruri=''):
        self._threads.start_thread("gn_%s_%s" % (grpc_path, masteruri), target=self._get_nodes_threaded, args=(grpc_path, masteruri))

    def _get_nodes_threaded(self, grpc_path='grpc://localhost:12321', masteruri=''):
        uri, _ = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get nodes from %s" % uri)
        lm = self.get_launch_manager(uri)
        try:
            launch_descriptions = lm.get_nodes(True, masteruri=masteruri)
            clean_url = nmdurl.nmduri_from_path(grpc_path)
            for ld in launch_descriptions:
                ld.path = nmdurl.join(clean_url, ld.path)
            self.launch_nodes.emit(clean_url, launch_descriptions)
        except Exception as err:
            self.error.emit("_get_nodes", grpc_path, masteruri, err)
        if hasattr(self, '_threads'):
            self._threads.finished("gn_%s_%s" % (grpc_path, masteruri))

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
            fm = self.get_file_manager(uri)
            try:
                result = fm.list_packages(clear_ros_cache)
                fixed_result = {nmdurl.join(grpc_url, path): name for path, name in result.items()}
                self._cache_packages[grpc_url] = fixed_result
                self.packages.emit(grpc_url, fixed_result)
                self.packages_available.emit(grpc_url)
            except Exception as err:
                remote.remove_insecure_channel(uri)
                self.error.emit("_list_packages", "grpc://%s" % uri, path, err)
        if hasattr(self, '_threads'):
            self._threads.finished("gmt_%s_%d" % (grpc_url_or_path, clear_ros_cache))

    def start_node(self, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False, loglevel='', logformat=''):
        rospy.loginfo("start node: %s with %s" % (name, grpc_path))
        uri, _ = nmdurl.split(grpc_path)
        lm = self.get_launch_manager(uri)
        try:
            return lm.start_node(name, loglevel=loglevel, logformat=logformat, masteruri=masteruri, reload_global_param=reload_global_param)
        except grpc.RpcError as gerr:
            rospy.logdebug("remove connection", uri)
            remote.remove_insecure_channel(uri)
            raise gerr
        except Exception as err:
            raise err

    def start_standalone_node(self, grpc_url, package, binary, name, ns, args=[], env={}, masteruri=None, host=None):
        rospy.loginfo("start standalone node: %s on %s" % (name, grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        lm = self.get_launch_manager(uri)
        try:
            startcfg = StartConfig(package, binary)
            startcfg.name = name
            startcfg.namespace = ns
            startcfg.fullname = rospy.names.ns_join(ns, name)
            startcfg.prefix = ''
            startcfg.cwd = ''
            startcfg.env = env
            startcfg.remaps = {}
            startcfg.params = {}
            startcfg.clear_params = []
            startcfg.args = args
            startcfg.masteruri = masteruri
            startcfg.host = host
            startcfg.loglevel = ''
            startcfg.logformat = ''
            startcfg.respawn = False
            startcfg.respawn_delay = 30
            startcfg.respawn_max = 0
            startcfg.respawn_min_runtime = 0
            return lm.start_standalone_node(startcfg)
        except grpc.RpcError as err:
            rospy.logdebug("remove connection", uri)
            remote.remove_insecure_channel(uri)
            raise err

    def get_start_cfg(self, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False, loglevel='', logformat=''):
        rospy.logdebug("get start configuration for '%s' from %s" % (name, grpc_path))
        uri, _ = nmdurl.split(grpc_path)
        lm = self.get_launch_manager(uri)
        try:
            return lm.get_start_cfg(name, loglevel=loglevel, logformat=logformat, masteruri=masteruri, reload_global_param=reload_global_param)
        except Exception as err:
            raise err

    def get_all_screens(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get all screens from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm = self.get_screen_manager(uri)
        return sm.all_screens()

    def get_screens(self, grpc_url='grpc://localhost:12321', node=''):
        rospy.logdebug("get screen from %s for %s" % (grpc_url, node))
        uri, _ = nmdurl.split(grpc_url)
        sm = self.get_screen_manager(uri)
        return sm.screens(node)

    def multiple_screens_threaded(self, grpc_url='grpc://localhost:12321'):
        '''
        The existence of multiple screens for one node can lead to failures.
        This method starts a thread to request all nodes with multiple screens.
        On finish this method emit a qt-signal of type NmdClient.multiple_screens.

        :param str grpc_url: the url for grpc-server
        '''
        self._threads.start_thread("mst_%s" % grpc_url, target=self._multiple_screens, args=(grpc_url,))

    def _multiple_screens(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("get multiple screens from %s" % (grpc_url))
        try:
            uri, _ = nmdurl.split(grpc_url)
            sm = self.get_screen_manager(uri)
            result = sm.multiple_screens()
            self.multiple_screens.emit(grpc_url, result)
        except Exception as e:
            self.error.emit("get_multiple_screens", "grpc://%s" % uri, "", e)
        if hasattr(self, '_threads'):
            self._threads.finished("mst_%s" % grpc_url)

    def rosclean(self, grpc_url='grpc://localhost:12321'):
        rospy.logdebug("rosclean purge -y on %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        sm = self.get_screen_manager(uri)
        return sm.rosclean()

    def delete_log(self, grpc_url='grpc://localhost:12321', nodes=[]):
        rospy.logdebug("delete logs on %s for %s" % (grpc_url, nodes))
        uri, _ = nmdurl.split(grpc_url)
        sm = self.get_screen_manager(uri)
        return sm.delete_log(nodes)

    def get_version_threaded(self, grpc_url='grpc://localhost:12321'):
        self._threads.start_thread("gvt_%s" % grpc_url, target=self.get_version, args=(grpc_url, True))

    def get_version(self, grpc_url='grpc://localhost:12321', threaded=False):
        rospy.logdebug("get version from %s" % (grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        vm = self.get_version_manager(uri)
        try:
            version, date = vm.get_version()
            if threaded:
                self.version_signal.emit(grpc_url, version, date)
                self._threads.finished("gvt_%s" % grpc_url)
            return version, date
        except Exception as e:
            self.error.emit("get_version", "grpc://%s" % uri, "", e)

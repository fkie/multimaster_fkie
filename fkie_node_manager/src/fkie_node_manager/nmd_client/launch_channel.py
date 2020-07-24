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



import grpc
import rospy
import threading
from python_qt_binding.QtCore import Signal

import fkie_node_manager_daemon.exceptions as exceptions
import fkie_node_manager_daemon.launch_stub as lstub
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager_daemon.startcfg import StartConfig

from .channel_interface import ChannelInterface


class BinarySelectionRequest(Exception):
    ''' '''

    def __init__(self, choices, error):
        Exception.__init__(self)
        self.choices = choices
        self.error = error

    def __str__(self):
        return 'BinarySelectionRequest from %s::%s' % (self.choices, repr(self.error))


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


class LaunchChannel(ChannelInterface):

    mtimes = Signal(str, float, dict)
    '''
    :ivar str,float,dict mtimes: this signal is emitted on new mtimes for requested files {grpc_url, mtime, {grpc_path: mtime}}.
    '''
    changed_binaries = Signal(str, dict)
    '''
    :ivar str,dict changed_binaries: this signal is emitted on result of change binaries request {grpc_url, {node names: mtime}}.
    '''
    launch_nodes = Signal(str, list)
    '''
      :ivar str,list update_signal: is a signal, which is emitted, if a new list with launch files is retrieved. The signal has the URI of the node manager daemon and a list with fkie_node_manager_daemon.launch_description.LaunchDescription.
    '''

    def __init__(self):
        ChannelInterface.__init__(self)
        self._args_lock = threading.RLock()
        self._cache_file_includes = {}
        self._cache_file_unique_includes = {}
        self._launch_args = {}

    def clear_cache(self, grpc_path=''):
        if grpc_path:
            try:
                del self._cache_file_includes[grpc_path]
            except Exception:
                pass
            try:
                del self._cache_file_unique_includes[grpc_path]
            except Exception:
                pass
        else:
            self._cache_file_includes.clear()
            self._cache_file_unique_includes.clear()
            with self._args_lock:
                self._launch_args.clear()

    def get_launch_manager(self, uri='localhost:12321'):
        channel = self.get_insecure_channel(uri)
        return lstub.LaunchStub(channel), channel

    def launch_args(self, grpc_path):
        with self._args_lock:
            if grpc_path in self._launch_args:
                return self._launch_args[grpc_path]
        return {}

    def _print_inc_file(self, indent, linenr, path, exists, inc_files):
        rospy.loginfo("%s %.4d\t%s %s" % (" " * indent, linenr, '+' if exists else '-', path))
        for ln, ph, ex, ifi in inc_files:
            self._print_inc_file(indent + 2, ln, ph, ex, ifi)

    def get_included_files_set(self, grpc_path='grpc://localhost:12321', recursive=True, include_pattern=[], search_in_ext=[]):
        '''
        :param str root: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :param search_in_ext: file extensions to search in
        :type search_in_ext: [str]
        :return: Returns a list of included files.
        :rtype: [str]
        '''
        result = []
        try:
            result = self._cache_file_unique_includes[grpc_path]
        except KeyError:
            rospy.logdebug("get_included_files_set for %s, recursive: %s" % (grpc_path, recursive))
            uri, path = nmdurl.split(grpc_path, with_scheme=False)
            lm, channel = self.get_launch_manager(uri)
            url, path = nmdurl.split(grpc_path, with_scheme=True)
            reply = lm.get_included_files_set(path, recursive, {}, include_pattern, search_in_ext)
            for fname in reply:
                result.append(nmdurl.join(url, fname))
            self._cache_file_unique_includes[grpc_path] = result
            self.close_channel(channel, uri)
        return result

    def get_included_files(self, grpc_path='grpc://localhost:12321', recursive=True, include_args={}, include_pattern=[], search_in_ext=[]):
        '''
        :param str grpc_path: the root path to search for included files
        :param bool recursive: True for recursive search
        :param include_args: dictionary with arguments to override while include.
        :type include_args: {str: str}
        :param include_pattern: the list with regular expression patterns to find include files.
        :type include_pattern: [str]
        :param search_in_ext: file extensions to search in
        :type search_in_ext: [str]
        :return: Returns an iterator for tuple with root path, line number, path of included file, file exists or not, file size and dictionary with defined arguments.
        :rtype: iterator (str, int, str, bool, int, {str: str})
        '''
        dorequest = False
        try:
            for entry in self._cache_file_includes[grpc_path]:
                do_return = True
                if not recursive and entry.rec_depth != 0:
                    do_return = False
                if do_return:
                    rospy.logdebug("get_included_files from cache: %s, include_args: %s" % (entry.inc_path, entry.args))
                    yield entry
        except KeyError:
            dorequest = True
        if dorequest:
            current_path = grpc_path
            try:
                uri, path = nmdurl.split(current_path)
                lm, channel = self.get_launch_manager(uri)
                rospy.logdebug("get_included_files for %s, recursive: %s, include_args: %s, pattern: %s, search_in_ext: %s" % (grpc_path, recursive, include_args, include_pattern, search_in_ext))
                reply = lm.get_included_files(path, recursive, include_args, include_pattern, search_in_ext)
                url, _ = nmdurl.split(grpc_path, with_scheme=True)
                # initialize requested path in cache
                if recursive:
                    if grpc_path not in self._cache_file_includes:
                        self._cache_file_includes[grpc_path] = []
                for inc_file in reply:
                    entry = inc_file
                    entry.path_or_str = nmdurl.join(url, inc_file.path_or_str)
                    entry.inc_path = nmdurl.join(url, inc_file.inc_path)
                    # initialize and add returned root path to cache, only on recursive
                    if recursive:
                        if current_path not in self._cache_file_includes:
                            self._cache_file_includes[current_path] = []
                        self._cache_file_includes[current_path].append(entry)
                    yield entry
            except grpc._channel._Rendezvous as grpc_error:
                self.clear_cache(grpc_path)
                self.clear_cache(current_path)
                if grpc_error.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    raise exceptions.GrpcTimeout(grpc_path, grpc_error)
                raise
            finally:
                self.close_channel(channel, uri)

    def get_interpreted_path(self, grpc_url_or_path='grpc://localhost:12321', text=[]):
        '''
        :param str grpc_url_or_path: the url for node manager daemon
        :param [str] text: list of string with text to interpret to a path
        :return: Returns an iterator for tuples with interpreted path and file exists or not
        :rtype: tuple(str, bool)
        '''
        uri, _ = nmdurl.split(grpc_url_or_path)
        lm, channel = self.get_launch_manager(uri)
        rospy.logdebug("get_interpreted_path in text %s" % text)
        reply = lm.get_interpreted_path(text)
        url, _ = nmdurl.split(grpc_url_or_path, with_scheme=True)
        for path, exists in reply:
            yield (nmdurl.join(url, path), exists)
        self.close_channel(channel, uri)

    def load_launch(self, grpc_path, masteruri='', host='', package='', launch='', args={}):
        '''
        Loads given file on remote grpc-server.

        :return: Path of loaded file
        :rtype: str
        '''
        uri, path = nmdurl.split(grpc_path)
        lm, channel = self.get_launch_manager(uri)
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
                    params[name] = {':value': value, ':type': 'string'}
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
            finally:
                self.close_channel(channel, uri)
        launch_file = nmdurl.join("grpc://%s" % uri, launch_file)
        rospy.logdebug("  load launch file result - %s: %s" % ('OK' if ok else "ERR", launch_file))
        with self._args_lock:
            rospy.logdebug("add args after load %s: %s" % (launch_file, args_res))
            self._launch_args[launch_file] = args_res
        return launch_file, args_res

    def reload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("reload launch %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        lm, channel = self.get_launch_manager(uri)
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
        finally:
            self.close_channel(channel, uri)

    def unload_launch(self, grpc_path, masteruri=''):
        rospy.logdebug("unload launch %s" % grpc_path)
        uri, path = nmdurl.split(grpc_path)
        lm, channel = self.get_launch_manager(uri)
        launch_file = lm.unload_launch(path, masteruri)
        with self._args_lock:
            if launch_file in self._launch_args:
                rospy.logdebug("delete args after unload %s" % launch_file)
                del self._launch_args[launch_file]
        self.close_channel(channel, uri)
        return launch_file

    def get_loaded_files_threaded(self, grpc_path='grpc://localhost:12321'):
        self._threads.start_thread("glft_%s" % grpc_path, target=self._get_loaded_files_threaded, args=(grpc_path,))

    def _get_loaded_files_threaded(self, grpc_path='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get loaded_files from %s" % uri)
        try:
            lm, channel = self.get_launch_manager(uri)
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
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("glft_%s" % grpc_path)

    def get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        self._threads.start_thread("gmt_%s" % grpc_path, target=self._get_mtimes_threaded, args=(grpc_path,))

    def _get_mtimes_threaded(self, grpc_path='grpc://localhost:12321'):
        uri, path = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get mtimes from %s" % uri)
        try:
            lm, channel = self.get_launch_manager(uri)
            rpath, mtime, included_files = lm.get_mtimes(path)
            url, _ = nmdurl.split(grpc_path, with_scheme=True)
            self.mtimes.emit(nmdurl.join(url, rpath), mtime, {nmdurl.join(url, pobj.path): pobj.mtime for pobj in included_files})
        except Exception:
            pass
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("gmt_%s" % grpc_path)

    def get_changed_binaries_threaded(self, grpc_url='grpc://localhost:12321', nodes=[]):
        self._threads.start_thread("gcbt_%s" % grpc_url, target=self._get_changed_binaries_threaded, args=(grpc_url, nodes))

    def _get_changed_binaries_threaded(self, grpc_url='grpc://localhost:12321', nodes=[]):
        uri, _path = nmdurl.split(grpc_url)
        rospy.logdebug("[thread] get changed binaries from %s" % uri)
        try:
            lm, channel = self.get_launch_manager(uri)
            nodes = lm.get_changed_binaries(nodes)
            self.changed_binaries.emit(grpc_url, nodes)
        except Exception:
            import traceback
            print(traceback.format_exc())
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("gcbt_%s" % grpc_url)

    def get_nodes(self, grpc_path='grpc://localhost:12321', masteruri=''):
        uri, _ = nmdurl.split(grpc_path)
        rospy.logdebug("get nodes from %s" % uri)
        lm, channel = self.get_launch_manager(uri)
        try:
            launch_descriptions = lm.get_nodes(True, masteruri=masteruri)
            return launch_descriptions
        except grpc.RpcError as gerr:
            rospy.logdebug("remove connection %s" % uri)
            raise gerr
        finally:
            self.close_channel(channel, uri)

    def get_nodes_threaded(self, grpc_path='grpc://localhost:12321', masteruri=''):
        self._threads.start_thread("gn_%s_%s" % (grpc_path, masteruri), target=self._get_nodes_threaded, args=(grpc_path, masteruri))

    def _get_nodes_threaded(self, grpc_path='grpc://localhost:12321', masteruri=''):
        uri, _ = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] get nodes from %s" % uri)
        lm, channel = self.get_launch_manager(uri)
        try:
            launch_descriptions = lm.get_nodes(True, masteruri=masteruri)
            clean_url = nmdurl.nmduri_from_path(grpc_path)
            for ld in launch_descriptions:
                ld.path = nmdurl.join(clean_url, ld.path)
            self.launch_nodes.emit(clean_url, launch_descriptions)
        except Exception as err:
            self.error.emit("_get_nodes", grpc_path, masteruri, err)
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("gn_%s_%s" % (grpc_path, masteruri))

    def start_node(self, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False, loglevel='', logformat='', path='', cmd_prefix=''):
        rospy.loginfo("start node: %s with %s" % (name, grpc_path))
        uri, opt_launch = nmdurl.split(grpc_path)
        lm, channel = self.get_launch_manager(uri)
        try:
            return lm.start_node(name, opt_binary=path, opt_launch=opt_launch, loglevel=loglevel, logformat=logformat, masteruri=masteruri, reload_global_param=reload_global_param, cmd_prefix=cmd_prefix)
        except grpc.RpcError as gerr:
            rospy.logdebug("remove connection %s" % uri)
            raise gerr
        except exceptions.BinarySelectionRequest as bsr:
            rospy.loginfo("Question while start node: %s" % bsr.error)
            binaries = bsr.choices
            raise BinarySelectionRequest(binaries, 'Needs binary selection')
        except Exception as err:
            raise err
        finally:
            self.close_channel(channel, uri)

    def start_standalone_node(self, grpc_url, package, binary, name, ns, args=[], env={}, masteruri=None, host=None):
        rospy.loginfo("start standalone node: %s on %s" % (name, grpc_url))
        uri, _ = nmdurl.split(grpc_url)
        lm, channel = self.get_launch_manager(uri)
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
            rospy.logdebug("remove connection %s" % uri)
            raise err
        except exceptions.BinarySelectionRequest as bsr:
            rospy.loginfo("Question while start node: %s" % bsr.error)
            binaries = bsr.choices
            raise BinarySelectionRequest(binaries, 'Needs binary selection')
        finally:
            self.close_channel(channel, uri)

    def get_start_cfg(self, name, grpc_path='grpc://localhost:12321', masteruri='', reload_global_param=False, loglevel='', logformat=''):
        rospy.logdebug("get start configuration for '%s' from %s" % (name, grpc_path))
        uri, _ = nmdurl.split(grpc_path)
        lm, channel = self.get_launch_manager(uri)
        try:
            return lm.get_start_cfg(name, loglevel=loglevel, logformat=logformat, masteruri=masteruri, reload_global_param=reload_global_param)
        except Exception as err:
            raise err
        finally:
            self.close_channel(channel, uri)

    def reset_package_path_threaded(self, grpc_path='grpc://localhost:12321'):
        self._threads.start_thread("rpp_%s" % (grpc_path), target=self._reset_package_path, args=(grpc_path,))

    def _reset_package_path(self, grpc_path='grpc://localhost:12321'):
        uri, _ = nmdurl.split(grpc_path)
        rospy.logdebug("[thread] reset package path on %s" % uri)
        lm, channel = self.get_launch_manager(uri)
        try:
            lm.reset_package_path()
        except Exception as err:
            self.error.emit("_reset_package_path", grpc_path, "", err)
        finally:
            self.close_channel(channel, uri)
        if hasattr(self, '_threads'):
            self._threads.finished("rpp_%s" % (grpc_path))

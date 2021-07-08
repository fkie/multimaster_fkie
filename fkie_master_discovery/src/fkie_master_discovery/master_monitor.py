# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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
try:
    from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from SocketServer import ThreadingMixIn
    import cStringIO as io  # python 2 compatibility
except ImportError:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from socketserver import ThreadingMixIn
    import io
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse
from datetime import datetime
import getpass
import roslib.network
import roslib.message
import rospy
import socket
import subprocess
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

from . import interface_finder

from .common import masteruri_from_ros, get_hostname
from .common import gen_pattern
from .filter_interface import FilterInterface
from .master_info import MasterInfo


try:  # to avoid the problems with autodoc on ros.org/wiki site
    from fkie_multimaster_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo, SyncServiceInfo
    from fkie_multimaster_msgs.srv import DiscoverMasters, GetSyncInfo
except:
    pass


class MasterConnectionException(Exception):
    '''
    The exception class to handle the connection problems with ROS Master.
    '''
    pass


def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise Exception("remote call failed: %s" % msg)
    return val


class RPCThreading(ThreadingMixIn, SimpleXMLRPCServer):
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class RPCThreadingV6(ThreadingMixIn, SimpleXMLRPCServer):
    address_family = socket.AF_INET6
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class MasterMonitor(object):
    '''
    This class provides methods to get the state from the ROS master using his
    RPC API and test for changes. Furthermore an XML-RPC server will be created
    to offer the complete current state of the ROS master by one method call.

    :param rpcport: the port number for the XML-RPC server

    :type rpcport:  int

    :param do_retry: retry to create XML-RPC server

    :type do_retry: bool

    :see: :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getCurrentState()`, respectively
          :mod:`fkie_master_discovery.master_monitor.MasterMonitor.updateState()`

    :RPC Methods:
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getListedMasterInfo()` or
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getMasterContacts()` as RPC:
        ``masterInfo()`` and ``masterContacts()``
    '''

    MAX_PING_SEC = 10.0
    ''' The time to update the node URI, ID or service URI (Default: ``10.0``)'''

    INTERVAL_UPDATE_LAUNCH_URIS = 15.0

    def __init__(self, rpcport=11611, do_retry=True, ipv6=False, rpc_addr=''):
        '''
        Initialize method. Creates an XML-RPC server on given port and starts this
        in its own thread.

        :param rpcport: the port number for the XML-RPC server

        :type rpcport:  int

        :param do_retry: retry to create XML-RPC server

        :type do_retry: bool

        :param ipv6: Use ipv6

        :type ipv6: bool
        '''
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri_from_ros()
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = None
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = str(rospy.get_name())
        if rospy.has_param('~name'):
            self.__mastername = rospy.get_param('~name')
        self.__mastername = self.getMastername()
        rospy.set_param('/mastername', self.__mastername)

        self.__master_state = None
        '''the current state of the ROS master'''
        self.rpcport = rpcport
        '''the port number of the RPC server'''

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()

        self._master_errors = list()
        # Create an XML-RPC server
        self.ready = False
        while not self.ready and not rospy.is_shutdown():
            try:
                RPCClass = RPCThreading
                if ipv6:
                    RPCClass = RPCThreadingV6
                self.rpcServer = RPCClass((rpc_addr, rpcport), logRequests=False, allow_none=True)
                rospy.loginfo("Start RPC-XML Server at %s", self.rpcServer.server_address)
                self.rpcServer.register_introspection_functions()
                self.rpcServer.register_function(self.getListedMasterInfo, 'masterInfo')
                self.rpcServer.register_function(self.getListedMasterInfoFiltered, 'masterInfoFiltered')
                self.rpcServer.register_function(self.getMasterContacts, 'masterContacts')
                self.rpcServer.register_function(self.getMasterErrors, 'masterErrors')
                self.rpcServer.register_function(self.getCurrentTime, 'getCurrentTime')
                self.rpcServer.register_function(self.setTime, 'setTime')
                self.rpcServer.register_function(self.getTopicsMd5sum, 'getTopicsMd5sum')
                self.rpcServer.register_function(self.getUser, 'getUser')
                self._rpcThread = threading.Thread(target=self.rpcServer.serve_forever)
                self._rpcThread.setDaemon(True)
                self._rpcThread.start()
                self.ready = True
            except socket.error as e:
                if not do_retry:
                    raise Exception("Error while start RPC-XML server on port %d: %s\nIs a Node Manager already running?" % (rpcport, e))
                rospy.logwarn("Error while start RPC-XML server on port %d: %s\nTry again..." % (rpcport, e))
                time.sleep(1)
            except:
                print(traceback.format_exc())
                if not do_retry:
                    raise

        self._master = xmlrpcclient.ServerProxy(self.getMasteruri())
        # Hide parameter
        self._re_hide_nodes = gen_pattern(rospy.get_param('~hide_nodes', []), 'hide_nodes')
        self._re_hide_topics = gen_pattern(rospy.get_param('~hide_topics', []), 'hide_topics')
        self._re_hide_services = gen_pattern(rospy.get_param('~hide_services', []), 'hide_services')
        # === UPDATE THE LAUNCH URIS Section ===
        # subscribe to get parameter updates
        rospy.loginfo("Subscribe to parameter `/roslaunch/uris`")
        self.__mycache_param_server = rospy.impl.paramserver.get_param_server_cache()
        # HACK: use own method to get the updates also for parameters in the subgroup
        self.__mycache_param_server.update = self.__update_param
        # first access, make call to parameter server
        self._update_launch_uris_lock = threading.RLock()
        self.__launch_uris = {}
        code, msg, value = self._master.subscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
        # the new timer will be created in self._update_launch_uris()
        self._timer_update_launch_uris = None
        if code == 1:
            for k, v in value.items():
                self.__launch_uris[roslib.names.ns_join('/roslaunch/uris', k)] = v
        self._update_launch_uris()
        # === END: UPDATE THE LAUNCH URIS Section ===

    def __update_param(self, key, value):
        # updates the /roslaunch/uris parameter list
        with self._update_launch_uris_lock:
            try:
                if value:
                    self.__launch_uris[key] = value
                else:
                    del self.__launch_uris[key]
            except:
                pass

    def shutdown(self):
        '''
        Shutdown the RPC Server.
        '''
        if self._timer_update_launch_uris is not None:
            try:
                self._timer_update_launch_uris.cancel()
            except Exception:
                pass
        if hasattr(self, 'rpcServer'):
            if self._master is not None:
                rospy.loginfo("Unsubscribe from parameter `/roslaunch/uris`")
                try:
                    self._master.unsubscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
                except Exception as e:
                    rospy.logwarn("Error while unsubscribe from `/roslaunch/uris`: %s" % e)
            rospy.loginfo("shutdown own RPC server")
            self.rpcServer.shutdown()
            del self.rpcServer.socket
            del self.rpcServer

    def is_running(self):
        return hasattr(self, 'rpcServer')

    def _update_launch_uris(self, params={}):
        with self._update_launch_uris_lock:
            if params:
                self.__launch_uris = params
            try:
                socket.setdefaulttimeout(3.0)
                for key, value in self.__launch_uris.items():
                    try:
                        # contact the launch server
                        launch_server = xmlrpcclient.ServerProxy(value)
                        c, m, pid = launch_server.get_pid()
                    except:
                        try:
                            # remove the parameter from parameter server on error
                            master = xmlrpcclient.ServerProxy(self.getMasteruri())
                            master.deleteParam(self.ros_node_name, key)
                        except:
                            pass
            finally:
                socket.setdefaulttimeout(None)
                # create the new timer
                if not rospy.is_shutdown():
                    self._timer_update_launch_uris = threading.Timer(self.INTERVAL_UPDATE_LAUNCH_URIS, self._update_launch_uris)
                    self._timer_update_launch_uris.start()

    def _getNodePid(self, nodes):
        '''
        Gets process id of the node.
        This method blocks until the info is retrieved or socket timeout is reached (0.7 seconds).

        :param nodename: the name of the node

        :type nodename: str

        :param uri: the uri of the node

        :type uri: str
        '''
        for (nodename, uri) in nodes.items():
            if uri is not None:
                pid = None
                try:
                    with self._lock:
                        if nodename in self.__cached_nodes:
                            if time.time() - self.__cached_nodes[nodename][2] < self.MAX_PING_SEC:
                                return
                    socket.setdefaulttimeout(0.7)
                    node = xmlrpcclient.ServerProxy(uri)
                    pid = _succeed(node.getPid(self.ros_node_name))
                except (Exception, socket.error) as e:
                    with self._lock:
                        self._limited_log(nodename, "can't get PID: %s" % str(e), level=rospy.DEBUG)
                    master = xmlrpcclient.ServerProxy(self.getMasteruri())
                    code, message, new_uri = master.lookupNode(self.ros_node_name, nodename)
                    with self._lock:
                        self.__new_master_state.getNode(nodename).uri = None if (code == -1) else new_uri
                        if code == -1:
                            self._limited_log(nodename, "can't update contact information. ROS master responds with: %s" % message)
                        try:
                            del self.__cached_nodes[nodename]
                        except:
                            pass
                else:
                    with self._lock:
                        self.__new_master_state.getNode(nodename).pid = pid
                        self.__cached_nodes[nodename] = (uri, pid, time.time())
#          print "_getNodePid _lock RET", threading.current_thread()
                finally:
                    socket.setdefaulttimeout(None)

    def _getServiceInfo(self, services):
        '''
        Gets service info through the RPC interface of the service.
        This method blocks until the info is retrieved or socket timeout is reached (0.5 seconds).

        :param service: the name of the service

        :type service: str

        :param uri: the uri of the service

        :type uri: str
        '''
        for (service, uri) in services.items():
            with self._lock:
                if service in self.__cached_services:
                    if time.time() - self.__cached_services[service][2] < self.MAX_PING_SEC:
                        return
            if uri is not None:
                dest_addr = dest_port = None
                try:
                    dest_addr, dest_port = rospy.parse_rosrpc_uri(uri)
                except:
                    continue
        #      raise ROSServiceException("service [%s] has an invalid RPC URI [%s]"%(service, uri))
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    # connect to service and probe it to get the headers
                    s.settimeout(0.5)
                    s.connect((dest_addr, dest_port))
                    header = {'probe': '1', 'md5sum': '*',
                              'callerid': self.ros_node_name, 'service': service}
                    roslib.network.write_ros_handshake_header(s, header)
                    buf = io.StringIO() if sys.version_info < (3, 0) else io.BytesIO()
                    stype = roslib.network.read_ros_handshake_header(s, buf, 2048)
                    with self._lock:
                        self.__new_master_state.getService(service).type = stype['type']
                        self.__cached_services[service] = (uri, stype['type'], time.time())
                except socket.error:
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
        #      raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service, uri))
                except:
                    with self._lock:
                        self._limited_log(service, "can't get service type: %s" % traceback.format_exc(), level=rospy.DEBUG)
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
                    pass
                finally:
                    if s is not None:
                        s.close()

    def getListedMasterInfo(self):
        '''
        :return: a extended ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
        t = str(time.time())
        result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [])
        if not (self.__master_state is None):
            try:
                with self._state_access_lock:
                    result = self.__master_state.listedState()
            except:
                print(traceback.format_exc())
        return result

    def getListedMasterInfoFiltered(self, filter_list):
        '''
        :return: a extended filtered ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
        t = str(time.time())
        result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [])
        if not (self.__master_state is None):
            try:
                with self._state_access_lock:
                    fi = FilterInterface.from_list(filter_list)
                    fi.set_hide_pattern(self._re_hide_nodes, self._re_hide_topics, self._re_hide_services)
                    result = self.__master_state.listedState(fi)
            except:
                print(traceback.format_exc())
        return result

    def getCurrentState(self):
        '''
        :return: The current ROS Master State

        :rtype: :mod:`fkie_master_discovery.master_info.MasterInfo` or ``None``
        '''
        with self._state_access_lock:
            return self.__master_state

    def updateState(self, clear_cache=False):
        '''
        Gets state from the ROS Master through his RPC interface.

        :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                            If remote hosted nodes or services was restarted, the cache must
                            be cleared! The local nodes will be updated periodically after
                            :mod:`fkie_master_discovery.master_monitor.MasterMonitor.MAX_PING_SEC`.

        :type clear_cache: bool (Default: ``False``)

        :rtype: :mod:`fkie_master_discovery.master_info.MasterInfo`

        :raise: ``MasterConnectionException``, if not complete information was get from the ROS master.
        '''
        with self._create_access_lock:
            now = time.time()
            threads = []
            try:
                self._lock.acquire(True)
                if clear_cache:
                    self.__cached_nodes = dict()
                    self.__cached_services = dict()
                socket.setdefaulttimeout(5)
                self.__new_master_state = master_state = MasterInfo(self.getMasteruri(), self.getMastername())
                # update master state
                master = self._master
                # master = xmlrpclib.ServerProxy(self.getMasteruri())
                # get topic types
                code, message, topicTypes = master.getTopicTypes(self.ros_node_name)
                # convert topicType list to the dict
                topicTypesDict = {}
                for topic, type in topicTypes:
                    topicTypesDict[topic] = type
                # get system state
                code, message, state = master.getSystemState(self.ros_node_name)

                # add published topics
                for t, l in state[0]:
                    master_state.topics = t
                    for n in l:
                        master_state.nodes = n
                        master_state.getNode(n).publishedTopics = t
                        master_state.getTopic(t).publisherNodes = n
                        master_state.getTopic(t).type = topicTypesDict.get(t, 'None')
                # add subscribed topics
                for t, l in state[1]:
                    master_state.topics = t
                    for n in l:
                        master_state.nodes = n
                        master_state.getNode(n).subscribedTopics = t
                        master_state.getTopic(t).subscriberNodes = n
                        master_state.getTopic(t).type = topicTypesDict.get(t, 'None')

                # add services
                services = dict()
                tmp_slist = []
                # multi-call style xmlrpc to lock up the service uri
                param_server_multi = xmlrpcclient.MultiCall(master)
                for t, l in state[2]:
                    master_state.services = t
                    for n in l:
                        master_state.nodes = n
                        master_state.getNode(n).services = t
                        service = master_state.getService(t)
                        service.serviceProvider = n
                        if service.name in self.__cached_services:
                            service.uri = self.__cached_services[service.name][0]
                            service.type = self.__cached_services[service.name][1]
                            if service.isLocal and time.time() - self.__cached_services[service.name][2] > self.MAX_PING_SEC:
                                services[service.name] = service.uri
                        else:
                            tmp_slist.append(service)
                            param_server_multi.lookupService(self.ros_node_name, t)
                try:
                    r = param_server_multi()
                    for (code, msg, uri), service in zip(r, tmp_slist):
                        if code == 1:
                            service.uri = uri
                            if service.isLocal:
                                services[service.name] = uri
                            else:
                                self.__cached_services[service.name] = (uri, None, time.time())
                        else:
                            with self._lock:
                                self._limited_log(service.name, "can't get contact information. ROS master responds with: %s" % msg)
                except:
                    traceback.print_exc()
                if services:
                    pidThread = threading.Thread(target=self._getServiceInfo, args=((services,)))
                    pidThread.start()
                    threads.append(pidThread)

                # get additional node information
                nodes = dict()
                try:
                    # multi-call style xmlrpc to loock up the node uri
                    param_server_multi = xmlrpcclient.MultiCall(master)
                    tmp_nlist = []
                    for name, node in master_state.nodes.items():
                        if node.name in self.__cached_nodes:
                            node.uri = self.__cached_nodes[node.name][0]
                            node.pid = self.__cached_nodes[node.name][1]
                            if node.isLocal and time.time() - self.__cached_nodes[node.name][2] > self.MAX_PING_SEC:
                                nodes[node.name] = node.uri
                        else:
                            # 'print "request node:", node.name
                            tmp_nlist.append(node)
                            param_server_multi.lookupNode(self.ros_node_name, name)
                    r = param_server_multi()
                    for (code, msg, uri), node in zip(r, tmp_nlist):
                        if code == 1:
                            node.uri = uri
                            if node.isLocal:
                                nodes[node.name] = uri
                            else:
                                self.__cached_nodes[node.name] = (uri, None, time.time())
                        else:
                            with self._lock:
                                self._limited_log(node.name, "can't get contact information. ROS master responds with: %s" % msg)
                except:
                    traceback.print_exc()

                if nodes:
                    # get process id of the nodes
                    pidThread = threading.Thread(target=self._getNodePid, args=((nodes,)))
                    pidThread.start()
                    threads.append(pidThread)

                master_state.timestamp = now
            except socket.error as e:
                if isinstance(e, tuple):
                    (errn, msg) = e
                    if errn not in [100, 101, 102]:
                        formatted_lines = traceback.format_exc().splitlines()
                        raise MasterConnectionException(formatted_lines[-1])
                else:
                    raise MasterConnectionException(traceback.format_exc(1))
            except:
                formatted_lines = traceback.format_exc().splitlines()
                raise MasterConnectionException(formatted_lines[-1])
            finally:
                self._lock.release()
                socket.setdefaulttimeout(None)

            # wait for all threads are finished
            while threads:
                th = threads.pop()
                if th.is_alive():
                    th.join()
                del th
            if time.time() - self._last_clearup_ts > 300:
                self._last_clearup_ts = time.time()
                self._clearup_cached_logs()
            return master_state

    def _limited_log(self, provider, msg, level=rospy.WARN):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            if level == rospy.DEBUG:
                rospy.logdebug("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.INFO:
                rospy.loginfo("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.WARN:
                rospy.logwarn("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.ERROR:
                rospy.logerr("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.FATAL:
                rospy.logfatal("MasterMonitor[%s]: %s" % (provider, msg))

    def _clearup_cached_logs(self, age=300):
        cts = time.time()
        with self._lock:
            for p, msgs in list(self._printed_errors.items()):
                for msg, ts in list(msgs.items()):
                    if cts - ts > age:
                        del self._printed_errors[p][msg]
                if not self._printed_errors[p]:
                    del self._printed_errors[p]

    def updateSyncInfo(self):
        '''
        This method can be called to update the origin ROS master URI of the nodes
        and services in new ``master_state``. This is only need, if a synchronization is
        running. The synchronization service will be detect automatically by searching
        for the service ending with ``get_sync_info``. The method will be called by
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.checkState()`.
        '''
        # 'print "updateSyncInfo _create_access_lock try...", threading.current_thread()

        def getNodeuri(nodename, publisher, subscriber, services):
            for p in publisher:
                if nodename == p.node:
                    return p.nodeuri
            for p in subscriber:
                if nodename == p.node:
                    return p.nodeuri
            for s in services:
                if nodename == s.node:
                    return s.nodeuri
            return None

        with self._create_access_lock:
            master_state = self.__new_master_state
            sync_info = None
            # get synchronization info, if sync node is running
            # to determine the origin ROS MASTER URI of the nodes
            for name, service in master_state.services.items():
                if service.name.endswith('get_sync_info'):
                    if get_hostname(self.getMasteruri()) == get_hostname(service.uri):
                        socket.setdefaulttimeout(3)
                        get_sync_info = rospy.ServiceProxy(service.name, GetSyncInfo)
                        try:
                            sync_info = get_sync_info()
                        except rospy.ServiceException as e:
                            rospy.logwarn("ERROR Service call 'get_sync_info' failed: %s", str(e))
                        finally:
                            socket.setdefaulttimeout(None)

            # update the origin ROS MASTER URI of the nodes, if sync node is running
            if sync_info:
                for m in sync_info.hosts:
                    for n in m.nodes:
                        try:
                            # TODO: add nodeuri to the nodes (needs changes in the MSG definitions)
                            # set the sync node only if it has the same uri
                            nuri = getNodeuri(n, m.publisher, m.subscriber, m.services)
                            state_node = master_state.getNode(n)
                            if state_node is not None and (state_node.uri == nuri or nuri is None):
                                state_node.masteruri = m.masteruri
                        except:
                            pass
                    for s in m.services:
                        try:
                            state_service = master_state.getService(s.service)
                            if state_service is not None and state_service.uri == s.serviceuri:
                                state_service.masteruri = m.masteruri
                        except:
                            pass

    def getMasteruri(self):
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it.

        :return: ROS master URI

        :rtype: str or ``None``
        '''
        code = -1
        if self.__masteruri_rpc is None:
            master = xmlrpcclient.ServerProxy(self.__masteruri)
            code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
        return self.__masteruri_rpc if code >= 0 or self.__masteruri_rpc is not None else self.__masteruri

    def getMastername(self):
        '''
        Returns the name of the master. If no name is set, the hostname of the
        ROS master URI will be extracted.

        :return: the name of the ROS master

        :rtype: str or ``None``
        '''
        if self.__mastername is None:
            try:
                self.__mastername = get_hostname(self.getMasteruri())
                try:
                    master_port = urlparse(self.__masteruri).port
                    if master_port != 11311:
                        self.__mastername = '%s_%d' % (self.__mastername, master_port)
                except:
                    pass
            except:
                pass
        return self.__mastername

    def getMasterContacts(self):
        '''
        The RPC method called by XML-RPC server to request the master contact information.

        :return: (``timestamp of the ROS master state``, ``ROS master URI``, ``master name``, ``name of this service``, ``URI of this RPC server``)
        :rtype: (str, str, str, str, str)
        '''
        t = 0
        if self.__master_state is not None:
            with self._state_access_lock:
                t = self.__master_state.timestamp
        return ('%.9f' % t, str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, roslib.network.create_local_xmlrpc_uri(self.rpcport))

    def getMasterErrors(self):
        '''
        The RPC method called by XML-RPC server to request the occured network errors.

        :return: (``ROS master URI``, ``list with errors``)
        :rtype: (str, [str])
        '''
        return (str(self.getMasteruri()), self._master_errors)

    def getCurrentTime(self):
        '''
        The RPC method called by XML-RPC server to request the current host time.

        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        return (str(self.getMasteruri()), time.time())

    def setTime(self, timestamp):
        '''
        The RPC method called by XML-RPC server to set new host time.
        :param timestamp: UNIX timestamp
        :type timestamp: float
        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        dtime = datetime.fromtimestamp(timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        rospy.loginfo('Set time: %s' % args)
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        success = True
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            if result_err:
                success = False
        return (str(self.getMasteruri()), success, time.time(), result_err)

    def getTopicsMd5sum(self, topic_types):
        '''
        :return: a list with topic type and current md5sum.

                - ``topic types`` is of the form

                    ``[ (topic1, md5sum1) ... ]``

        :rtype:  list
        '''
        topic_list = []
        for ttype in topic_types:
            try:
                entry = (ttype, roslib.message.get_message_class(ttype)._md5sum)
                topic_list.append(entry)
            except Exception as err:
                rospy.logwarn(err)
        return topic_list

    def getUser(self):
        '''
        The RPC method called by XML-RPC server to request the user name used to launch the master_discovery.

        :return: (``ROS master URI``, ``user name``)
        :rtype: (str, str)
        '''
        return (str(self.getMasteruri()), getpass.getuser())


    def checkState(self, clear_cache=False):
        '''
        Gets the state from the ROS master and compares it to the stored state.

        :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                            If remote hosted nodes or services was restarted, the cache must
                            be cleared! The local nodes will be updated periodically after
                            :mod:`fkie_master_discovery.master_monitor.MasterMonitor.MAX_PING_SEC`.

        :type clear_cache: bool (Default: ``False``)

        :return: ``True`` if the ROS master state is changed

        :rtype: bool
        '''
        result = False
        s = self.updateState(clear_cache)
        with self._create_access_lock:
            do_update = False
            with self._state_access_lock:
                if s != self.__master_state:
                    do_update = True
                if self.__master_state is not None and s.timestamp < self.__master_state.timestamp:
                    do_update = True
                    result = True
                    timejump_msg = "Timejump into past detected! Restart all ROS nodes, includes master_discovery, please!"
                    rospy.logwarn(timejump_msg)
                    if timejump_msg not in self._master_errors:
                        self._master_errors.append(timejump_msg)
                    self._exit_timer = threading.Timer(5.0, self._timejump_exit)
                    self._exit_timer.start()
            if do_update:
                self.updateSyncInfo()
                with self._state_access_lock:
                    # test for local changes
                    ts_local = self.__new_master_state.timestamp_local
                    if self.__master_state is not None and not self.__master_state.has_local_changes(s):
                        ts_local = self.__master_state.timestamp_local
                    self.__master_state = self.__new_master_state
                    self.__master_state.timestamp_local = ts_local
                    result = True
            self.__master_state.check_ts = self.__new_master_state.timestamp
            return result

    def _timejump_exit(self):
        rospy.logwarn('Shutdown yourself to avoid system instability because of time jump into past!\n')
        rospy.signal_shutdown('Shutdown yourself to avoid system instability because of time jump into past')

    def reset(self):
        '''
        Sets the master state to ``None``.
        '''
        with self._state_access_lock:
            if self.__master_state is not None:
                del self.__master_state
            self.__master_state = None

    def update_master_errors(self, error_list):
        self._master_errors = list(error_list)
